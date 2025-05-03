#include "../include/Simulation.h"
#include "../include/Config.h"
#include <algorithm>
#include <random>
#include <thread>
#include <iostream>
#include <cmath>

Simulation::Simulation(const Config& config)
    : fieldSize(config.field_size),
      timeStep(config.time_step),
      containmentField(std::make_unique<ContainmentField>(config)),
      threadManager(std::make_unique<ThreadManager>(config.initial_threads)),
      numThreads(config.initial_threads) {
    // Initialize with the correct number of threads from the config
    initializeParticles(config);
}

Simulation::~Simulation() {
    stop();
}

void Simulation::initializeParticles(const Config& config) {
    std::random_device rd;
    std::mt19937 gen(config.random_seed > 0 ? config.random_seed : rd());
    std::uniform_real_distribution<> posDis(-fieldSize/2, fieldSize/2);
    std::uniform_real_distribution<> velDis(-1.0, 1.0); // Velocity range
    
    // Use the correct number of particles from config
    size_t count = config.num_particles;
    for (size_t i = 0; i < count; ++i) {
        auto particle = std::make_unique<Particle>(
            posDis(gen), posDis(gen),
            config.initial_energy,
            config.particle_radius,
            config.max_energy
        );
        particle->setVelocity(velDis(gen), velDis(gen));
        particles.push_back(std::move(particle));
    }
    std::cout << "Initialized " << particles.size() << " particles." << std::endl;
}

void Simulation::setContainmentField(std::unique_ptr<ContainmentField> field) {
    std::lock_guard<std::mutex> lock(simulationMutex);
    containmentField = std::move(field);
}

void Simulation::start() {
    std::lock_guard<std::mutex> lock(simulationMutex);
    if (running) return;
    
    running = true;
    threadManager->start();
    std::cout << "Simulation started with " << numThreads << " threads." << std::endl;
}

void Simulation::stop() {
    {
        std::lock_guard<std::mutex> lock(simulationMutex);
        if (!running) return;
        running = false;
    }
    
    threadManager->stop();
    std::cout << "Simulation stopped." << std::endl;
}

void Simulation::step() {
    // Use the thread manager to parallelize work
    removeEscapedParticles();
    
    // Update particle positions using threads
    updatePositions(timeStep);
    
    // Apply containment forces
    applyForces(timeStep);
    
    // Handle collisions between particles
    handleCollisions();
    
    // Wait for all thread tasks to complete
    threadManager->waitForCompletion();
}

void Simulation::addParticle(std::unique_ptr<Particle> particle) {
    std::lock_guard<std::mutex> lock(particleMutex);
    particles.push_back(std::move(particle));
}

void Simulation::removeEscapedParticles() {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Remove particles that are outside the containment field
    particles.erase(
        std::remove_if(particles.begin(), particles.end(), 
            [this](const std::unique_ptr<Particle>& p) {
                return !containmentField->isParticleContained(*p);
            }
        ),
        particles.end()
    );
}

size_t Simulation::getParticleCount() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return particles.size();
}

const std::vector<std::unique_ptr<Particle>>& Simulation::getParticles() const {
    return particles;
}

double Simulation::getTotalEnergy() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    double total = 0.0;
    for (const auto& particle : particles) {
        total += particle->getEnergy();
    }
    return total;
}

void Simulation::setNumThreads(size_t newNumThreads) {
    std::lock_guard<std::mutex> lock(simulationMutex);
    numThreads = newNumThreads;
    threadManager->setNumThreads(newNumThreads);
}

size_t Simulation::getNumThreads() const {
    return numThreads;
}

void Simulation::updatePositions(double dt) {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Distribute work across threads using a simple partitioning
    size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
    
    for (size_t startIdx = 0; startIdx < particles.size(); startIdx += particlesPerThread) {
        size_t endIdx = std::min(startIdx + particlesPerThread, particles.size());
        
        // Create a task for this range of particles
        threadManager->addTask([this, startIdx, endIdx, dt]() {
            for (size_t i = startIdx; i < endIdx; ++i) {
                // Update position based on velocity
                double x = particles[i]->getX() + particles[i]->getVX() * dt;
                double y = particles[i]->getY() + particles[i]->getVY() * dt;
                particles[i]->setPosition(x, y);
            }
        });
    }
}

void Simulation::handleCollisions() {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Use a spatial partitioning approach to reduce collision checks
    // For simplicity here, we'll just use a more efficient nested loop
    
    // Distribute collision checks across threads
    size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
    
    for (size_t startIdx = 0; startIdx < particles.size(); startIdx += particlesPerThread) {
        size_t endIdx = std::min(startIdx + particlesPerThread, particles.size());
        
        threadManager->addTask([this, startIdx, endIdx]() {
            for (size_t i = startIdx; i < endIdx; ++i) {
                for (size_t j = i + 1; j < particles.size(); ++j) {
                    if (particles[i]->isColliding(*particles[j])) {
                        particles[i]->collide(*particles[j]);
                    }
                }
            }
        });
    }
}

void Simulation::applyForces(double dt) {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    // Distribute work across threads
    size_t particlesPerThread = std::max(size_t(1), particles.size() / numThreads);
    
    for (size_t startIdx = 0; startIdx < particles.size(); startIdx += particlesPerThread) {
        size_t endIdx = std::min(startIdx + particlesPerThread, particles.size());
        
        threadManager->addTask([this, startIdx, endIdx, dt]() {
            for (size_t i = startIdx; i < endIdx; ++i) {
                double x = particles[i]->getX();
                double y = particles[i]->getY();
                
                // Calculate direction to center (for containment force)
                double distance = std::sqrt(x*x + y*y);
                
                // Calculate containment force
                double forceMagnitude = containmentField->getContainmentForce(*particles[i]);
                
                // Direction is toward the center of the field (0,0)
                double forceDirectionX = (distance > 1e-10) ? -x / distance : 0;
                double forceDirectionY = (distance > 1e-10) ? -y / distance : 0;
                
                // Calculate acceleration from force
                double ax = forceMagnitude * forceDirectionX;
                double ay = forceMagnitude * forceDirectionY;
                
                // Update velocity based on acceleration
                double vx = particles[i]->getVX() + ax * dt;
                double vy = particles[i]->getVY() + ay * dt;
                
                particles[i]->setVelocity(vx, vy);
            }
        });
    }
}

void Simulation::workerThread(size_t threadId) {
    while (running) {
        // Perform simulation steps when running
        if (threadManager->isRunning()) {
            step();
        }
        
        // Reasonable delay to prevent CPU overuse
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}