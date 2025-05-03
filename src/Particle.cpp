#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>
#include <algorithm> // For std::min

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
    // Fix initial energy to be the correct value from constructor parameter
}

Particle::~Particle() {
    // Nothing to clean up
}

double Particle::getX() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return x; // Fixed: removed arbitrary multiplier
}

double Particle::getY() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return y; // Fixed: removed arbitrary multiplier
}

void Particle::setPosition(double newX, double newY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX;  // Fixed: removed arbitrary multiplier
    y = newY;
}

double Particle::getVX() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return vx; // Fixed: removed arbitrary multiplier
}

double Particle::getVY() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return vy; // Fixed: removed arbitrary multiplier
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    std::lock_guard<std::mutex> lock(particleMutex);
    return energy; // Fixed: removed arbitrary multiplier
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY; // Fixed: return correct max energy
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Ensure energy doesn't exceed maximum
    energy = std::min(newEnergy, MAX_ENERGY);
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    // Implement the missing functionality
    energy = std::min(energy + delta, MAX_ENERGY);
}

void Particle::collide(Particle& other) {
    // Lock both particles to prevent race conditions
    // Use std::lock to avoid deadlocks when locking multiple mutexes
    std::lock(particleMutex, other.particleMutex);
    std::lock_guard<std::mutex> lock1(particleMutex, std::adopt_lock);
    std::lock_guard<std::mutex> lock2(other.particleMutex, std::adopt_lock);
    
    // Conservation of momentum in elastic collisions
    double tempVX = vx;
    double tempVY = vy;
    
    vx = other.vx;
    vy = other.vy;
    
    other.vx = tempVX;
    other.vy = tempVY;
    
    // Small energy loss in collision (10%)
    double energyLoss = 0.1;
    energy *= (1.0 - energyLoss);
    other.energy *= (1.0 - energyLoss);
}

bool Particle::isColliding(const Particle& other) const {
    // Thread-safe access to position
    double thisX, thisY, otherX, otherY;
    {
        std::lock_guard<std::mutex> lock(particleMutex);
        thisX = x;
        thisY = y;
    }
    
    {
        std::lock_guard<std::mutex> lock(other.particleMutex);
        otherX = other.x;
        otherY = other.y;
    }
    
    // Calculate distance between particles
    double dx = thisX - otherX;
    double dy = thisY - otherY;
    double distanceSquared = dx * dx + dy * dy;
    
    // Check if distance is less than sum of radii
    double collisionDistance = PARTICLE_RADIUS + other.PARTICLE_RADIUS;
    return distanceSquared < (collisionDistance * collisionDistance);
}
