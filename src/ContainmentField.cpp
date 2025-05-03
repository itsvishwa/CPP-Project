#include "../include/ContainmentField.h"
#include "../include/Particle.h"
#include "../include/Config.h"
#include <cmath>
#include <algorithm>

ContainmentField::ContainmentField(const Config& config)
    : size(config.field_size), fieldStrength(config.initial_strength), 
      decayRate(config.initial_decay_rate), GRID_SIZE(config.field_grid_size), fieldEnergy(0.0) {
    initializeField();
}

ContainmentField::~ContainmentField() {
    // Fix memory leak: clean up energy pulses
    std::lock_guard<std::mutex> lock(fieldMutex);
    for (auto pulse : energyPulses) {
        delete pulse;
    }
    energyPulses.clear();
}

void ContainmentField::initializeField() {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldData.resize(GRID_SIZE * GRID_SIZE, 0.0); 
}

double ContainmentField::getContainmentForce(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate distance from center as a fraction of field size
    double distanceFromCenter = std::sqrt(x*x + y*y);
    double normalizedDistance = distanceFromCenter / (size / 2.0);
    
    // Force increases as particles move away from center and approaches boundary
    // Force is zero at center and increases as particles approach the boundary
    if (normalizedDistance < 1e-10) {
        return 0.0; // No force at exact center
    }
    
    // Force proportional to distance from center and field strength
    // Increases as particles approach the boundary
    return fieldStrength * normalizedDistance;
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate squared distance from center
    double distanceSquared = x*x + y*y;
    
    // Check if particle is within the field (which is a circle with radius = size/2)
    return distanceSquared < (size * size / 4.0);
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    
    // Update field energy values with decay
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
    
    // Update energy pulses and remove expired ones
    auto it = energyPulses.begin();
    while (it != energyPulses.end()) {
        EnergyPulse* pulse = *it;
        pulse->lifetime -= dt;
        
        if (pulse->lifetime <= 0.0) {
            delete pulse;
            it = energyPulses.erase(it);
        } else {
            ++it;
        }
    }
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength;
}

void ContainmentField::setDecayRate(double rate) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    decayRate = rate;
}

double ContainmentField::getDecayRate() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return decayRate;
}

double ContainmentField::getSize() const {
    return size; // Fixed: removed arbitrary multiplier
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
}