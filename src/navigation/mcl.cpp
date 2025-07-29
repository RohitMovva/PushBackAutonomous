#include "navigation/mcl.hpp"
#include "utilities/logger.hpp"
#include "utilities/math/angle.hpp"
#include <algorithm>
#include <numeric>
#include <cmath>

// Particle implementation
void Particle::predict(const Pose& motion, double motionNoise)
{
    // Apply motion model with noise
    std::normal_distribution<double> noise(0.0, motionNoise);
    std::mt19937 gen(std::random_device{}());
    
    pose.x += motion.x + noise(gen);
    pose.y += motion.y + noise(gen);
    pose.theta += motion.theta;
    
    pose.theta = Angles::normalizeAngle(pose.theta);
}

void Particle::updateWeight(double likelihood)
{
    weight *= likelihood;
}

// MCL implementation
MCL::MCL(pros::MotorGroup &left, pros::MotorGroup &right,
                                pros::Rotation &lateral, pros::Imu &imuSensor,
                                std::vector<Distance>& distanceSensors,
                                int particle_count, double motion_noise_std)
    : numParticles(particle_count)
    , resampleThreshold(0.5)
    , confidence(0.0)
    , generator(randomDevice())
    , motionNoise(0.0, motion_noise_std)
    , imuSensor(imuSensor)
    , distanceSensors(distanceSensors) // Initialize to nullptr, will be set later if needed
{
    // Create internal motion model for particle prediction
    motionModel = std::make_unique<Odometry>(
        left, right, lateral, imuSensor,
        false, true  // Enable filters for better motion prediction
    );
    
    reset();
}

void MCL::initializeParticles()
{
    particles.clear();
    particles.reserve(numParticles);
    
    for (int i = 0; i < numParticles; ++i) {
        // Just create particles that are empty
        particles.emplace_back(
            0,
            0, 
            0,
            1.0 / numParticles 
        );
    }
}

void MCL::reset()
{
    estimatedPose = Pose();
    confidence = 0.0;
    leftVelocity = Velocity();
    rightVelocity = Velocity();
    
    if (motionModel) {
        motionModel->reset();
    }

    initializeParticles();
}

void MCL::update()
{
    if (motionModel) {
        motionModel->update();
        
        // Get velocities from motion model for interface compatibility
        leftVelocity = motionModel->getLeftVelocity();
        rightVelocity = motionModel->getRightVelocity();
    }
    
    predictParticles();
    
    updateParticlesWithSensors();
    
    // 4. Resample if needed
    if (shouldResample()) {
        resampleParticles();
    }
    
    // 5. Estimate pose from particles
    estimatedPose = estimatePoseFromParticles();
    
    // 6. Calculate confidence
    confidence = calculateEffectiveParticleCount() / numParticles;
    
    Logger::getInstance()->log("MCL Pose: %f %f %f (confidence: %f)", 
                              estimatedPose.x, estimatedPose.y, estimatedPose.theta, confidence);
}

void MCL::predictParticles()
{
    if (!motionModel) return;
    
    // Get motion from odometry (this should be the delta since last update)
    Pose motion = motionModel->getPose();
    
    // Apply motion model to each particle with noise
    for (auto& particle : particles) {
        particle.predict(motion, motionNoise.stddev());
    }
    
    // Reset motion model for next iteration to get deltas
    motionModel->setPose(Pose(0, 0, 0));
}

void MCL::updateParticlesWithSensors()
{
    // TODO: Implement sensor updates when sensors are available
    // For now, just maintain equal weights (pure odometry)
    
    // Example of how sensor updates would work:
    // for (auto& particle : particles) {
    //     double likelihood = calculateLikelihood(particle, sensorReading);
    //     particle.updateWeight(likelihood);
    // }
    
    // Placeholder: maintain current weights
    normalizeWeights();
}

void MCL::normalizeWeights()
{
    double totalWeight = 0.0;
    for (const auto& particle : particles) {
        totalWeight += particle.weight;
    }
    
    if (totalWeight > 0.0) {
        for (auto& particle : particles) {
            particle.weight /= totalWeight;
        }
    } else {
        // If all weights are zero, reset to uniform
        for (auto& particle : particles) {
            particle.weight = 1.0 / numParticles;
        }
    }
}

bool MCL::shouldResample() const
{
    return calculateEffectiveParticleCount() < (resampleThreshold * numParticles);
}

double MCL::calculateEffectiveParticleCount() const
{
    double sumSquaredWeights = 0.0;
    for (const auto& particle : particles) {
        sumSquaredWeights += particle.weight * particle.weight;
    }
    return sumSquaredWeights > 0.0 ? 1.0 / sumSquaredWeights : 0.0;
}

void MCL::resampleParticles()
{
    std::vector<Particle> newParticles;
    newParticles.reserve(numParticles);

    // Create cumulative distribution
    std::vector<double> cumulative(numParticles);
    cumulative[0] = particles[0].weight;
    for (size_t i = 1; i < particles.size(); ++i) {
        cumulative[i] = cumulative[i-1] + particles[i].weight;
    }

    // Systematic resampling
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double start = dist(generator) * (1.0 / numParticles);
    double step = 1.0 / numParticles;
    size_t index = 0;

    for (int i = 0; i < numParticles; ++i) {
        double sample = start + i * step;
        while (index < cumulative.size() - 1 && sample > cumulative[index]) {
            ++index;
        }
        newParticles.push_back(particles[index]);
        newParticles.back().weight = 1.0 / numParticles;
    }

    particles = std::move(newParticles);
}

Pose MCL::estimatePoseFromParticles()
{
    if (particles.empty()) return Pose();
    
    // Weighted average of particle poses
    double totalWeight = 0.0;
    double x = 0.0, y = 0.0;
    
    for (const auto& particle : particles) {
        totalWeight += particle.weight;
        x += particle.pose.x * particle.weight;
        y += particle.pose.y * particle.weight;
    }
    
    if (totalWeight > 0.0) {
        x /= totalWeight;
        y /= totalWeight;
    }
    
    return Pose(x, y, Angles::normalizeAngle(this->imuSensor.get_heading()));
}

// Core interface implementation
Pose MCL::getPose() const { return estimatedPose; }
void MCL::setPose(const Pose& newPose) {
    estimatedPose = newPose;
    
    // Reinitialize particles around new pose
    std::normal_distribution<double> xNoise(newPose.x, 1.0);
    std::normal_distribution<double> yNoise(newPose.y, 1.0);
    
    for (auto& particle : particles) {
        particle.pose.x = xNoise(generator);
        particle.pose.y = yNoise(generator);
        particle.pose.theta = newPose.theta;
        particle.weight = 1.0 / numParticles;
    }
}

double MCL::getHeading() const { return estimatedPose.theta; }
double MCL::getX() const { return estimatedPose.x; }
double MCL::getY() const { return estimatedPose.y; }
Velocity MCL::getLeftVelocity() const { return leftVelocity; }
Velocity MCL::getRightVelocity() const { return rightVelocity; }

bool MCL::isReliable() const
{
    // MCL is reliable if we have good confidence and motion model is reliable
    return confidence > 0.3 && 
           (motionModel ? motionModel->isReliable() : true);
}

// MCL-specific methods
void MCL::setParticleCount(int count)
{
    numParticles = count;
    initializeParticles();
}

void MCL::setMotionNoiseStd(double std)
{
    motionNoise = std::normal_distribution<double>(0.0, std);
}

MCL::DebugInfo MCL::getMCLDebugInfo() const
{
    if (particles.empty()) {
        return {0, 0.0, 0.0, motionNoise.stddev(), Pose(), Pose(), 0.0};
    }
    
    // Find best and worst particles
    auto minWeightIt = std::min_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    auto maxWeightIt = std::max_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    
    // Calculate weight variance
    double meanWeight = 1.0 / numParticles;  // Should be this after normalization
    double variance = 0.0;
    for (const auto& particle : particles) {
        double diff = particle.weight - meanWeight;
        variance += diff * diff;
    }
    variance /= numParticles;
    
    return {
        (int)particles.size(),
        calculateEffectiveParticleCount(),
        confidence,
        motionNoise.stddev(),
        maxWeightIt->pose,
        minWeightIt->pose,
        variance
    };
}

std::unordered_map<std::string, double> MCL::getDebugData() const
{
    std::unordered_map<std::string, double> debug;
    debug["particle_count"] = numParticles;
    debug["effective_particles"] = calculateEffectiveParticleCount();
    debug["confidence"] = confidence;
    debug["motion_noise_std"] = motionNoise.stddev();
    debug["resample_threshold"] = resampleThreshold;
    debug["reliable"] = isReliable() ? 1.0 : 0.0;
    debug["left_velocity"] = leftVelocity.linear;
    debug["right_velocity"] = rightVelocity.linear;
    
    if (motionModel) {
        debug["motion_model_reliable"] = motionModel->isReliable() ? 1.0 : 0.0;
    }
    
    return debug;
}
