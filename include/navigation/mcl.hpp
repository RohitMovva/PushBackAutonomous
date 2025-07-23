#ifndef MCL_H
#define MCL_H

#include "navigation/i_localization.hpp"
#include "navigation/odometry.hpp"
#include "api.h"
#include <memory>
#include <vector>
#include <random>

/**
 * @brief Particle structure for Monte Carlo Localization
 */
struct Particle
{
    Pose pose;
    double weight;
    
    Particle(double x = 0, double y = 0, double theta = 0, double w = 1.0)
        : pose(x, y, theta), weight(w) {}
        
    void predict(const Pose& motion, double motionNoise);
    void updateWeight(double likelihood);
};

/**
 * @brief Monte Carlo Localization implementation
 * 
 * Uses particle filtering with sensor fusion for robust localization.
 * Maintains a set of particles representing possible robot poses and
 * updates them based on motion model and sensor observations.
 */
class MCL : public ILocalization
{
private:
    // Configuration
    int numParticles;
    double resampleThreshold;
    
    // Particle filter state
    std::vector<Particle> particles;
    Pose estimatedPose;
    double confidence;
    
    // Motion model (internal odometry)
    std::unique_ptr<Odometry> motionModel;
    
    // Velocity tracking for interface compatibility
    Velocity leftVelocity;
    Velocity rightVelocity;
    
    // Random number generation
    std::random_device randomDevice;
    std::mt19937 generator;
    std::normal_distribution<double> motionNoise;
    
    // Sensor integration (placeholders for future implementation)
    // std::unique_ptr<LidarSensor> lidar;
    // std::unique_ptr<FieldMap> fieldMap;
    // std::unique_ptr<VisionSensor> vision;
    
    // Helper methods
    void initializeParticles(Pose startingPose);
    void predictParticles();
    void updateParticlesWithSensors();
    void resampleParticles();
    bool shouldResample() const;
    Pose estimatePoseFromParticles();
    double calculateEffectiveParticleCount() const;
    void normalizeWeights();
    
public:
    /**
     * @brief Debug information structure for MCL tuning
     */
    struct DebugInfo
    {
        int activeParticles;
        double effectiveParticleCount;
        double confidence;
        double motionNoise;
        Pose bestParticle;
        Pose worstParticle;
        double weightVariance;
    };

    /**
     * @brief Construct a new MCL Localization object
     *
     * @param left Left drive motors (for internal motion model)
     * @param right Right drive motors (for internal motion model)  
     * @param lateral Lateral tracking encoder (for internal motion model)
     * @param imuSensor IMU sensor (for internal motion model)
     * @param particle_count Number of particles to use
     * @param motion_noise_std Standard deviation of motion noise
     */
    MCL(pros::MotorGroup &left, pros::MotorGroup &right,
                   pros::Rotation &lateral, pros::Imu &imuSensor,
                   int particle_count = 1000,
                   double motion_noise_std = 0.1);

    // Implement core interface
    void update() override;
    void reset() override;
    Pose getPose() const override;
    void setPose(const Pose& newPose) override;
    double getHeading() const override;
    double getX() const override;
    double getY() const override;
    bool isReliable() const override;
    double getTrackWidth() const override;
    Velocity getLeftVelocity() const override;
    Velocity getRightVelocity() const override;
    
    LocalizationType getType() const override { return LocalizationType::MONTE_CARLO; }
    std::string getTypeName() const override { return "Monte Carlo Localization"; }
    
    // MCL-specific methods
    void setParticleCount(int count);
    int getParticleCount() const { return numParticles; }
    double getConfidence() const { return confidence; }
    void setMotionNoiseStd(double std);
    void setResampleThreshold(double threshold) { resampleThreshold = threshold; }
    
    // Sensor integration methods (for future implementation)
    // void updateWithLidar(const LidarReading& reading);
    // void updateWithVision(const VisionReading& reading);
    // void setFieldMap(std::unique_ptr<FieldMap> map);
    
    // Particle access for debugging/visualization
    const std::vector<Particle>& getParticles() const { return particles; }
    DebugInfo getMCLDebugInfo() const;
    
    // Debug interface implementation
    std::unordered_map<std::string, double> getDebugData() const override;
};

#endif // MCL_H