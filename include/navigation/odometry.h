#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "api.h"
#include "filters/kalman_2d.h"
#include "filters/heading_filter.h"
#include "filters/slew_rate_limiter.h"
#include <cmath>
#include <vector>
#include <string>
#include <utility>

/**
 * @brief Represents a 2D pose (position and orientation)
 */
struct Pose {
    double x;         ///< X position in inches
    double y;         ///< Y position in inches
    double theta;     ///< Heading in radians
    
    Pose(double x = 0, double y = 0, double theta = 0);
};

/**
 * @brief Represents linear and angular velocity
 */
struct Velocity {
    double linear;    ///< Linear velocity in inches/sec
    double angular;   ///< Angular velocity in rad/sec
    
    Velocity(double linear = 0, double angular = 0);
};

/**
 * @brief Advanced odometry system using Kalman filtering and sensor fusion
 * 
 * Tracks robot position using wheel encoders, IMU, and advanced filtering techniques.
 * Handles sensor validation, noise reduction, and provides uncertainty estimates.
 */
class Odometry {
private:
    // Constants
    const double WHEEL_DIAMETER;
    const double WHEEL_CIRCUMFERENCE;
    const double GEAR_RATIO;
    const double TICKS_PER_ROTATION;
    const double MAX_VELOCITY_CHANGE;
    const double MAX_ENCODER_DEVIATION;
    
    // Chassis measurements
    double track_width;
    double odom_wheel_offset;
    
    // Hardware references
    pros::MotorGroup& leftDrive;
    pros::MotorGroup& rightDrive;
    pros::Rotation& lateralEncoder;
    pros::Imu& imu;
    
    // Filters
    Kalman2D positionFilter;
    // ExponentialFilter leftVelocityFilter;
    // ExponentialFilter rightVelocityFilter;
    SlewRateLimiter leftVelocityLimiter;
    SlewRateLimiter rightVelocityLimiter;
    HeadingFilter headingFilter;

    // Filter toggles
    bool useHeadingFilter;
    bool useVelocityFilters;
    bool usePositionFilter;
    
    // State variables
    Pose currentPose;
    double lastUpdateTime;
    std::vector<double> prevLeftPos;
    std::vector<double> prevRightPos;
    double prevLateralPos;
    double prevTime;
    
    // Velocity and acceleration tracking
    Velocity leftVelocity;
    Velocity rightVelocity;
    Velocity leftAccel;
    Velocity rightAccel;
    
    // Helper functions
    double ticksToInches(double ticks);
    double latTicksToInches(int ticks);
    double degreesToRadians(double degrees);
    double getAveragePosition(const std::vector<double>& positions);
    std::vector<double> getMotorPositionsInches(const std::vector<double>& motorPositions, double prevPos);
    bool isValidEncoderReading(double newPos, double oldPos, double deltaTime);
    double validateAndFilterEncoders(const std::vector<double>& positions,
                                   const std::vector<double>& prevPositions,
                                   double deltaTime);

public:
    /**
     * @brief Debug information structure for tuning
     */
    struct DebugInfo {
        double leftVelocityRaw;
        double rightVelocityRaw;
        double leftVelocityFiltered;
        double rightVelocityFiltered;
        double headingRaw;
        double headingFiltered;
        std::vector<double> kalmanState;
        std::pair<double, double> uncertainty;
    };

    /**
     * @brief Construct a new Odometry object
     * 
     * @param left Left drive motors
     * @param right Right drive motors
     * @param lateral Lateral tracking encoder
     * @param imuSensor IMU sensor
     * @param chassis_track_width Distance between left and right wheels
     * @param lateral_wheel_offset Distance from tracking center to lateral wheel
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     * @param enable_position_filter Enable position filter
     */
    Odometry(pros::MotorGroup& left, pros::MotorGroup& right,
             pros::Rotation& lateral, pros::Imu& imuSensor,
             double chassis_track_width, double lateral_wheel_offset,
             bool enable_heading_filter,
             bool enable_velocity_filters,
             bool enable_position_filter);

    /**
     * @brief Reset odometry to initial state
     */
    void reset();
    
    /**
     * @brief Update odometry calculations
     */
    void update();
    
    // Getters and setters
    Pose getPose() const;
    Velocity getLeftVelocity() const;
    Velocity getRightVelocity() const;
    Velocity getLeftAcceleration() const;
    Velocity getRightAcceleration() const;
    double getHeading() const;
    double getX() const;
    double getY() const;
    void setPose(const Pose& newPose);
    std::pair<double, double> getFilteredVelocities() const;
    std::pair<double, double> getPositionUncertainty() const;
    double getAngularVelocity() const;
    double getHeadingUncertainty() const;
    std::pair<double, double> getWheelOffsets() const;
    void setWheelOffsets(double new_track_width, double new_lateral_offset);
    DebugInfo getDebugInfo();
    
    /**
     * @brief Check if odometry readings are reliable
     */
    bool isReliable() const;
    
    /**
     * @brief Temporarily disable a specific sensor
     * 
     * @param sensor Sensor to disable ("left", "right", or "lateral")
     */
    void disableSensor(const std::string& sensor);

    /**
     * @brief Set which filters are enabled
     * 
     * @param heading Enable heading filter
     * @param velocity Enable velocity filters
     * @param position Enable position filter
     */
    void setFiltersEnabled(bool heading, bool velocity, bool position);
};

#endif // ODOMETRY_H