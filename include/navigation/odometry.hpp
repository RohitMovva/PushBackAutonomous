#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "i_localization.hpp"
#include "api.h"
#include "filters/kalman_2d.hpp"
#include "filters/heading_filter.hpp"
#include "filters/slew_rate_limiter.hpp"
#include "config.hpp"
#include <cmath>
#include <vector>
#include <numeric>
#include "utilities/logger.hpp"
#include "utilities/math/units.hpp"
#include "utilities/math/angle.hpp"

/**
 * @brief Odometry implementation of localization using wheel encoders and IMU
 *
 * Tracks robot position using advanced filtering techniques including Kalman filtering
 * and sensor fusion. Handles sensor validation, noise reduction, and uncertainty estimates.
 */
class Odometry : public ILocalization
{
private:
    // Chassis measurements
    double track_width;
    double odom_wheel_offset;

    // Hardware references
    pros::MotorGroup &leftDrive;
    pros::MotorGroup &rightDrive;
    pros::Rotation &lateralEncoder;
    pros::Imu &imu;

    // Filters
    SlewRateLimiter leftVelocityLimiter;
    SlewRateLimiter rightVelocityLimiter;
    HeadingFilter headingFilter;

    // Filter toggles
    bool useHeadingFilter;
    bool useVelocityFilters;

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
    double getAveragePosition(const std::vector<double> &positions);
    std::vector<double> getMotorPositionsInches(const std::vector<double> &motorPositions, double prevPos);
    bool isValidEncoderReading(double newPos, double oldPos, double deltaTime);
    double validateAndFilterEncoders(const std::vector<double> &positions,
                                     const std::vector<double> &prevPositions,
                                     double deltaTime);

public:
    /**
     * @brief Debug information structure for tuning
     */
    struct DebugInfo
    {
        double leftVelocityRaw;
        double rightVelocityRaw;
        double leftVelocityFiltered;
        double rightVelocityFiltered;
        double headingRaw;
        double headingFiltered;
        std::pair<double, double> uncertainty;
    };

    /**
     * @brief Construct a new Odometry Localization object
     *
     * @param left Left drive motors
     * @param right Right drive motors
     * @param lateral Lateral tracking encoder
     * @param imuSensor IMU sensor
     * @param chassis_track_width Distance between left and right wheels
     * @param lateral_wheel_offset Distance from tracking center to lateral wheel
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
             pros::Rotation &lateral, pros::Imu &imuSensor,
             bool enable_heading_filter = false,
             bool enable_velocity_filters = false);

    // Implement core interface
    void update() override;
    void reset() override;
    Pose getPose() const override;
    void setPose(const Pose &newPose) override;
    double getHeading() const override;
    double getX() const override;
    double getY() const override;
    bool isReliable() const override;
    Velocity getLeftVelocity() const override;
    Velocity getRightVelocity() const override;

    LocalizationType getType() const override { return LocalizationType::ODOMETRY; }
    std::string getTypeName() const override { return "Odometry"; }

    // Odometry-specific methods
    Velocity getLeftAcceleration() const;
    Velocity getRightAcceleration() const;
    double getAngularVelocity() const;
    double getHeadingUncertainty() const;
    std::pair<double, double> getFilteredVelocities() const;
    std::pair<double, double> getPositionUncertainty() const;
    void setFiltersEnabled(bool heading, bool velocity);
    void disableSensor(const std::string &sensor);
    DebugInfo getOdometryDebugInfo();

    // Debug interface implementation
    std::unordered_map<std::string, double> getDebugData() const override;
};

#endif // ODOMETRY_H