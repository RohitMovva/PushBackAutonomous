
#include <vector>
#include <cmath>
#include <string>
#include <utility>
#include <iostream>
#include "mock_pros.h"  // Use our mock instead of api.h

// Constants
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Pose {
    Pose(double x = 0.0, double y = 0.0, double theta = 0.0)
        : x(x), y(y), theta(theta) {}
    double x;
    double y;
    double theta;
};

struct Velocity {
    Velocity(double linear = 0.0, double angular = 0.0)
        : linear(linear), angular(angular) {}
    double linear;
    double angular;
};

// Forward declarations for filter classes if needed
class HeadingFilter {
public:
    void predict(double dt) {}
    void update(double heading, double angular_velocity, bool reliable) {}
    double getHeading() const { return 0.0; }
    double getAngularVelocity() const { return 0.0; }
    double getHeadingUncertainty() const { return 0.0; }
    void reset() {}
};

class VelocityFilter {
public:
    VelocityFilter(double alpha = 0.0) : alpha_(alpha) {}
    double update(double value) const { return value; }
    void reset() {}
private:
    double alpha_;
};

class PositionFilter {
public:
    void predict(double dt) {}
    void update(double x, double y) {}
    std::vector<double> getState() const { return {0.0, 0.0, 0.0, 0.0}; }
};

class Odometry {
private:
void debugPrint(const std::string& msg) const {
    std::cout << msg << std::endl;
}
public:
    struct DebugInfo {
        double raw_left_velocity;
        double raw_right_velocity;
        double filtered_left_velocity;
        double filtered_right_velocity;
        double imu_heading;
        double filtered_heading;
        std::vector<double> kalman_state;
        std::pair<double, double> position_uncertainty;
    };

    Odometry(pros::MotorGroup& left, pros::MotorGroup& right,
             pros::Encoder& lateral, pros::Imu& imuSensor,
             double chassis_track_width, double lateral_wheel_offset,
             bool enable_heading_filter = false,
             bool enable_velocity_filters = false,
             bool enable_position_filter = false);

    void reset();
    void update();
    
    // Getters
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
    bool isReliable() const;
    double getAngularVelocity() const;
    double getHeadingUncertainty() const;
    void disableSensor(const std::string& sensor);
    void setFiltersEnabled(bool heading, bool velocity, bool position);
    std::pair<double, double> getWheelOffsets() const;
    void setWheelOffsets(double new_track_width, double new_lateral_offset);
    DebugInfo getDebugInfo() const;

private:
    // Constants
    const double WHEEL_DIAMETER;
    const double WHEEL_CIRCUMFERENCE;
    const double GEAR_RATIO;
    const double TICKS_PER_ROTATION;
    const double MAX_VELOCITY_CHANGE;
    const double MAX_ENCODER_DEVIATION;

    // Configuration
    double track_width;
    double odom_wheel_offset;

    // Hardware references
    pros::MotorGroup& leftDrive;
    pros::MotorGroup& rightDrive;
    pros::Encoder& lateralEncoder;
    pros::Imu& imu;

    // State variables
    Pose currentPose;
    Velocity leftVelocity;
    Velocity rightVelocity;
    Velocity leftAccel;
    Velocity rightAccel;
    std::vector<double> prevLeftPos;
    std::vector<double> prevRightPos;
    double prevLateralPos;
    double lastUpdateTime;
    double prevTime;

    // Filters
    VelocityFilter leftVelocityFilter;
    VelocityFilter rightVelocityFilter;
    HeadingFilter headingFilter;
    PositionFilter positionFilter;
    bool useHeadingFilter;
    bool useVelocityFilters;
    bool usePositionFilter;
    double P[4][4] = {{0}}; // Covariance matrix for position uncertainty

    // Helper methods
    double ticksToInches(double ticks) const;
    double degreesToRadians(double degrees) const;
    double getAveragePosition(const std::vector<double>& positions) const;
    std::vector<double> getMotorPositionsInches(const std::vector<double>& motorPositions) const;
    bool isValidEncoderReading(double newPos, double oldPos, double deltaTime) const;
    double validateAndFilterEncoders(const std::vector<double>& positions,
                                   const std::vector<double>& prevPositions,
                                   double deltaTime) const;
    double angleDifference(double angle1, double angle2) const {
        double diff = fmod(angle1 - angle2 + M_PI, 2.0 * M_PI) - M_PI;
        return diff < -M_PI ? diff + 2.0 * M_PI : diff;
    }
};