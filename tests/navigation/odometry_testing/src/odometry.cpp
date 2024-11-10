#include "navigation/odometry.h"
#include <algorithm>
#include <numeric>

// Odometry implementation
Odometry::Odometry(pros::MotorGroup& left, pros::MotorGroup& right,
                   pros::Encoder& lateral, pros::Imu& imuSensor,
                   double chassis_track_width, double lateral_wheel_offset,
                   bool enable_heading_filter,
                   bool enable_velocity_filters,
                   bool enable_position_filter)
    : WHEEL_DIAMETER(2.75)
    , WHEEL_CIRCUMFERENCE(WHEEL_DIAMETER * M_PI)
    , GEAR_RATIO(36.0 / 48.0)
    , TICKS_PER_ROTATION(900.0)
    , MAX_VELOCITY_CHANGE(100.0)
    , MAX_ENCODER_DEVIATION(2.0)
    , track_width(chassis_track_width)
    , odom_wheel_offset(lateral_wheel_offset)
    , leftDrive(left)
    , rightDrive(right)
    , lateralEncoder(lateral)
    , imu(imuSensor)
    , leftVelocityFilter(0.7)
    , rightVelocityFilter(0.7)
    , headingFilter()
    , useHeadingFilter(enable_heading_filter)
    , useVelocityFilters(enable_velocity_filters)
    , usePositionFilter(enable_position_filter)
{
    reset();
}

// Helper function implementations
double Odometry::ticksToInches(double ticks) const {
    return (ticks / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;
}

double Odometry::degreesToRadians(double degrees) const {
    return degrees * M_PI / 180.0;
}

double Odometry::getAveragePosition(const std::vector<double>& positions) const {
    if (positions.empty()) return 0.0;
    return std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();
}

std::vector<double> Odometry::getMotorPositionsInches(const std::vector<double>& motorPositions) const {
    std::vector<double> positions;
    positions.reserve(motorPositions.size());
    for (double pos : motorPositions) {
        positions.push_back(ticksToInches(pos));
    }
    return positions;
}

bool Odometry::isValidEncoderReading(double newPos, double oldPos, double deltaTime) const {
    double velocityChange = std::abs(newPos - oldPos) / deltaTime;
    return velocityChange <= MAX_VELOCITY_CHANGE;
}

double Odometry::validateAndFilterEncoders(const std::vector<double>& positions,
                                         const std::vector<double>& prevPositions,
                                         double deltaTime) const {
    std::vector<double> validPositions;
    validPositions.reserve(positions.size());
    
    for (size_t i = 0; i < positions.size(); i++) {
        if (isValidEncoderReading(positions[i], prevPositions[i], deltaTime)) {
            validPositions.push_back(positions[i]);
        }
    }
    
    if (validPositions.empty()) {
        return prevPositions[0] + (leftVelocity.linear * deltaTime);
    }
    
    return getAveragePosition(validPositions);
}

// Main methods implementation
void Odometry::reset() {
    currentPose = Pose();
    lastUpdateTime = 0.0;  // We'll use regular doubles for time in testing
    
    std::vector<double> leftPositions = leftDrive.get_position_all();
    std::vector<double> rightPositions = rightDrive.get_position_all();
    
    prevLeftPos = std::vector<double>(leftPositions.size(), 0);
    prevRightPos = std::vector<double>(rightPositions.size(), 0);
    prevLateralPos = 0;
    prevTime = lastUpdateTime;
}

void Odometry::update() {
    double currentTime = pros::millis() / 1000.0;
    double deltaTime = currentTime - lastUpdateTime;
    
    if (deltaTime < 0.001) {
        return;
    }

    if (lastUpdateTime == 0.0) {
        lastUpdateTime = currentTime;
        prevTime = lastUpdateTime;
        return;
    }

    if (deltaTime < 0.001 || deltaTime > 1.0) {  // Reject unrealistic time deltas
        lastUpdateTime = currentTime;
        prevTime = lastUpdateTime;
        return;
    }

    // Get encoder readings and convert to inches
    std::vector<double> leftPositions = getMotorPositionsInches(leftDrive.get_position_all());
    std::vector<double> rightPositions = getMotorPositionsInches(rightDrive.get_position_all());
    double lateralPos = ticksToInches(lateralEncoder.get_value());

    double currentLeftPos = getAveragePosition(leftPositions);
    double currentRightPos = getAveragePosition(rightPositions);
    
    // Calculate changes in position
    double deltaLeft = currentLeftPos - getAveragePosition(prevLeftPos);
    double deltaRight = currentRightPos - getAveragePosition(prevRightPos);
    double deltaLateral = lateralPos - prevLateralPos;

    debugPrint("Time delta: " + std::to_string(deltaTime));
    debugPrint("Left delta: " + std::to_string(deltaLeft));
    debugPrint("Right delta: " + std::to_string(deltaRight));
    debugPrint("Lateral delta: " + std::to_string(deltaLateral));

    // Calculate velocities
    leftVelocity.linear = deltaTime > 0.001 ? deltaLeft / deltaTime : 0.0;
    rightVelocity.linear = deltaTime > 0.001 ? deltaRight / deltaTime : 0.0;

    debugPrint("Left velocity: " + std::to_string(leftVelocity.linear));
    debugPrint("Right velocity: " + std::to_string(rightVelocity.linear));

    // Get heading from IMU
    double newHeading = degreesToRadians(imu.get_rotation());
    double deltaTheta = angleDifference(newHeading, currentPose.theta);

    debugPrint("Heading (deg): " + std::to_string(imu.get_rotation()));
    debugPrint("Delta theta: " + std::to_string(deltaTheta));

    // Calculate forward displacement
    double forwardDisplacement = (deltaLeft + deltaRight) / 2.0;
    debugPrint("Forward displacement: " + std::to_string(forwardDisplacement));
    
    // Simplified position update
    double deltaX, deltaY;
    if (std::abs(deltaTheta) > 0.001) {
        // Current implementation:
        double turnRadius = forwardDisplacement / deltaTheta;
        double centralChord = 2.0 * turnRadius * sin(deltaTheta / 2.0);
        
        // Add this check:
        if (std::abs(forwardDisplacement) < 0.001) {
            // Pure rotation case
            deltaX = 0.0;
            deltaY = 0.0;
        } else {
            // Arc motion case (keep existing code)
            deltaX = centralChord * cos(currentPose.theta + deltaTheta / 2.0);
            deltaY = centralChord * sin(currentPose.theta + deltaTheta / 2.0);
        }
    } else {
        // Straight motion
        deltaX = forwardDisplacement * cos(currentPose.theta) - 
                deltaLateral * sin(currentPose.theta);
        deltaY = forwardDisplacement * sin(currentPose.theta) + 
                deltaLateral * cos(currentPose.theta);
    }

    debugPrint("Delta X: " + std::to_string(deltaX));
    debugPrint("Delta Y: " + std::to_string(deltaY));

    // Update pose
    currentPose.x += deltaX;
    currentPose.y += deltaY;
    currentPose.theta = newHeading;

    debugPrint("Current pose - X: " + std::to_string(currentPose.x) + 
               " Y: " + std::to_string(currentPose.y) + 
               " Theta: " + std::to_string(currentPose.theta));
    debugPrint("-------------------");

    // Update previous values
    prevLeftPos = leftPositions;
    prevRightPos = rightPositions;
    prevLateralPos = lateralPos;
    prevTime = lastUpdateTime;
    lastUpdateTime = currentTime;
}

// Getter implementations
Pose Odometry::getPose() const {
    return currentPose;
}

Velocity Odometry::getLeftVelocity() const {
    return leftVelocity;
}

Velocity Odometry::getRightVelocity() const {
    return rightVelocity;
}

Velocity Odometry::getLeftAcceleration() const {
    return leftAccel;
}

Velocity Odometry::getRightAcceleration() const {
    return rightAccel;
}

double Odometry::getHeading() const {
    return currentPose.theta;
}

double Odometry::getX() const {
    return currentPose.x;
}

double Odometry::getY() const {
    return currentPose.y;
}

void Odometry::setPose(const Pose& newPose) {
    currentPose = newPose;
    // Update Kalman filter state
    std::vector<double> state = positionFilter.getState();
    state[0] = newPose.x;
    state[1] = newPose.y;
    // Reset velocity states to 0
    state[2] = 0;
    state[3] = 0;
}

std::pair<double, double> Odometry::getFilteredVelocities() const {
    std::vector<double> state = positionFilter.getState();
    return {state[2], state[3]};
}

std::pair<double, double> Odometry::getPositionUncertainty() const {
    return {std::sqrt(P[0][0]), std::sqrt(P[1][1])};
}

bool Odometry::isReliable() const {
    std::pair<double, double> uncertainty = getPositionUncertainty();
    const double MAX_UNCERTAINTY = 5.0; // inches
    
    return uncertainty.first < MAX_UNCERTAINTY &&
           uncertainty.second < MAX_UNCERTAINTY &&
           std::abs(leftVelocity.linear) < MAX_VELOCITY_CHANGE &&
           std::abs(rightVelocity.linear) < MAX_VELOCITY_CHANGE;
}

double Odometry::getAngularVelocity() const {
    return headingFilter.getAngularVelocity();
}

double Odometry::getHeadingUncertainty() const {
    return headingFilter.getHeadingUncertainty();
}

void Odometry::disableSensor(const std::string& sensor) {
    if (sensor == "left") {
        leftVelocityFilter.reset();
    } else if (sensor == "right") {
        rightVelocityFilter.reset();
    } else if (sensor == "lateral") {
        // Reset lateral tracking
        prevLateralPos = lateralEncoder.get_value();
    }
}

void Odometry::setFiltersEnabled(bool heading, bool velocity, bool position) {
    useHeadingFilter = heading;
    useVelocityFilters = velocity;
    usePositionFilter = position;
}

std::pair<double, double> Odometry::getWheelOffsets() const {
    return {track_width, odom_wheel_offset};
}

void Odometry::setWheelOffsets(double new_track_width, double new_lateral_offset) {
    track_width = new_track_width;
    odom_wheel_offset = new_lateral_offset;
}

Odometry::DebugInfo Odometry::getDebugInfo() const {
    return {
        leftVelocity.linear,
        rightVelocity.linear,
        leftVelocityFilter.update(leftVelocity.linear),
        rightVelocityFilter.update(rightVelocity.linear),
        degreesToRadians(imu.get_rotation()),
        currentPose.theta,
        positionFilter.getState(),
        getPositionUncertainty()
    };
}
