#include "navigation/odometry.h"
#include <algorithm>
#include <numeric>

// Pose implementation
Pose::Pose(double x, double y, double theta)
    : x(x), y(y), theta(theta)
{
}

// Velocity implementation
Velocity::Velocity(double linear, double angular)
    : linear(linear), angular(angular)
{
}


// TODO make these static methods in odom class
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double angleDifference(double a, double b) {
    return normalizeAngle(a - b);
}

// Odometry implementation
Odometry::Odometry(pros::MotorGroup& left, pros::MotorGroup& right,
                   pros::Rotation& lateral, pros::Imu& imuSensor,
                   double chassis_track_width, double lateral_wheel_offset,
                   bool enable_heading_filter=false,
                   bool enable_velocity_filters=false,
                   bool enable_position_filter=false)
    : WHEEL_DIAMETER(2.75)
    , WHEEL_CIRCUMFERENCE(WHEEL_DIAMETER * M_PI)
    , GEAR_RATIO(36.0 / 48.0)
    , TICKS_PER_ROTATION(300.0)
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
    , headingFilter() // 0.8
    , useHeadingFilter(enable_heading_filter)
    , useVelocityFilters(enable_velocity_filters)
    , usePositionFilter(enable_position_filter)
{
    reset();
}

// Helper function implementations
double Odometry::ticksToInches(double ticks) {
    return (ticks / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;
}

double Odometry::latTicksToInches(int ticks) {
    // Convert lateral encoder ticks to inches
    return ((double(ticks) / 1000.0) / 360.0) * WHEEL_CIRCUMFERENCE;
}


double Odometry::degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

double Odometry::getAveragePosition(const std::vector<double>& positions) {
    if (positions.empty()) return 0.0;
    return std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();
}

std::vector<double> Odometry::getMotorPositionsInches(const std::vector<double>& motorPositions, double prevPos) {
    std::vector<double> positions;
    positions.reserve(motorPositions.size());
    for (double pos : motorPositions) {
        positions.push_back(ticksToInches(pos) + prevPos);
    }
    return positions;
}

bool Odometry::isValidEncoderReading(double newPos, double oldPos, double deltaTime) {
    double velocityChange = std::abs(newPos - oldPos) / deltaTime;
    return velocityChange <= MAX_VELOCITY_CHANGE;
}

double Odometry::validateAndFilterEncoders(const std::vector<double>& positions,
                                         const std::vector<double>& prevPositions,
                                         double deltaTime) {
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
    lastUpdateTime = pros::millis() / 1000.0;
    
    std::vector<double> leftPositions = leftDrive.get_position_all();
    std::vector<double> rightPositions = rightDrive.get_position_all();
    
    prevLeftPos = std::vector<double>(leftPositions.size(), 0);
    prevRightPos = std::vector<double>(rightPositions.size(), 0);
    prevLateralPos = 0;
    prevTime = lastUpdateTime;
    
    leftVelocityFilter.reset();
    rightVelocityFilter.reset();
    headingFilter.reset();
    
    leftDrive.tare_position_all();
    rightDrive.tare_position_all();
    lateralEncoder.reset();
    imu.tare_rotation();
}

void Odometry::update() {
    double currentTime = pros::millis() / 1000.0;
    double deltaTime = currentTime - lastUpdateTime;
    
    // if (lastUpdateTime == 0.0) {
    //     lastUpdateTime = currentTime;
    //     prevTime = lastUpdateTime;
    //     return;
    // }

    if (deltaTime < 0.001 || deltaTime > 1.0) {  // Reject unrealistic time deltas
        lastUpdateTime = currentTime;
        prevTime = lastUpdateTime;
        return;
    }


    // Handle heading updates with toggle
    double filteredHeading;
    if (useHeadingFilter) {
        headingFilter.predict(deltaTime);
        double measured_heading = degreesToRadians(imu.get_heading());
        double measured_angular_velocity = degreesToRadians(imu.get_gyro_rate().x);
        headingFilter.update(measured_heading, measured_angular_velocity, true);
        filteredHeading = headingFilter.getHeading();
    } else {
        filteredHeading = degreesToRadians(imu.get_heading());
    }
    
    // double deltaTheta = angleDifference(filteredHeading, currentPose.theta);
    double deltaTheta = filteredHeading;
    
    // Get and validate encoder readings
    std::vector<double> leftPositions = getMotorPositionsInches(leftDrive.get_position_all(), getAveragePosition(prevLeftPos));
    std::vector<double> rightPositions = getMotorPositionsInches(rightDrive.get_position_all(), getAveragePosition(prevLeftPos));
    double lateralPos = latTicksToInches(lateralEncoder.get_position());
    // lateralEncoder.
    
    double currentLeftPos = getAveragePosition(leftPositions);
    double currentRightPos = getAveragePosition(rightPositions);

    
    // Calculate position changes
    double deltaLeft = currentLeftPos - getAveragePosition(prevLeftPos);
    double deltaRight = currentRightPos - getAveragePosition(prevRightPos);
    double deltaLateral = lateralPos - prevLateralPos;
    
    // Update velocities with optional filtering
    if (useVelocityFilters) {
        leftVelocity.linear = leftVelocityFilter.update(deltaLeft / deltaTime);
        rightVelocity.linear = rightVelocityFilter.update(deltaRight / deltaTime);
    } else {
        leftVelocity.linear = deltaLeft / deltaTime;
        rightVelocity.linear = deltaRight / deltaTime;
    }
    
    // Calculate accelerations
    leftAccel.linear = (leftVelocity.linear - (prevLeftPos[0] - getAveragePosition(prevLeftPos)) /
                       (prevTime - lastUpdateTime)) / deltaTime;
    rightAccel.linear = (rightVelocity.linear - (prevRightPos[0] - getAveragePosition(prevRightPos)) /
                        (prevTime - lastUpdateTime)) / deltaTime;
    
    // Calculate forward displacement
    double forwardDisplacement = (deltaLeft + deltaRight) / 2.0;
    
    // Local position update
    double deltaX, deltaY;
    
    if (std::abs(deltaTheta) > 0.001) {
        // Arc motion
        double turnRadius = forwardDisplacement / deltaTheta;
        double centralChord = 2.0 * turnRadius * sin(deltaTheta / 2.0);
        double lateralChord = deltaLateral - (odom_wheel_offset * deltaTheta);
        
        deltaX = centralChord * cos(currentPose.theta + deltaTheta / 2.0) -
                lateralChord * sin(currentPose.theta + deltaTheta / 2.0);
        deltaY = centralChord * sin(currentPose.theta + deltaTheta / 2.0) +
                lateralChord * cos(currentPose.theta + deltaTheta / 2.0);
    } else {
        // Straight motion
        deltaX = forwardDisplacement * cos(currentPose.theta) -
                deltaLateral * sin(currentPose.theta);
        deltaY = forwardDisplacement * sin(currentPose.theta) +
                deltaLateral * cos(currentPose.theta);
    }
    
    // Update position with optional Kalman filtering
    if (usePositionFilter) {
        positionFilter.predict(deltaTime);
        positionFilter.update(currentPose.x + deltaX, currentPose.y + deltaY);
        std::vector<double> state = positionFilter.getState();
        currentPose.x = state[0];
        currentPose.y = state[1];
    } else {
        currentPose.x += deltaX;
        currentPose.y += deltaY;
    }
    
    currentPose.theta = normalizeAngle(currentPose.theta + deltaTheta);
    
    // Update previous values
    prevLeftPos = leftPositions;
    prevRightPos = rightPositions;
    prevLateralPos = lateralPos;
    prevTime = lastUpdateTime;
    lastUpdateTime = currentTime;

    lateralEncoder.reset();
    imu.tare_heading();
    leftDrive.tare_position_all();
    rightDrive.tare_position_all();
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
    // Assuming P is a 2x2 matrix representing the covariance matrix
    std::vector<std::vector<double>> P = positionFilter.getCovarianceMatrix();
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
        prevLateralPos = lateralEncoder.get_position();
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

Odometry::DebugInfo Odometry::getDebugInfo() {
    return {
        leftVelocity.linear,
        rightVelocity.linear,
        leftVelocityFilter.update(leftVelocity.linear),
        rightVelocityFilter.update(rightVelocity.linear),
        degreesToRadians(imu.get_heading()),
        currentPose.theta,
        positionFilter.getState(),
        getPositionUncertainty()
    };
}
