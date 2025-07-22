#include "navigation/odometry.hpp"

Odometry::Odometry(pros::MotorGroup &left, pros::MotorGroup &right,
                   pros::Rotation &lateral, pros::Imu &imuSensor,
                   bool enable_heading_filter,
                   bool enable_velocity_filters)
    : leftDrive(left), rightDrive(right), lateralEncoder(lateral), imu(imuSensor), leftVelocityLimiter(200, 200), rightVelocityLimiter(200, 200), headingFilter(), useHeadingFilter(enable_heading_filter), useVelocityFilters(enable_velocity_filters)
{
    reset();
}

void Odometry::reset()
{
    currentPose = Pose();
    lastUpdateTime = pros::millis() / 1000.0;

    std::vector<double> leftPositions = leftDrive.get_position_all();
    std::vector<double> rightPositions = rightDrive.get_position_all();

    prevLeftPos = std::vector<double>(leftPositions.size(), 0);
    prevRightPos = std::vector<double>(rightPositions.size(), 0);
    prevLateralPos = 0;
    prevTime = lastUpdateTime;

    leftVelocityLimiter.reset(0);
    rightVelocityLimiter.reset(0);
    headingFilter.reset();

    leftDrive.tare_position_all();
    rightDrive.tare_position_all();
    lateralEncoder.reset_position();
    imu.tare_rotation();
}

void Odometry::update()
{
    double currentTime = pros::millis() / 1000.0;
    double deltaTime = currentTime - lastUpdateTime;
    if (deltaTime <= 0 || deltaTime > 0.1) // Avoid too small or too large time steps
    {
        Logger::getInstance()->logWarning("Invalid deltaTime: %f, reverting to 10ms", deltaTime);
        deltaTime = 0.01;
    }

    // Get filtered heading
    double filteredHeading;
    if (useHeadingFilter)
    {
        headingFilter.predict(deltaTime);
        double measured_heading = Angles::degreesToRadians(imu.get_heading() * -1);
        double measured_angular_velocity = Angles::degreesToRadians(imu.get_gyro_rate().x);
        headingFilter.update(measured_heading, measured_angular_velocity, true);
        filteredHeading = headingFilter.getHeading();
    }
    else
    {
        filteredHeading = Angles::degreesToRadians(imu.get_heading() * -1);
    }

    Logger::getInstance()->log("IMU Heading: %f", imu.get_heading() * -1);
    Logger::getInstance()->log("Heading: %f", filteredHeading);

    double deltaTheta = filteredHeading;

    // Get and validate encoder readings
    std::vector<double> leftPositions = getMotorPositionsInches(leftDrive.get_position_all(),
                                                                getAveragePosition(prevLeftPos));
    std::vector<double> rightPositions = getMotorPositionsInches(rightDrive.get_position_all(),
                                                                 getAveragePosition(prevRightPos));
    double lateralPos = latTicksToInches(lateralEncoder.get_position()) + prevLateralPos;

    // For now, disable lateral encoder
    lateralPos = 0;

    Logger::getInstance()->log("Left Positions: %f %f %f", leftDrive.get_position_all()[0],
                               leftDrive.get_position_all()[1], leftDrive.get_position_all()[2]);
    Logger::getInstance()->log("Right Positions: %f %f %f", rightDrive.get_position_all()[0],
                               rightDrive.get_position_all()[1], rightDrive.get_position_all()[2]);

    double currentLeftPos = getAveragePosition(leftPositions);
    double currentRightPos = getAveragePosition(rightPositions);
    Logger::getInstance()->log("Left: %f, Right: %f", currentLeftPos, currentRightPos);

    // Calculate position changes
    double deltaLeft = currentLeftPos - getAveragePosition(prevLeftPos);
    double deltaRight = currentRightPos - getAveragePosition(prevRightPos);
    Logger::getInstance()->log("DeltaLeft: %f, DeltaRight: %f", deltaLeft, deltaRight);
    double deltaLateral = lateralPos - prevLateralPos;

    // Update velocities with optional filtering
    if (useVelocityFilters)
    {
        leftVelocity.linear = leftVelocityLimiter.calculate(deltaLeft / deltaTime, deltaTime);
        rightVelocity.linear = rightVelocityLimiter.calculate(deltaRight / deltaTime, deltaTime);
    }
    else
    {
        leftVelocity.linear = deltaLeft / deltaTime;
        rightVelocity.linear = deltaRight / deltaTime;
    }

    // Calculate accelerations
    leftAccel.linear = (leftVelocity.linear - (prevLeftPos[0] - getAveragePosition(prevLeftPos)) /
                                                  (prevTime - lastUpdateTime)) /
                       deltaTime;
    rightAccel.linear = (rightVelocity.linear - (prevRightPos[0] - getAveragePosition(prevRightPos)) /
                                                    (prevTime - lastUpdateTime)) /
                        deltaTime;

    // Calculate forward displacement
    double forwardDisplacement = (deltaLeft + deltaRight) / 2.0;
    Logger::getInstance()->log("Forward displacement: %f", forwardDisplacement);
    Logger::getInstance()->log("Delta theta: %f", deltaTheta);

    // Local position update
    double deltaX, deltaY;

    if (std::abs(deltaTheta) > 0.0001)
    {
        // Arc odometry
        double turnRadius = forwardDisplacement / deltaTheta;
        double centralChord = 2.0 * turnRadius * sin(deltaTheta / 2.0);
        double lateralChord = deltaLateral - (Config::LATERAL_WHEEL_OFFSET * deltaTheta);
        lateralChord = 0; // Disable for now

        deltaX = centralChord * cos(currentPose.theta + deltaTheta / 2.0) -
                 lateralChord * sin(currentPose.theta + deltaTheta / 2.0);
        deltaY = centralChord * sin(currentPose.theta + deltaTheta / 2.0) +
                 lateralChord * cos(currentPose.theta + deltaTheta / 2.0);
    }
    else
    {
        // Straight line approximation when dtheta is small
        deltaX = forwardDisplacement * cos(currentPose.theta) -
                 deltaLateral * sin(currentPose.theta);
        deltaY = forwardDisplacement * sin(currentPose.theta) +
                 deltaLateral * cos(currentPose.theta);
    }

    Logger::getInstance()->log("DeltaX: %f, DeltaY: %f", deltaX, deltaY);

    // Update position with optional Kalman filtering
    currentPose.x += deltaX;
    currentPose.y += deltaY;

    currentPose.theta = Angles::normalizeAngle(currentPose.theta + deltaTheta);

    // Update previous values
    prevLeftPos = leftPositions;
    prevRightPos = rightPositions;
    prevLateralPos = lateralPos;
    prevTime = lastUpdateTime;
    lastUpdateTime = currentTime;

    // Reset encoders for next iteration
    lateralEncoder.reset_position();
    imu.tare_heading();
    leftDrive.tare_position_all();
    rightDrive.tare_position_all();
}

// Core interface implementation
Pose Odometry::getPose() const { return currentPose; }
void Odometry::setPose(const Pose &newPose)
{
    currentPose = newPose;
}
double Odometry::getHeading() const { return currentPose.theta; }
double Odometry::getX() const { return currentPose.x; }
double Odometry::getY() const { return currentPose.y; }
Velocity Odometry::getLeftVelocity() const { return leftVelocity; }
Velocity Odometry::getRightVelocity() const { return rightVelocity; }

bool Odometry::isReliable() const
{
    return true; // TODO update based on if sensors are outputting valid 
}

// Helper function implementations
double Odometry::ticksToInches(double ticks)
{
    return Units::ticksToDistance(ticks, Config::TICKS_PER_ROTATION, Config::WHEEL_CIRCUMFERENCE, Config::GEAR_RATIO);
}

double Odometry::latTicksToInches(int ticks)
{
    return Units::ticksToDistance(ticks, Units::Constants::V5_ROTATION_SENSOR_TICKS_PER_ROTATION, Config::WHEEL_CIRCUMFERENCE, Units::Constants::GEAR_RATIO_1_1);
}

double Odometry::getAveragePosition(const std::vector<double> &positions)
{
    if (positions.empty())
        return 0.0;
    return std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();
}

std::vector<double> Odometry::getMotorPositionsInches(
    const std::vector<double> &motorPositions, double prevPos)
{
    std::vector<double> positions;
    positions.reserve(motorPositions.size());
    for (double pos : motorPositions)
    {
        positions.push_back(ticksToInches(pos) + prevPos);
    }
    return positions;
}

// Odometry-specific methods
Velocity Odometry::getLeftAcceleration() const { return leftAccel; }
Velocity Odometry::getRightAcceleration() const { return rightAccel; }
double Odometry::getAngularVelocity() const
{
    return useHeadingFilter ? headingFilter.getAngularVelocity() : 0.0;
}
double Odometry::getHeadingUncertainty() const
{
    return useHeadingFilter ? headingFilter.getHeadingUncertainty() : 0.0;
}

std::pair<double, double> Odometry::getFilteredVelocities() const
{
    return {leftVelocity.linear, rightVelocity.linear};
}

void Odometry::setFiltersEnabled(bool heading, bool velocity)
{
    useHeadingFilter = heading;
    useVelocityFilters = velocity;
}

void Odometry::disableSensor(const std::string &sensor)
{
    if (sensor == "left")
    {
        leftVelocityLimiter.reset();
    }
    else if (sensor == "right")
    {
        rightVelocityLimiter.reset();
    }
    else if (sensor == "lateral")
    {
        prevLateralPos = lateralEncoder.get_position() * -1;
    }
}

Odometry::DebugInfo Odometry::getOdometryDebugInfo()
{
    return {
        leftVelocity.linear,
        rightVelocity.linear,
        leftVelocityLimiter.calculate(leftVelocity.linear, 0.01),
        rightVelocityLimiter.calculate(rightVelocity.linear, 0.01),
        Angles::degreesToRadians(imu.get_heading() * -1),
        currentPose.theta,
        getPositionUncertainty()};
}

std::unordered_map<std::string, double> Odometry::getDebugData() const
{
    std::unordered_map<std::string, double> debug;
    debug["left_velocity"] = leftVelocity.linear;
    debug["right_velocity"] = rightVelocity.linear;
    debug["left_acceleration"] = leftAccel.linear;
    debug["right_acceleration"] = rightAccel.linear;
    debug["heading_raw"] = Angles::degreesToRadians(imu.get_heading() * -1);
    debug["heading_filtered"] = currentPose.theta;
    debug["reliable"] = isReliable() ? 1.0 : 0.0;

    if (useHeadingFilter)
    {
        debug["angular_velocity"] = getAngularVelocity();
        debug["heading_uncertainty"] = getHeadingUncertainty();
    }

    return debug;
}