#include "navigation/distance_reset_odometry.hpp"
#include "utilities/math/angle.hpp"

DistanceResetOdometry::DistanceResetOdometry(pros::MotorGroup &left, pros::MotorGroup &right,
                                           pros::Rotation &lateral, pros::Imu &imuSensor,
                                           const DistanceSensors& distance_sensors,
                                           bool enable_heading_filter,
                                           bool enable_velocity_filters)
    : Odometry(left, right, lateral, imuSensor, enable_heading_filter, enable_velocity_filters),
      sensors(distance_sensors)
{
}

void DistanceResetOdometry::reset(const DistanceSensor& sensor, double sensor_angle) {
    // imperfect system but will do for now
    if (sensor.sensor == nullptr) {
        Logger::getInstance()->logWarning("Distance sensor not available for reset");
        return;
    }

    double sensorReading = sensor.sensor->get();

    if (sensorReading >= 9999) {
        Logger::getInstance()->logWarning("Invalid distance sensor reading: %f", sensorReading);
        return;
    }

    double sensorReadingInches = sensorReading / 25.4;
    Logger::getInstance()->log("DISTANCE RESET");
    Logger::getInstance()->log("Distance sensor reading: %f in", sensorReadingInches);


    Pose currentPose = getPose();
    double robotHeading = currentPose.theta;

    // Logger::getInstance()->log("Current Pose: x=%f, y=%f, theta=%f rad", currentPose.x, currentPose.y, currentPose.theta);

    
    Logger::getInstance()->log("Robot heading: %f deg", currentPose.theta * (180.0 / M_PI) + sensor_angle);
    int headingDeg = (int)((currentPose.theta * (180.0 / M_PI) + sensor_angle));
    Logger::getInstance()->log("Robot heading: %d deg", headingDeg);

    headingDeg = (headingDeg + 360) % 360;
    Logger::getInstance()->log("Robot heading the return: %d deg", headingDeg);

    bool resettingX = false;
    double wallSign = 1.0;

    if (315 <= headingDeg || headingDeg <= 45) // Right wall 
    {
        resettingX = true;
        wallSign = 1.0;
    }
    else if (45 <= headingDeg && headingDeg < 135) // Top wall
    {
        resettingX = false;
        wallSign = 1.0;
    }
    else if (135 <= headingDeg && headingDeg < 225) // Left wall
    {
        resettingX = true;
        wallSign = -1.0;
    }
    else { // Bottom wall
        resettingX = false;
        wallSign = -1.0;
    }

    Logger::getInstance()->log("resettingX: %d, wallSign: %f", resettingX, wallSign);

        if (resettingX && int(sensor_angle)%180 == 90) {
        robotHeading += M_PI / 2;
    } else if (!resettingX && int(sensor_angle)%180 == 0){
        robotHeading += M_PI / 2;
    }
    double sensorToWall = std::cos(robotHeading) * sensorReadingInches;

    double sensorToCenter = sensor.offsetX * std::cos(robotHeading) + sensor.offsetY * std::sin(robotHeading);
    Logger::getInstance()->log("sensorToWall: %f, sensorToCenter: %f", sensorToWall, sensorToCenter);
    double wallToCenter = abs(sensorToWall + sensorToCenter);
    double actualPos = wallSign * (70.7 - wallToCenter);

    if (resettingX) {
        setPose({actualPos, currentPose.y, currentPose.theta});
        pros::lcd::print(4, "Reset X to: %.2f", actualPos);
    } else {
        setPose({currentPose.x, actualPos, currentPose.theta});
        pros::lcd::print(5, "Reset Y to: %.2f", actualPos);
    }
}

void DistanceResetOdometry::resetFront()
{
    reset(sensors.front, 0.0);
}

void DistanceResetOdometry::resetBack()
{
    reset(sensors.back, 180.0);
}

void DistanceResetOdometry::resetLeft()
{
    reset(sensors.left, 90.0);
}

void DistanceResetOdometry::resetRight()
{
    reset(sensors.right, 270.0);
}