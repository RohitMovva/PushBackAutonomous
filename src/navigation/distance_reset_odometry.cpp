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
    if (sensor.sensor == nullptr) {
        Logger::getInstance()->logWarning("Distance sensor not available for reset");
        return;
    }

    double sensorReading = sensor.sensor->get();
    if (sensorReading >= 9999) {
        Logger::getInstance()->logWarning("Invalid distance sensor reading: %f", sensorReading);
        return;
    }

    Pose currentPose = getPose();
    double robotHeading = getHeading();

    
    int headingDeg = (int)(robotHeading * (180.0 / M_PI) + sensor_angle);

    headingDeg = headingDeg % 360;

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
        wallSign = -1.0;
    }
    else if (135 <= headingDeg && headingDeg < 225) // Left wall
    {
        resettingX = true;
        wallSign = -1.0;
    }
    else { // Bottom wall
        resettingX = false;
        wallSign = 1.0;
    }


    if (resettingX) {
        robotHeading += M_PI / 2;
    }

    double sensorToWall = std::cos(robotHeading) * sensorReading;

    double sensorToCenter = sensor.offsetX * std::cos(robotHeading) + sensor.offsetY * std::sin(robotHeading);

    double wallToCenter = sensorToWall + sensorToCenter;
    double actualPos = wallSign * (70.7 - wallToCenter);

    if (resettingX) {
        setPose({actualPos, currentPose.y, currentPose.theta});
    } else {
        setPose({currentPose.x, actualPos, currentPose.theta});
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