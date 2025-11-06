#ifndef DISTANCE_RESET_ODOMETRY_H
#define DISTANCE_RESET_ODOMETRY_H

#include "odometry.hpp"
#include "api.h"

/**
 * @brief Extended odometry with distance sensor resets
 * 
 * Adds functionality to reset robot position using distance sensors based on field walls, has to be triggered manually
 */
class DistanceResetOdometry : public Odometry
{
public:
    /**
     * @brief Structure to hold distance sensor and its offset
     */
    struct DistanceSensor
    {
        pros::Distance* sensor;
        double offsetX;
        double offsetY;

        DistanceSensor(pros::Distance* s = nullptr, double oX = 0.0, double oY = 0.0) 
            : sensor(s), offsetX(oX), offsetY(oY) {}
    };

    /**
     * @brief Structure to organize all distance sensors
     */
    struct DistanceSensors
    {
        DistanceSensor front;  // Y-offset from center
        DistanceSensor back;   // Y-offset from center  
        DistanceSensor left;   // X-offset from center
        DistanceSensor right;  // X-offset from center
        
        DistanceSensors() = default;
        DistanceSensors(DistanceSensor f, DistanceSensor b, DistanceSensor l, DistanceSensor r)
            : front(f), back(b), left(l), right(r) {}
    };

private:
    // Field dimensions may need to tune further but idk
    static constexpr double FIELD_WIDTH = 144.0;  // inches
    static constexpr double FIELD_HEIGHT = 144.0; // inches

    // Distance sensors
    DistanceSensors sensors;
    
    /**
     * @brief Reset position for any sensor
     * 
     * @param sensor The sensor to reset
     * 
     */
    void reset(const DistanceSensor& sensor, double sensor_angle);

public:
    /**
     * @brief Construct a new Distance Reset Odometry object
     * 
     * @param left Left drive motors
     * @param right Right drive motors
     * @param lateral Lateral tracking encoder
     * @param imuSensor IMU sensor
     * @param distance_sensors Structure containing all distance sensors and their offsets
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    DistanceResetOdometry(pros::MotorGroup &left, pros::MotorGroup &right,
                         pros::Rotation &lateral, pros::Imu &imuSensor,
                         const DistanceSensors& distance_sensors,
                         bool enable_heading_filter = false,
                         bool enable_velocity_filters = false);

    /**
     * @brief Reset position using front distance sensor
     * Robot's local Y position is corrected based on distance to front wall
     */
    void resetFront();

    /**
     * @brief Reset position using back distance sensor
     * Robot's local Y position is corrected based on distance to back wall
     */
    void resetBack();

    /**
     * @brief Reset position using left distance sensor
     * Robot's local X position is corrected based on distance to left wall
     */
    void resetLeft();

    /**
     * @brief Reset position using right distance sensor
     * Robot's local X position is corrected based on distance to right wall
     */
    void resetRight();

    LocalizationType getType() const override { return LocalizationType::DISTANCE_RESET_ODOMETRY; }
    std::string getTypeName() const override { return "DistanceResetOdometry"; }
};

#endif // DISTANCE_RESET_ODOMETRY_H