#ifndef UNITS_H
#define UNITS_H

namespace Units
{

    // =============================================================================
    // LENGTH CONVERSIONS
    // =============================================================================

    /**
     * @brief Convert inches to meters
     * @param inches Length in inches
     * @return Length in meters
     */
    constexpr double inchesToMeters(double inches)
    {
        return inches * 0.0254;
    }

    /**
     * @brief Convert meters to inches
     * @param meters Length in meters
     * @return Length in inches
     */
    constexpr double metersToInches(double meters)
    {
        return meters / 0.0254;
    }

    /**
     * @brief Convert feet to inches
     * @param feet Length in feet
     * @return Length in inches
     */
    constexpr double feetToInches(double feet)
    {
        return feet * 12.0;
    }

    /**
     * @brief Convert inches to feet
     * @param inches Length in inches
     * @return Length in feet
     */
    constexpr double inchesToFeet(double inches)
    {
        return inches / 12.0;
    }

    // =============================================================================
    // ENCODER/MOTOR CONVERSIONS
    // =============================================================================
    /**
     * @brief Convert motor encoder to distance based on wheel parameters
     * @param ticks Encoder ticks
     * @param ticksPerRotation Ticks per wheel rotation
     * @param wheelDiameter Wheel diameter in inches
     * @param gearRatio Gear ratio (output/input)
     * @return Distance in inches
     */
    constexpr double ticksToDistance(double ticks, double ticksPerRotation,
                                     double wheelDiameter, double gearRatio = 1.0)
    {
        const double wheelCircumference = wheelDiameter * M_PI;
        const double rotations = ticks / ticksPerRotation;
        const double wheelRotations = rotations * gearRatio;
        return wheelRotations * wheelCircumference;
    }

    /**
     * @brief Convert distance to encoder ticks based on wheel parameters
     * @param distance Distance in inches
     * @param ticksPerRotation Ticks per wheel rotation
     * @param wheelDiameter Wheel diameter in inches
     * @param gearRatio Gear ratio (output/input)
     * @return Encoder ticks
     */
    constexpr double distanceToTicks(double distance, double ticksPerRotation,
                                     double wheelDiameter, double gearRatio = 1.0)
    {
        const double wheelCircumference = wheelDiameter * M_PI;
        const double wheelRotations = distance / wheelCircumference;
        const double motorRotations = wheelRotations / gearRatio;
        return motorRotations * ticksPerRotation;
    }

    /**
     * @brief Convert RPM to linear velocity
     * @param rpm Rotations per minute
     * @param wheelDiameter Wheel diameter in inches
     * @param gearRatio Gear ratio (output/input)
     * @return Linear velocity in inches per second
     */
    constexpr double rpmToVelocity(double rpm, double wheelDiameter, double gearRatio = 1.0)
    {
        const double wheelCircumference = wheelDiameter * M_PI;
        const double wheelRPM = rpm / gearRatio;
        const double wheelRPS = wheelRPM / 60.0; // Convert minutes to seconds
        return wheelRPS * wheelCircumference;
    }

    /**
     * @brief Convert linear velocity to RPM
     * @param velocity Linear velocity in inches per second
     * @param wheelDiameter Wheel diameter in inches
     * @param gearRatio Gear ratio (output/input)
     * @return RPM
     */
    constexpr double velocityToRPM(double velocity, double wheelDiameter, double gearRatio = 1.0)
    {
        const double wheelCircumference = wheelDiameter * M_PI;
        const double wheelRPS = velocity / wheelCircumference;
        const double wheelRPM = wheelRPS * 60.0; // Convert seconds to minutes
        return wheelRPM * gearRatio;
    }

    // =============================================================================
    // TIME CONVERSIONS
    // =============================================================================

    /**
     * @brief Convert milliseconds to seconds
     * @param milliseconds Time in milliseconds
     * @return Time in seconds
     */
    constexpr double millisecondsToSeconds(double milliseconds)
    {
        return milliseconds / 1000.0;
    }

    /**
     * @brief Convert seconds to milliseconds
     * @param seconds Time in seconds
     * @return Time in milliseconds
     */
    constexpr double secondsToMilliseconds(double seconds)
    {
        return seconds * 1000.0;
    }

    // =============================================================================
    // VOLTAGE/POWER CONVERSIONS
    // =============================================================================

    /**
     * @brief Convert voltage percentage to millivolts (assuming 12V system)
     * @param percentage Voltage as percentage (-100 to 100)
     * @return Voltage in millivolts
     */
    constexpr int percentageToMillivolts(double percentage)
    {
        return static_cast<int>((percentage / 100.0) * 12000.0);
    }

    /**
     * @brief Convert millivolts to voltage percentage (assuming 12V system)
     * @param millivolts Voltage in millivolts
     * @return Voltage as percentage (-100 to 100)
     */
    constexpr double millivoltsToPercentage(int millivolts)
    {
        return (static_cast<double>(millivolts) / 12000.0) * 100.0;
    }

    // =============================================================================
    // COMMON ROBOT CONSTANTS
    // =============================================================================

    namespace Constants
    {
        // Wheel diameters (inches)
        constexpr double WHEEL_2_75_INCH = 2.75;
        constexpr double WHEEL_3_25_INCH = 3.25;
        constexpr double WHEEL_4_INCH = 4.0;

        // Gear ratios (in output/input)
        constexpr double GEAR_RATIO_36_48 = 36.0 / 48.0; // 36 teeth output, 48 teeth input
        constexpr double GEAR_RATIO_48_36 = 48.0 / 36.0; // 48 teeth output, 36 teeth input
        constexpr double GEAR_RATIO_1_1 = 1.0; // 1:1 ratio

        // Motor ticks per rotation
        constexpr double V5_BLUE_MOTOR_TICKS_PER_ROTATION = 300.0;
        constexpr double V5_GREEN_MOTOR_TICKS_PER_ROTATION = 900.0;
        constexpr double V5_RED_MOTOR_TICKS_PER_ROTATION = 1800.0;

        constexpr double V5_ROTATION_SENSOR_TICKS_PER_ROTATION = 36000.0;
    }

}

#endif // UNITS_H