#ifndef CONFIG_H
#define CONFIG_H

#include "utilities/math/units.hpp"

namespace Config {
    // Robot config
    constexpr double WHEEL_DIAMETER = Units::Constants::WHEEL_2_75_INCH;             ///< Wheel diameter in inches
    constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI; ///< Wheel circumference in inches
    constexpr double TICKS_PER_ROTATION = Units::Constants::V5_BLUE_MOTOR_TICKS_PER_ROTATION;        ///< Encoder ticks per rotation for blue cartridges
    constexpr double GEAR_RATIO = 36.0 / 48.0;          ///< Gear ratio of the drivetrain
    constexpr double WHEEL_BASE_WIDTH = 14.1966209238; ///< Distance between the left and right wheels in inches
    constexpr double LATERAL_WHEEL_OFFSET = 2.419; ///< Distance from tracking center to lateral wheel in inches

    // Velocity Controller Constants
    constexpr double KS = 2.5;  ///< Static friction compensation (motor units)
    constexpr double KV = 1.85; ///< Velocity feedforward gain
    constexpr double KA = 0.3;  ///< Acceleration feedforward gain
    constexpr double KP = 9.0;  ///< Proportional gain for velocity control
    constexpr double KI = 0.00; ///< Integral gain for velocity control
    constexpr double KD = 0.0;  ///< Derivative gain for velocity control

    // Ramsete Controller Constants
    constexpr double RAMSETE_B = 2.0; ///< Ramsete B gain
    constexpr double RAMSETE_ZETA = 0.7; ///< Ramsete Zeta
    constexpr double RAMSETE_MAX_VELOCITY = 4.5 * 12; ///< Maximum velocity in inches per second
    constexpr double RAMSETE_MAX_ACCELERATION = 5.0; ///< Maximum acceleration in inches per second squared
    constexpr double RAMSETE_SCALE_FACTOR = 0.0254000508; ///< Scale factor for inputs and outputs (in relation to meters)

    // Miscellaneous constants
    constexpr double DT = 0.01;                         ///< 10ms in seconds
    constexpr bool VELOCITY_FILTER = true; ///< Enable velocity filter
    constexpr double VELOCITY_FILTER_ALPHA = 0.1; ///< Alpha value for velocity filter
    constexpr bool HEADING_FILTER = false; ///< Enable heading filter
}

#endif
