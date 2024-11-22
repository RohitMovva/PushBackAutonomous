#ifndef DRIVETRAIN_CONTROLLER_H
#define DRIVETRAIN_CONTROLLER_H

#include "api.h"
#include <cmath>

/**
 * @brief Controller for differential drivetrain motor outputs
 * 
 * Implements feedforward and PID feedback control for precise velocity control
 * of a differential drivetrain. Handles conversion between real-world units
 * and motor units, and provides voltage outputs suitable for V5 motors.
 */
class DrivetrainController {
public:
    /**
     * @brief Struct to hold motor voltage outputs
     */
    struct MotorVoltages {
        int left;   ///< Left motor voltage (-127 to 127)
        int right;  ///< Right motor voltage (-127 to 127)
    };

    /**
     * @brief Construct a new Drivetrain Controller
     * 
     * @param kS Static friction compensation (motor units)
     * @param kV Velocity feedforward gain (motor units)/(inches/sec)
     * @param kA Acceleration feedforward gain (motor units)/(inches/sec²)
     * @param kP Proportional gain for PID
     * @param kI Integral gain for PID
     * @param kD Derivative gain for PID
     */
    DrivetrainController(double kS=5.0, double kV=0.2, double kA=0.05,
                        double kP=0.1, double kI=0.001, double kD=0.01);

    /**
     * @brief Convert linear velocity to encoder ticks
     * 
     * @param velocityInchesPerSec Velocity in inches per second
     * @return double Velocity in encoder ticks per second
     */
    double velocityToTicks(double velocityInchesPerSec);

    /**
     * @brief Convert encoder ticks to linear velocity
     * 
     * @param ticksPerSec Velocity in encoder ticks per second
     * @return double Velocity in inches per second
     */
    double ticksToVelocity(double ticksPerSec);

    /**
     * @brief Calculate motor voltages based on desired velocities and accelerations
     * 
     * @param leftVelocitySetpoint Desired left velocity (inches/sec)
     * @param rightVelocitySetpoint Desired right velocity (inches/sec)
     * @param leftVelocityActual Current left velocity (inches/sec)
     * @param rightVelocityActual Current right velocity (inches/sec)
     * @param leftAcceleration Desired left acceleration (inches/sec²)
     * @param rightAcceleration Desired right acceleration (inches/sec²)
     * @return MotorVoltages Voltage commands for left and right motors (-127 to 127)
     */
    MotorVoltages calculateVoltages(double leftVelocitySetpoint,
                                  double rightVelocitySetpoint,
                                  double leftVelocityActual,
                                  double rightVelocityActual,
                                  double leftAcceleration,
                                  double rightAcceleration);

private:
    // Feedforward constants
    const double kS;  ///< Static friction compensation (motor units)
    const double kV;  ///< Velocity feedforward gain
    const double kA;  ///< Acceleration feedforward gain
    
    // PID constants
    const double kP;  ///< Proportional gain
    const double kI;  ///< Integral gain
    const double kD;  ///< Derivative gain

    // State tracking
    double prevLeftVelocityError;
    double prevRightVelocityError;
    double leftIntegralError;
    double rightIntegralError;
    uint32_t prevTime;

    // Robot constants
    static constexpr double WHEEL_DIAMETER = 2.75;        ///< Wheel diameter (inches)
    static constexpr double GEAR_RATIO = 48.0/36.0;      ///< Gear ratio (input/output)
    static constexpr double MAX_RPM = 450.0;             ///< Maximum output RPM
    static constexpr double TICKS_PER_REV = 900.0;       ///< Encoder ticks per revolution
    static constexpr double PI = 3.14159;                ///< Mathematical constant π

    /**
     * @brief Calculate feedforward component of motor output
     * 
     * @param velocityTicks Velocity in encoder ticks/sec
     * @param accelerationTicks Acceleration in encoder ticks/sec²
     * @return double Feedforward term in motor units
     */
    double calculateFeedforward(double velocityTicks, double accelerationTicks);

    /**
     * @brief Calculate feedback component of motor output
     * 
     * @param error Velocity error in encoder ticks/sec
     * @param integral Accumulated error term
     * @param derivative Rate of change of error
     * @return double Feedback term in motor units
     */
    double calculateFeedback(double error, double integral, double derivative);
};

#endif // DRIVETRAIN_CONTROLLER_H