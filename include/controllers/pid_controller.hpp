#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "api.h"
#include <cmath>

/**
 * @brief General-purpose PID controller implementation
 * 
 * Provides proportional, integral, and derivative control for any process
 * variable. Includes integral windup protection, output clamping, and
 * optional derivative filtering for robust performance.
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID Controller
     * 
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param minOutput Minimum output value (default: -127)
     * @param maxOutput Maximum output value (default: 127)
     * @param integralLimit Maximum integral accumulation (default: 50.0)
     */
    PIDController(double kP, double kI, double kD, 
                  double minOutput = -127.0, double maxOutput = 127.0,
                  double integralLimit = 50.0);

    /**
     * @brief Calculate PID output based on current error
     * 
     * @param setpoint Desired target value
     * @param processVariable Current measured value
     * @return double Control output clamped to output limits
     */
    double calculate(double setpoint, double processVariable);

    /**
     * @brief Calculate PID output with explicit error input
     * 
     * @param error Direct error value (setpoint - processVariable)
     * @return double Control output clamped to output limits
     */
    double calculate(double error);

    /**
     * @brief Reset controller state
     * 
     * Clears integral accumulation and derivative history.
     * Call when changing setpoints or after extended periods of inactivity.
     */
    void reset();

    /**
     * @brief Set new PID gains
     * 
     * @param kP New proportional gain
     * @param kI New integral gain
     * @param kD New derivative gain
     */
    void setGains(double kP, double kI, double kD);

    /**
     * @brief Set output limits
     * 
     * @param minOutput New minimum output value
     * @param maxOutput New maximum output value
     */
    void setOutputLimits(double minOutput, double maxOutput);

    /**
     * @brief Set integral windup limit
     * 
     * @param integralLimit Maximum absolute value for integral accumulation
     */
    void setIntegralLimit(double integralLimit);

    /**
     * @brief Get current integral term value
     * 
     * @return double Current integral accumulation
     */
    double getIntegral() const { return integralTerm; }

    /**
     * @brief Get last calculated error
     * 
     * @return double Most recent error value
     */
    double getError() const { return lastError; }

    /**
     * @brief Get last calculated output
     * 
     * @return double Most recent control output
     */
    double getOutput() const { return lastOutput; }

private:
    // PID gains
    double kP;  ///< Proportional gain
    double kI;  ///< Integral gain
    double kD;  ///< Derivative gain

    // Output limits
    double minOutput;  ///< Minimum output value
    double maxOutput;  ///< Maximum output value

    // State variables
    double integralTerm;     ///< Accumulated integral error
    double lastError;        ///< Previous error for derivative calculation
    double lastOutput;       ///< Previous output value
    uint32_t lastTime;       ///< Previous calculation timestamp
    bool firstRun;           ///< Flag for first calculation

    // Integral windup protection
    double integralLimit;    ///< Maximum integral accumulation

    // Timing constants
    static constexpr double MIN_DELTA_TIME = 0.001;  ///< Minimum time step (seconds)

    /**
     * @brief Clamp value to specified range
     * 
     * @param value Input value to clamp
     * @param min Minimum allowed value
     * @param max Maximum allowed value
     * @return double Clamped value
     */
    double clamp(double value, double min, double max);

    /**
     * @brief Convert milliseconds to seconds
     * 
     * @param timeMs Time in milliseconds
     * @return double Time in seconds
     */
    double msToSeconds(uint32_t timeMs);
};

#endif // PID_CONTROLLER_H