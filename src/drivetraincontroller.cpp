#include "main.h"
#include <cmath>

class DrivetrainController {
    private:
        // Feedforward constants
        // For V5, voltage range is -127 to 127
        // kS is in motor units
        // kV is in (motor units)/(inches/sec)
        // kA is in (motor units)/(inches/sec²)
        const double kS;  // Static friction compensation
        const double kV;  // Velocity-to-motor units gain
        const double kA;  // Acceleration-to-motor units gain
        
        // PID constants for velocity control
        const double kP;
        const double kI;
        const double kD;
        
        // Track previous values
        double prevLeftVelocityError = 0.0;
        double prevRightVelocityError = 0.0;
        double leftIntegralError = 0.0;
        double rightIntegralError = 0.0;
        uint32_t prevTime = 0;

        // Constants for your specific robot
        const double WHEEL_DIAMETER = 2.75;  // inches
        const double GEAR_RATIO = 1.33;      // 600 RPM to 450 RPM
        const double MAX_RPM = 450;          // Output RPM
        
        // Conversion factors
        const double TICKS_PER_REV = 900.0;  // V5 encoder ticks per revolution
        const double PI = 3.14159;

    public:
        DrivetrainController(
                double kS, double kV, double kA,
                double kP, double kI, double kD) 
            : kS(kS), kV(kV), kA(kA), kP(kP), kI(kI), kD(kD) {}
        
        // Converts from inches/sec to encoder ticks/sec
        double velocityToTicks(double velocityInchesPerSec) {
            return (velocityInchesPerSec * TICKS_PER_REV) / (WHEEL_DIAMETER * PI);
        }
        
        // Converts from encoder ticks/sec to inches/sec
        double ticksToVelocity(double ticksPerSec) {
            return (ticksPerSec * WHEEL_DIAMETER * PI) / TICKS_PER_REV;
        }
        
        struct MotorVoltages {
            int left;
            int right;
        };
        
        MotorVoltages calculateVoltages(
                double leftVelocitySetpoint,     // Desired left velocity (inches/sec)
                double rightVelocitySetpoint,    // Desired right velocity (inches/sec)
                double leftVelocityActual,       // Current left velocity (inches/sec)
                double rightVelocityActual,      // Current right velocity (inches/sec)
                double leftAcceleration,         // Desired left acceleration (inches/sec²)
                double rightAcceleration) {      // Desired right acceleration (inches/sec²)
            
            // Get current time and calculate delta
            uint32_t currentTime = pros::millis();
            double dt = (currentTime - prevTime) / 1000.0;
            if (dt <= 0) dt = 0.02;  // Default to 20ms
            
            // Convert velocities to encoder ticks/sec for consistency
            double leftSetpointTicks = velocityToTicks(leftVelocitySetpoint);
            double rightSetpointTicks = velocityToTicks(rightVelocitySetpoint);
            double leftActualTicks = velocityToTicks(leftVelocityActual);
            double rightActualTicks = velocityToTicks(rightVelocityActual);
            
            // Calculate errors in ticks/sec
            double leftVelocityError = leftSetpointTicks - leftActualTicks;
            double rightVelocityError = rightSetpointTicks - rightActualTicks;
            
            // Update integral terms (with anti-windup)
            const double integralLimit = 1000.0;
            leftIntegralError = std::clamp(
                leftIntegralError + leftVelocityError * dt,
                -integralLimit, integralLimit);
            rightIntegralError = std::clamp(
                rightIntegralError + rightVelocityError * dt,
                -integralLimit, integralLimit);
            
            // Calculate derivative terms
            double leftDerivative = (leftVelocityError - prevLeftVelocityError) / dt;
            double rightDerivative = (rightVelocityError - prevRightVelocityError) / dt;
            
            // Calculate feedforward (converting accelerations to ticks/sec²)
            double leftFeedforward = calculateFeedforward(
                leftSetpointTicks, 
                velocityToTicks(leftAcceleration));
            double rightFeedforward = calculateFeedforward(
                rightSetpointTicks, 
                velocityToTicks(rightAcceleration));
            
            // Calculate feedback
            double leftFeedback = calculateFeedback(
                leftVelocityError, 
                leftIntegralError, 
                leftDerivative);
            double rightFeedback = calculateFeedback(
                rightVelocityError, 
                rightIntegralError,
                rightDerivative);
            
            // Combine feedforward and feedback
            int leftVoltage = static_cast<int>(leftFeedforward + leftFeedback);
            int rightVoltage = static_cast<int>(rightFeedforward + rightFeedback);
            
            // Update previous values
            prevLeftVelocityError = leftVelocityError;
            prevRightVelocityError = rightVelocityError;
            prevTime = currentTime;
            
            // Clamp to V5 motor range (-127 to 127)
            leftVoltage = std::clamp(leftVoltage, -127, 127);
            rightVoltage = std::clamp(rightVoltage, -127, 127);
            
            return {leftVoltage, rightVoltage};
        }
        
    private:
        double calculateFeedforward(double velocityTicks, double accelerationTicks) {
            // Note: kS preserves sign of velocity for static friction compensation
            return std::copysign(kS, velocityTicks) + 
                   kV * velocityTicks + 
                   kA * accelerationTicks;
        }
        
        double calculateFeedback(double error, double integral, double derivative) {
            return kP * error + kI * integral + kD * derivative;
        }
};