#include "controllers/drivetrain_controller.h"
#include <algorithm>

DrivetrainController::DrivetrainController(
    double kS, double kV, double kA,
    double kP, double kI, double kD)
    : kS(kS), kV(kV), kA(kA)
    , kP(kP), kI(kI), kD(kD)
    , prevLeftVelocityError(0.0)
    , prevRightVelocityError(0.0)
    , leftIntegralError(0.0)
    , rightIntegralError(0.0)
    , prevTime(0)
{
}

double DrivetrainController::velocityToTicks(double velocityInchesPerSec) {
    return (velocityInchesPerSec * TICKS_PER_REV) / (WHEEL_DIAMETER * PI);
}

double DrivetrainController::ticksToVelocity(double ticksPerSec) {
    return (ticksPerSec * WHEEL_DIAMETER * PI) / TICKS_PER_REV;
}

DrivetrainController::MotorVoltages DrivetrainController::calculateVoltages(
    double leftVelocitySetpoint,
    double rightVelocitySetpoint,
    double leftVelocityActual,
    double rightVelocityActual,
    double leftAcceleration,
    double rightAcceleration)
{
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
    
    // Calculate feedforward
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

double DrivetrainController::calculateFeedforward(double velocityTicks, double accelerationTicks) {
    return std::copysign(kS, velocityTicks) + 
           kV * velocityTicks + 
           kA * accelerationTicks;
}

double DrivetrainController::calculateFeedback(double error, double integral, double derivative) {
    return kP * error + kI * integral + kD * derivative;
}