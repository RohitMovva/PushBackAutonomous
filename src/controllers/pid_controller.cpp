#include "controllers/pid_controller.hpp"
#include "utilities/logger.hpp"
#include <algorithm>

PIDController::PIDController(double kP, double kI, double kD,
                           double minOutput, double maxOutput,
                           double integralLimit)
    : kP(kP), kI(kI), kD(kD)
    , minOutput(minOutput), maxOutput(maxOutput)
    , integralTerm(0.0)
    , lastError(0.0)
    , lastOutput(0.0)
    , lastTime(0)
    , firstRun(true)
    , integralLimit(integralLimit)
{
    Logger::getInstance()->log("PID Controller initialized - kP: %f, kI: %f, kD: %f", kP, kI, kD);
}

double PIDController::calculate(double setpoint, double processVariable) {
    double error = setpoint - processVariable;
    return calculate(error);
}

double PIDController::calculate(double error) {
    // Get current time and calculate delta
    uint32_t currentTime = pros::millis();
    double dt = msToSeconds(currentTime - lastTime);
    
    // Handle first run or invalid time step
    if (firstRun || dt <= 0) {
        dt = MIN_DELTA_TIME;
        firstRun = false;
    }
    
    // Calculate proportional term
    double proportional = kP * error;
    
    // Calculate integral term with windup protection
    integralTerm = clamp(
        integralTerm + error * dt,
        -integralLimit, integralLimit);
    double integral = kI * integralTerm;
    
    // Calculate derivative term
    double derivative = 0.0;
    if (dt > MIN_DELTA_TIME) {
        derivative = kD * (error - lastError) / dt;
    }
    
    // Calculate total output
    double output = proportional + integral + derivative;
    
    // Clamp output to limits
    output = clamp(output, minOutput, maxOutput);
    
    // Log key values periodically (not every calculation)
    static uint32_t logCounter = 0;
    if (++logCounter % 50 == 0) {  // Log every 50 calculations
        Logger::getInstance()->log("PID Output: %f (P: %f, I: %f, D: %f)", 
                                 output, proportional, integral, derivative);
    }
    
    // Update state
    lastError = error;
    lastOutput = output;
    lastTime = currentTime;
    
    return output;
}

void PIDController::reset() {
    integralTerm = 0.0;
    lastError = 0.0;
    lastOutput = 0.0;
    lastTime = 0;
    firstRun = true;
    
    Logger::getInstance()->log("PID Controller reset");
}

void PIDController::setGains(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    
    Logger::getInstance()->log("PID gains updated - kP: %f, kI: %f, kD: %f", kP, kI, kD);
}

void PIDController::setOutputLimits(double minOutput, double maxOutput) {
    this->minOutput = minOutput;
    this->maxOutput = maxOutput;
    
    Logger::getInstance()->log("PID output limits updated - min: %f, max: %f", minOutput, maxOutput);
}

void PIDController::setIntegralLimit(double integralLimit) {
    this->integralLimit = integralLimit;
    
    // Clamp current integral term to new limit
    integralTerm = clamp(integralTerm, -integralLimit, integralLimit);
    
    Logger::getInstance()->log("PID integral limit updated: %f", integralLimit);
}

double PIDController::clamp(double value, double min, double max) {
    return std::clamp(value, min, max);
}

double PIDController::msToSeconds(uint32_t timeMs) {
    return timeMs / 1000.0;
}