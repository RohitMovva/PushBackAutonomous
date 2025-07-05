#include "filters/slew_rate_limiter.hpp"

SlewRateLimiter::SlewRateLimiter(double rateIncrease, double rateDecrease)
    : maxRateIncrease(rateIncrease)
    , maxRateDecrease(rateDecrease)
    , prevValue(0.0) 
{
}

double SlewRateLimiter::calculate(double input, double deltaTime) {
    double delta = input - prevValue;
    
    // Calculate maximum allowed change based on time step
    double maxDeltaIncrease = maxRateIncrease * deltaTime;
    double maxDeltaDecrease = maxRateDecrease * deltaTime;
    
    // Limit the rate of change
    if (delta > maxDeltaIncrease) {
        delta = maxDeltaIncrease;
    } else if (delta < -maxDeltaDecrease) {
        delta = -maxDeltaDecrease;
    }
    
    // Update and return new value
    prevValue += delta;
    return prevValue;
}

void SlewRateLimiter::reset(double value) {
    prevValue = value;
}