#ifndef SLEW_RATE_LIMITER_H
#define SLEW_RATE_LIMITER_H

class SlewRateLimiter {
private:
    double maxRateIncrease;  // Maximum rate of increase per second
    double maxRateDecrease;  // Maximum rate of decrease per second
    double prevValue;        // Previous output value
    
public:
    /**
     * Constructs a new slew rate limiter.
     * @param rateIncrease Maximum rate of increase per second
     * @param rateDecrease Maximum rate of decrease per second
     */
    SlewRateLimiter(double rateIncrease, double rateDecrease);
    
    /**
     * Calculates the slew rate limited value.
     * @param input The desired input value
     * @param deltaTime Time step in seconds
     * @return The rate-limited output value
     */
    double calculate(double input, double deltaTime);
    
    /**
     * Resets the internal state to a specified value.
     * @param value The value to reset to (defaults to 0.0)
     */
    void reset(double value = 0.0);
};

#endif // SLEW_RATE_LIMITER_H