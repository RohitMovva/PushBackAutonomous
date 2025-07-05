#include "filters/exponential_filter.hpp"

ExponentialFilter::ExponentialFilter(double smoothingFactor)
    : alpha(smoothingFactor)
    , prevValue(0.0)
    , initialized(false)
{
}

double ExponentialFilter::update(double newValue) {
    if (!initialized) {
        initialized = true;
        prevValue = newValue;
        return newValue;
    }
    
    double filtered = alpha * newValue + (1 - alpha) * prevValue;
    prevValue = filtered;
    return filtered;
}

void ExponentialFilter::reset() {
    initialized = false;
    prevValue = 0.0;
}
