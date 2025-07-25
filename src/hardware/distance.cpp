#include "hardware/distance.hpp"

void Distance::update() {
    // Read the distance sensor
    double rawDistance = distance.get();

    isValid = (rawDistance < 9999);
    
    // Apply tuning factor
    distanceValue = rawDistance * tuningFactor;
}

double Distance::getDistance() const {
    if (!isValid) {
        return 0.0; // Return 0 if the last reading was invalid
    }
    return distanceValue; // Return distance in inches
}