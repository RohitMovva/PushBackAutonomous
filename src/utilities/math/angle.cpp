#include "utilities/math/angle.hpp"
#include <vector>
#include <cmath>

namespace Angles {

double normalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

double angleDifference(double a, double b) {
    return normalizeAngle(a - b);
}

double degreesToRadians(double degrees) {
    // Normalize degrees first to avoid large angle wrapping issues
    while (degrees > 180.0) degrees -= 360.0;
    while (degrees < -180.0) degrees += 360.0;
    return degrees * M_PI / 180.0;
}

double radiansToDegrees(double radians) {
    return normalizeAngle(radians) * 180.0 / M_PI;
}

double normalizeAngleDegrees(double angle) {
    while (angle > 180.0)
        angle -= 360.0;
    while (angle < -180.0)
        angle += 360.0;
    return angle;
}

double angleDifferenceDegrees(double a, double b) {
    return normalizeAngleDegrees(a - b);
}

bool anglesEqual(double a, double b, double tolerance) {
    return std::abs(angleDifference(a, b)) <= tolerance;
}

double wrapAngle0To2Pi(double angle) {
    while (angle < 0.0)
        angle += 2 * M_PI;
    while (angle >= 2 * M_PI)
        angle -= 2 * M_PI;
    return angle;
}

double wrapAngle0To360(double angle) {
    while (angle < 0.0)
        angle += 360.0;
    while (angle >= 360.0)
        angle -= 360.0;
    return angle;
}

double averageAngles(const std::vector<double>& angles) {
    if (angles.empty()) return 0.0;
    
    double x = 0.0, y = 0.0;
    for (double angle : angles) {
        x += std::cos(angle);
        y += std::sin(angle);
    }
    
    return std::atan2(y, x);
}

}
