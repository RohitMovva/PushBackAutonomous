#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

#include <cmath>
#include <vector>

namespace Angles {

/**
 * @brief Normalize an angle to the range [-π, π]
 * @param angle Angle in radians
 * @return Normalized angle in radians
 */
double normalizeAngle(double angle);

/**
 * @brief Calculate the shortest angular difference between two angles
 * @param a First angle in radians
 * @param b Second angle in radians  
 * @return Shortest angular difference (a - b) in radians, normalized to [-π, π]
 */
double angleDifference(double a, double b);

/**
 * @brief Convert degrees to radians
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
double degreesToRadians(double degrees);

/**
 * @brief Convert radians to degrees
 * @param radians Angle in radians
 * @return Angle in degrees
 */
double radiansToDegrees(double radians);

/**
 * @brief Check if two angles are approximately equal within a tolerance
 * @param a First angle in radians
 * @param b Second angle in radians
 * @param tolerance Tolerance in radians (default: 1e-6)
 * @return True if angles are within tolerance
 */
bool anglesEqual(double a, double b, double tolerance = 1e-6);

/**
 * @brief Calculate the average of multiple angles (handles wraparound correctly)
 * @param angles Vector of angles in radians
 * @return Average angle in radians
 */
double averageAngles(const std::vector<double>& angles);

}

#endif // ANGLE_UTILS_H