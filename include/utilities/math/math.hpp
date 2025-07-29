#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>

namespace Math {
/**
 * @brief Cheap standard normal PDF using rational polynomial approximation
 * @param x Value to evaluate
 * @return Approximate PDF value
 */

inline double normalPDF(double x) {
    // Coefficients for the rational polynomial approximation
    const double a = 0.3989422804014337; // 1 / sqrt(2 * pi)
    const double e = 0.59422804014337;
    return a / (1.0 + e * x * x * x * x); // kinda rough approximation https://www.desmos.com/calculator/0yi9sjbdfu
}
}

#endif // MATH_UTILS_H