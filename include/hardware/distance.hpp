#ifndef DISTANCE_H
#define DISTANCE_H

#include "navigation/i_localization.hpp"
#include "utilities/math/angle.hpp"
#include "utilities/math/math.hpp"
#include "api.h"
// #include "navigation/mcl.hpp"


constexpr float WALL_0_X = 72.6542371505;
constexpr float WALL_1_Y = 72.6542371505;
constexpr float WALL_2_X = -72.6542371505;
constexpr float WALL_3_Y = -72.6542371505;

class Distance {
    private:
        Pose offset; ///< Offset from the robot's center to the distance sensor
        pros::Distance distance; ///< Distance sensor
        double distanceValue; ///< Last measured distance value in inches

        bool isValid; ///< Whether the last reading was valid

        double tuningFactor; ///< Tuning factor for distance sensor

    public:
        Distance(const Pose& offset = Pose(), double tuningFactor = 1.0)
            : offset(offset), distance(0.0), isValid(false), tuningFactor(tuningFactor) {}

        /**
         * @brief Update the distance reading
         * 
         */
        void update();

        /**
         * @brief Get the current distance reading
         * 
         * @return Current distance in inches
         */
        double getDistance() const;

        /**
         * @brief Check if the last reading was valid
         * 
         * @return True if valid, false otherwise
         */
        bool isValidReading() const;

        double predict(Particle& particle) const {
            // Predict the distance based on the last valid reading
            if (!isValid) {
                return -1.0; // Invalid reading
            }
            double angle = Angles::normalizeAngle(particle.pose.theta + offset.theta);

            Pose sensorPose(
                particle.pose.x + offset.x * std::cos(angle) - offset.y * std::sin(angle),
                particle.pose.y + offset.x * std::sin(angle) + offset.y * std::cos(angle),
                angle
            );

            double predicted = 50.0;

            if (const auto theta = abs(Angles::angleDifference(0, sensorPose.theta)); theta < M_PI_2) {
                // 45 degrees on either side of 0 degrees
                predicted = (WALL_0_X - sensorPose.x) / std::cos(theta);
            } else if (const auto theta = abs(Angles::angleDifference(M_PI_2, sensorPose.theta)); theta < M_PI_2) {
                // 45 degrees on either side of 90 degrees
                predicted = (WALL_1_Y - sensorPose.x) / std::cos(theta);
            } else if (const auto theta = abs(Angles::angleDifference(M_PI, sensorPose.theta)); theta < M_PI_2) {
                // 45 degrees on either side of 180 degrees
                predicted = (WALL_2_X - sensorPose.y) / std::sin(theta);
            } else if (const auto theta = abs(Angles::angleDifference(-1*M_PI_2, sensorPose.theta)); theta < M_PI_2) {
                // 45 degrees on either side of 270 degrees
                predicted = (WALL_3_Y - sensorPose.y) / std::sin(theta);
            }
            return Math::normalPDF((getDistance() - predicted) / tuningFactor);
        }
};

#endif // DISTANCE_H
