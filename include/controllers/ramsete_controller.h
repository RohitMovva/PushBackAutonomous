#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

/**
 * @brief RAMSETE Controller for trajectory tracking
 * 
 * Implements the RAMSETE (Rapid and Asymptotically Maximum-likelihood Estimation)
 * nonlinear time-varying feedback controller for trajectory tracking.
 * Reference: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
 */
class RamseteController {
private:
    void apply_velocity_limits(double& v, double& w) {
        v = std::max(-max_v_, std::min(v, max_v_));
        w = std::max(-max_w_, std::min(w, max_w_));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    double prev_x;
    double prev_y;
    double prev_time;
public:
    /**
     * @brief Construct a new RAMSETE Controller with default parameters
     * Default values: b = 2.0, zeta = 0.7
     */
    RamseteController();

    /**
     * @brief Construct a new RAMSETE Controller with custom tuning parameters
     * 
     * @param b Tuning parameter for convergence (default: 2.0)
     * @param zeta Damping factor (default: 0.7)
     */
    RamseteController(double b, double zeta);

    /**
     * @brief Construct a new RAMSETE Controller with all custom parameters
     * 
     * @param b Tuning parameter for convergence (default: 2.0)
     * @param zeta Damping factor (default: 0.7)
     * @param max_v Maximum linear velocity (default: 2.0)
     * @param max_w Maximum angular velocity (default: 2.0)
     * @param scale_factor Scale factor for inputs and outputs (in relation to meters) (default: 1.0)
     */
    RamseteController(double b, double zeta, double max_v, double max_w, double scale_factor);

    /**
     * @brief Calculate the global error between current and goal states
     * 
     * @param x Current x position
     * @param y Current y position
     * @param theta Current heading
     * @param goal_x Goal x position
     * @param goal_y Goal y position
     * @param goal_theta Goal heading
     * @return std::vector<double> Global error [x_error, y_error, theta_error]
     */
    std::vector<double> calculate_global_error(double x, double y, double theta,
                                             double goal_x, double goal_y, double goal_theta);

    /**
     * @brief Transform global error to local robot frame
     * 
     * @param global_error Global error vector
     * @param goal_theta Goal heading
     * @return std::vector<double> Local error in robot frame
     */
    std::vector<double> calculate_local_error(std::vector<double> global_error, double goal_theta);

    /**
     * @brief Calculate output velocities based on current state and error
     * 
     * @param v Linear velocity
     * @param w Angular velocity
     * @param theta Current heading
     * @param goal_theta Goal heading
     * @param k Control gain
     * @return std::vector<double> Output velocities [linear, angular]
     */
    std::vector<double> get_output_velocities(double v, double w, double theta,
                                            double goal_theta, double k);

    /**
     * @brief Convert desired velocities to wheel velocities
     * 
     * @param linear_velocity Desired linear velocity
     * @param angular_velocity Desired angular velocity
     * @param wheel_diameter Wheel diameter in inches
     * @param gear_ratio Gear ratio (output/input)
     * @return std::vector<double> Wheel velocities [left_rpm, right_rpm]
     */
    std::vector<double> calculate_wheel_velocities(double linear_velocity,
                                                 double angular_velocity,
                                                 double wheel_diameter,
                                                 double gear_ratio);

    /**
     * @brief Calculate control output for trajectory tracking
     * 
     * @param x Current x position
     * @param y Current y position
     * @param theta Current heading
     * @param goal_x Goal x position
     * @param goal_y Goal y position
     * @param goal_theta Goal heading
     * @param v Reference linear velocity
     * @param w Reference angular velocity
     * @return std::vector<double> Control outputs [linear_velocity, angular_velocity]
     */
    std::vector<double> calculate(double x, double y, double theta,
                                double goal_x, double goal_y, double goal_theta,
                                double v, double w);
    /**
     * @brief Helper function to calculate the sinc function
     * @param x Input value
     */
    double sinc(double x) {
        if (std::abs(x) < 1e-6) {
            return 1.0 - x * x / 6.0;
        } else {
            return std::sin(x) / x;
        }
    }

    /** 
     * @brief Helper function to normalize angle
     * @param angle Input angle in radians
     * @return double Normalized angle in range [-pi, pi]
     */
    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

private:
    double b_;      ///< Convergence tuning parameter
    double zeta_;   ///< Damping factor
    double max_v_;  ///< Maximum linear velocity
    double max_w_;  ///< Maximum angular velocity
    double scale_factor_;  ///< Scale factor for inputs and outputs
};

#endif // RAMSETE_CONTROLLER_H