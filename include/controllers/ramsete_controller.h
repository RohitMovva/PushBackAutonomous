#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

#include <vector>
#include <cmath>

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
        const double max_v = 2.0;
        const double max_w = 2.0;
        v = std::max(-max_v, std::min(v, max_v));
        w = std::max(-max_w, std::min(w, max_w));
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
public:
    /**
     * @brief Construct a new RAMSETE Controller with default parameters
     * Default values: b = 2.0, zeta = 0.7
     */
    RamseteController();

    /**
     * @brief Construct a new RAMSETE Controller with custom parameters
     * 
     * @param b Tuning parameter for convergence (default: 2.0)
     * @param zeta Damping factor (default: 0.7)
     */
    RamseteController(double b, double zeta);

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

private:
    double b_;      ///< Convergence tuning parameter
    double zeta_;   ///< Damping factor
};

#endif // RAMSETE_CONTROLLER_H