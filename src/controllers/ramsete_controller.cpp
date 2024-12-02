#include "controllers/ramsete_controller.h"
#include "api.h"

RamseteController::RamseteController()
    : b_(2.0)
    , zeta_(0.7)
{
}

RamseteController::RamseteController(double b, double zeta)
    : b_(b)
    , zeta_(zeta)
{
}

RamseteController::RamseteController(double b, double zeta, double max_v, double max_w, double scale_factor)
    : b_(b)
    , zeta_(zeta)
    , max_v_(max_v * scale_factor)
    , max_w_(max_w)
    , scale_factor_(scale_factor)
{
}



std::vector<double> RamseteController::calculate_global_error(double x, double y, double theta,
                                                            double goal_x, double goal_y, double goal_theta) {
    std::vector<double> error;
    // Simple difference in global coordinates
    error.push_back(goal_x - x);  // Changed from (x - goal_x) to (goal_x - x)
    error.push_back(goal_y - y);  // Changed from (y - goal_y) to (goal_y - y)
    
    // Normalize theta error to [-π, π]
    double theta_error = goal_theta - theta;
    while (theta_error > M_PI) theta_error -= 2 * M_PI;
    while (theta_error < -M_PI) theta_error += 2 * M_PI;
    error.push_back(theta_error);
    
    return error;
}

std::vector<double> RamseteController::calculate_local_error(std::vector<double> global_error, double theta) {
    std::vector<double> local_error;
    
    // Transform global error to robot frame
    local_error.push_back(global_error[0] * std::cos(theta) + global_error[1] * std::sin(theta));
    local_error.push_back(-global_error[0] * std::sin(theta) + global_error[1] * std::cos(theta));
    local_error.push_back(global_error[2]);  // Angular error remains the same
    
    return local_error;
}

std::vector<double> RamseteController::get_output_velocities(double v, double w, double theta,
                                                           double goal_theta, double k) {
    std::vector<double> output_velocities;
    
    double theta_error = goal_theta - theta;
    while (theta_error > M_PI) theta_error -= 2 * M_PI;
    while (theta_error < -M_PI) theta_error += 2 * M_PI;
    
    if (std::abs(theta_error) < 1e-6) {
        output_velocities.push_back(v);
        output_velocities.push_back(w);
    } else {
        double v_output = v * std::cos(theta_error);
        double w_output = w + k * theta_error;
        
        // Apply velocity limits
        apply_velocity_limits(v_output, w_output);
        
        output_velocities.push_back(v_output);
        output_velocities.push_back(w_output);
    }
    
    return output_velocities;
}


std::vector<double> RamseteController::calculate_wheel_velocities(double linear_velocity,
                                                                double angular_velocity,
                                                                double wheel_diameter,
                                                                double gear_ratio,
                                                                double track_width) {
    std::vector<double> wheel_velocities;
    
    // Calculate wheel speed difference based on angular velocity
    // angular_velocity (rad/s) * track_width/2 (inches) = inches/sec
    double wheel_speed_diff = angular_velocity * track_width / 2.0;
    
    // Calculate left and right wheel velocities in inches/sec
    wheel_velocities.push_back(linear_velocity - wheel_speed_diff);
    wheel_velocities.push_back(linear_velocity + wheel_speed_diff);
    
    return wheel_velocities;
}

std::vector<double> RamseteController::calculate(double x, double y, double theta,
                                               double goal_x, double goal_y, double goal_theta,
                                               double v_ref, double w_ref) {

    // std::cout << "Recieved w ref: " << w_ref << "\n";
    // Scale inputs to meters
    x *= scale_factor_;
    y *= scale_factor_;
    goal_x *= scale_factor_;
    goal_y *= scale_factor_;
    v_ref *= scale_factor_;

    // Calculate global errors and transform to robot frame
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double e_x = (goal_x - x) * cos_theta + (goal_y - y) * sin_theta;
    double e_y = -(goal_x - x) * sin_theta + (goal_y - y) * cos_theta;
    
    // Normalize theta error to [-π, π]
    double e_theta = goal_theta - theta;
    while (e_theta > M_PI) e_theta -= 2 * M_PI;
    while (e_theta < -M_PI) e_theta += 2 * M_PI;

    // Calculate distance error for gain adjustment
    double distance_error = std::sqrt(e_x * e_x + e_y * e_y);
    
    // Calculate gains with enhanced stability
    bool is_low_speed = std::abs(v_ref) < 0.5;
    double k = 2.0 * zeta_ * std::sqrt(w_ref * w_ref + b_ * std::abs(v_ref) * std::abs(v_ref));
    
    // Calculate velocities with safe handling of small angles
    double v = v_ref * std::cos(e_theta) + k * e_x;
    double w = w_ref;
    
    if (std::abs(e_theta) < 1e-6) {
        w += b_ * v_ref * e_y + k * e_theta;
    } else {
        w += b_ * v_ref * (std::sin(e_theta) / e_theta) * e_y + k * e_theta;
    }

    // Apply velocity limits with low-speed consideration
    if (is_low_speed && false) {
        double limit = std::abs(v_ref) * 1.1;
        v = std::clamp(v, -limit, limit);
    } else {
        v = std::clamp(v, -max_v_, max_v_);
    }
    w = std::clamp(w, -max_w_, max_w_);
    
    // Convert linear velocity back to user units
    v /= scale_factor_;
    
    return {v, w};
}
