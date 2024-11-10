#include "../../include/controllers/ramsete_controller.h"

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
                                                                double gear_ratio) {
    std::vector<double> wheel_velocities;
    double linear_motor_velocity = linear_velocity * 60.0 / (M_PI * wheel_diameter) / gear_ratio;
    double angular_motor_velocity = angular_velocity * 60.0 / (M_PI * wheel_diameter) / gear_ratio;
    
    wheel_velocities.push_back(linear_motor_velocity + angular_motor_velocity);
    wheel_velocities.push_back(linear_motor_velocity - angular_motor_velocity);
    
    return wheel_velocities;
}

std::vector<double> RamseteController::calculate(double x, double y, double theta,
                                               double goal_x, double goal_y, double goal_theta,
                                               double v_ref, double w_ref) {
    // Calculate errors in global frame
    std::vector<double> global_error = calculate_global_error(x, y, theta, goal_x, goal_y, goal_theta);
    
    // Transform errors to robot frame using explicit rotation matrix
    double e_x = std::cos(theta) * global_error[0] + std::sin(theta) * global_error[1];
    double e_y = -std::sin(theta) * global_error[0] + std::cos(theta) * global_error[1];
    double e_theta = global_error[2];
    
    // Calculate distance to goal and heading to goal
    double distance_to_goal = std::sqrt(global_error[0] * global_error[0] + global_error[1] * global_error[1]);
    double heading_to_goal = std::atan2(global_error[1], global_error[0]);
    
    // Calculate heading error (difference between current heading and heading to goal)
    double heading_error = heading_to_goal - theta;
    while (heading_error > M_PI) heading_error -= 2 * M_PI;
    while (heading_error < -M_PI) heading_error += 2 * M_PI;
    
    // Calculate control gains
    double k = 2.0 * zeta_ * std::sqrt(w_ref * w_ref + b_ * v_ref * v_ref);
    
    // Initialize control outputs
    double v = v_ref;
    double w = w_ref;
    
    // Adjust velocity based on heading error
    double heading_factor = std::cos(heading_error);
    v *= std::max(0.1, heading_factor);
    
    // Calculate control inputs using standard RAMSETE formulation
    v += k * e_x;
    
    // Special handling for angular velocity depending on distance to goal
    if (distance_to_goal > 0.5) {
        // When far from goal, prioritize heading correction
        w = b_ * v_ref * std::sin(heading_error) / heading_error * e_y + k * heading_error;
    } else {
        // When close to goal, focus on final orientation
        double orientation_error = goal_theta - theta;
        while (orientation_error > M_PI) orientation_error -= 2 * M_PI;
        while (orientation_error < -M_PI) orientation_error += 2 * M_PI;
        
        w = k * orientation_error;
    }
    
    // Apply progressive velocity reduction as we approach the goal
    double distance_factor = std::min(1.0, distance_to_goal / 1.0);
    v *= distance_factor;
    
    // Apply velocity limits with smooth saturation
    const double max_v = 1.0;
    const double max_w = 1.0;
    
    v = max_v * std::tanh(v / max_v);
    w = max_w * std::tanh(w / max_w);
    
    // Special case: if very close to goal but orientation is off, prioritize rotation
    if (distance_to_goal < 0.1 && std::abs(e_theta) > 0.1) {
        v *= 0.1;  // Reduce forward velocity to focus on rotation
        w = k * e_theta;  // Direct orientation correction
    }
    
    std::vector<double> output;
    output.push_back(v);
    output.push_back(w);
    
    return output;
}