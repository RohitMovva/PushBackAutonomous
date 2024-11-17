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
    static double true_prev_x = x * scale_factor_;
    static double true_prev_y = y * scale_factor_;
    static double true_prev_time = 0.0;
    const double dt = 0.02;

    // Convert positions from user units to meters
    x *= scale_factor_;
    y *= scale_factor_;
    goal_x *= scale_factor_;
    goal_y *= scale_factor_;
    v_ref *= scale_factor_;

    // Calculate actual velocity using scaled positions
    double dx = (x - true_prev_x) / dt;
    double dy = (y - true_prev_y) / dt;
    double actual_velocity = std::sqrt(dx*dx + dy*dy);

    // Add velocity scaling factor to match reference frame
    actual_velocity /= scale_factor_;
    dx /= scale_factor_;
    dy /= scale_factor_;

    // Calculate errors in global frame with proper scaling
    std::vector<double> global_error = calculate_global_error(x/scale_factor_, y/scale_factor_, 
                                                            theta, goal_x/scale_factor_, 
                                                            goal_y/scale_factor_, goal_theta);
    
    // Transform errors to robot frame (errors are now in user units)
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    double e_x = cos_theta * global_error[0] + sin_theta * global_error[1];
    double e_y = -sin_theta * global_error[0] + cos_theta * global_error[1];
    double e_theta = global_error[2];

    double distance_error = std::sqrt(e_x * e_x + e_y * e_y);
    
    static double prev_v_ref = v_ref;
    static double prev_v = 0.0;
    static double prev_w = 0.0;
    static double prev_distance_error = distance_error;
    static double error_integral = 0.0;

    // Calculate if we're falling behind
    double error_direction = std::atan2(global_error[1], global_error[0]);
    double velocity_in_error_direction = dx * std::cos(error_direction) + 
                                       dy * std::sin(error_direction);
    bool falling_behind = velocity_in_error_direction < v_ref && distance_error > 0.1;

    // Debug output
    if (true_prev_time >= 0.74 && true_prev_time <= 0.78) {
        double commanded_v = prev_v * scale_factor_;
        double commanded_dx = commanded_v * std::cos(theta);
        double commanded_dy = commanded_v * std::sin(theta);
        
        std::cout << "\nDebug output for time: " << true_prev_time << "s"
                  << "\nPosition: (" << x/scale_factor_ << ", " << y/scale_factor_ << ")"
                  << "\nTarget: (" << goal_x/scale_factor_ << ", " << goal_y/scale_factor_ << ")"
                  << "\nActual velocity components: dx=" << dx << ", dy=" << dy 
                  << "\nActual velocity magnitude: " << actual_velocity 
                  << "\nCommanded velocity: " << commanded_v
                  << "\nCommanded components: dx=" << commanded_dx << ", dy=" << commanded_dy
                  << "\nReference velocity: " << v_ref
                  << "\nRobot heading: " << theta
                  << "\nPosition change: dx=" << (x - true_prev_x) << ", dy=" << (y - true_prev_y)
                  << "\nError in global frame: " << global_error[0] << ", " << global_error[1]
                  << "\nError in robot frame: ex=" << e_x << ", ey=" << e_y
                  << "\nPrevious v,w: " << prev_v << ", " << prev_w 
                  << "\nFalling behind: " << falling_behind << "\n";
    }
    
    // Store true previous values for next iteration
    true_prev_x = x;
    true_prev_y = y;
    true_prev_time += dt;

    // State detection
    double v_ref_rate = (v_ref - prev_v_ref) / dt;
    bool is_low_speed = v_ref < 0.5 * scale_factor_;
    bool is_starting = prev_v < 0.1 * scale_factor_ && v_ref > prev_v_ref;
    bool is_decelerating = v_ref_rate < -0.2 * scale_factor_;
    bool is_high_speed = v_ref > 2.0 * scale_factor_;

    // Error tracking
    double error_rate = (distance_error - prev_distance_error) / dt;
    static double filtered_error_rate = error_rate;
    double error_filter = is_low_speed ? 0.8 : 0.6;
    filtered_error_rate = error_filter * filtered_error_rate + (1 - error_filter) * error_rate;
    bool error_growing = filtered_error_rate > 0.005;

    // Update error integral with anti-windup
    const double max_integral = falling_behind ? 1.0 : 0.5;
    if (!is_low_speed && !is_starting) {
        error_integral = std::clamp(error_integral + e_x * dt, -max_integral, max_integral);
    } else {
        error_integral = 0.0;
    }

    // Adaptive gains
    double k_base = 2.0 * zeta_ * std::sqrt(w_ref * w_ref + b_ * v_ref * v_ref);
    double k = k_base;
    
    if (falling_behind) {
        k *= 1.5;
        if (distance_error > 0.2) {
            k *= 1.0 + std::min(1.0, (distance_error - 0.2) * 2.0);
        }
    } else {
        if (is_low_speed) {
            k *= 1.1;
            if (is_starting) {
                k *= 0.8;
            }
        } else if (is_high_speed) {
            k *= 0.9;
            if (error_growing) {
                k *= 1.3;
            }
        }
    }

    if (is_decelerating && !falling_behind) {
        double decel_factor = std::min(1.0, std::abs(v_ref_rate));
        k *= (1.0 + 0.3 * decel_factor);
    }

    // Velocity calculation
    double v = v_ref * std::cos(e_theta);
    
    // Position correction
    double integral_gain = falling_behind ? 0.2 : 0.1;
    double position_correction = k * (e_x + integral_gain * error_integral);
    
    // Speed scaling
    double speed_scale = 1.0;
    if (falling_behind) {
        speed_scale = 1.0 + std::min(1.0, distance_error);
    } else {
        if (is_low_speed) {
            speed_scale = std::min(1.0, v_ref/scale_factor_ + 0.2);
        } else if (is_high_speed) {
            speed_scale = 0.95;
            if (error_growing) {
                speed_scale = 1.1;
            }
        }
    }
    
    position_correction *= speed_scale;
    if (!is_low_speed) {
        position_correction += (falling_behind ? 0.3 : 0.2) * filtered_error_rate;
    }
    
    v += position_correction;

    // Angular velocity calculation
    double sin_e_theta_over_e_theta = (std::abs(e_theta) < 1e-6) ? 1.0 : std::sin(e_theta) / e_theta;
    double w = w_ref + b_ * v_ref * sin_e_theta_over_e_theta * e_y + k * e_theta;

    // Acceleration limits
    double max_accel = scale_factor_ * (falling_behind ? 4.0 : (is_low_speed ? 2.0 : (is_high_speed ? 2.5 : 3.0)));
    if (is_starting) {
        max_accel *= 0.6;
    }
    
    double base_decel = scale_factor_ * (falling_behind ? 1.5 : (is_low_speed ? 2.0 : 3.0));
    double max_decel = base_decel;
    
    if (is_decelerating && !falling_behind) {
        if (distance_error > 0.1) {
            max_decel *= (1.0 + std::min(1.5, distance_error / 0.2));
        }
        if (v > v_ref * 1.1) {
            max_decel *= 1.5;
        }
    }

    // Acceleration limiting
    double v_accel = (v - prev_v) / dt;
    double target_accel = std::clamp(v_accel, -max_decel, max_accel);
    
    // Adaptive smoothing
    double smooth_factor = falling_behind ? 0.95 : (is_low_speed ? 0.6 : (is_high_speed ? 0.8 : 0.9));
    v = prev_v + (v_accel + smooth_factor * (target_accel - v_accel)) * dt;

    // Angular acceleration limiting
    double max_w_accel = scale_factor_ * (is_low_speed ? 1.5 : (is_high_speed ? 2.0 : 2.5));
    double w_accel = (w - prev_w) / dt;
    if (std::abs(w_accel) > max_w_accel) {
        w = prev_w + std::copysign(max_w_accel * dt, w_accel);
    }

    // Velocity limiting
    double v_limit = max_v_;
    if (falling_behind) {
        v_limit = std::min(max_v_, v_ref * (1.5 + std::min(1.0, distance_error)));
    } else {
        if (is_low_speed) {
            v_limit = std::min(v_limit, v_ref * 1.2);
        } else if (distance_error > 0.1) {
            v_limit *= std::max(0.85, 1.0 / (1.0 + 0.3 * distance_error));
        }
    }
    
    if (is_starting || is_low_speed) {
        v = std::max(0.0, std::min(v_limit, v));
    } else {
        v = std::clamp(v, -v_limit, v_limit);
    }
    
    w = std::clamp(w, -max_w_, max_w_);
    
    // Store values for next iteration
    prev_v = v;
    prev_w = w;
    prev_v_ref = v_ref;
    prev_distance_error = distance_error;

    // Convert linear velocity back to user units
    v /= scale_factor_;
    
    std::vector<double> output;
    output.push_back(v);
    output.push_back(w);

    return output;
}