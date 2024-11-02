#include <vector>

class RamseteController {
public:
    RamseteController() {
        b_ = 2.0;
        zeta_ = 0.7;
    }
    RamseteController(double b, double zeta) : b_(b), zeta_(zeta) {
    // Nothing to do here
    }

    // Calculate the global error of  the robot
    std::vector<double> calculate_global_error(double x, double y, double theta, double goal_x, double goal_y, double goal_theta) {
        std::vector<double> relative_error;
        relative_error.push_back((x - goal_x) * std::cos(goal_theta) + (y - goal_y) * std::sin(goal_theta));
        relative_error.push_back(-1 * (x - goal_x) * std::sin(goal_theta) + (y - goal_y) * std::cos(goal_theta));
        relative_error.push_back(theta - goal_theta);
        return relative_error;
    }

    // Apply transformation matrix to convert global error to local error
    std::vector<double> calculate_local_error(std::vector<double> global_error, double goal_theta) {
        std::vector<double> local_error;
        local_error.push_back(global_error[0] * std::cos(goal_theta) + global_error[1] * std::sin(goal_theta));
        local_error.push_back(-1 * global_error[0] * std::sin(goal_theta) + global_error[1] * std::cos(goal_theta));
        local_error.push_back(global_error[2]);
        return local_error;
    }

    // Get output velocioties from the controller
    std::vector<double> get_output_velocities(double v, double w, double theta, double goal_theta, double k) {
        std::vector<double> output_velocities;
        output_velocities.push_back(v * std::cos(theta - goal_theta) + k * calculate_local_error(calculate_global_error(v, w, theta, v, w, goal_theta), goal_theta)[0]);
        output_velocities.push_back(w + k * calculate_local_error(calculate_global_error(v, w, theta, v, w, goal_theta), goal_theta)[2] + b_ * v * calculate_local_error(calculate_global_error(v, w, theta, v, w, goal_theta), goal_theta)[1]);
        return output_velocities;
    }

    // Calculate the wheel velocities of the robot. Inputs are linear and angular velocities and wheel diameter
    std::vector<double> calculate_wheel_velocities(double linear_velocity, double angular_velocity, double wheel_diameter, double gear_ratio) {
        std::vector<double> wheel_velocities;
        linear_motor_velocity = linear_velocity * 60.0 / (M_PI * wheel_diameter) / gear_ratio; // Convert linear velocity to motor velocity in RPM
        angular_motor_velocity = angular_velocity * 60.0 / (M_PI * wheel_diameter) / gear_ratio; // Convert angular velocity to motor velocity in RPM
        wheel_velocities.push_back(linear_motor_velocity + angular_motor_velocity);
        wheel_velocities.push_back(linear_motor_velocity - angular_motor_velocity);
        return wheel_velocities;
    }
    

    // Get output from the RAMSETE controller, inputs are current state of the robot and goal state
    std::vector<double> calculate(double x, double y, double theta, double goal_x, double goal_y, double goal_theta, double v, double w) {
        std::vector<double> global_error = calculate_global_error(x, y, theta, goal_x, goal_y, goal_theta);
        std::vector<double> local_error = calculate_local_error(global_error, goal_theta);
        double k = 2.0 * zeta_ * std::sqrt(std::pow(w, 2) + b_ * std::pow(v, 2));
        std::vector<double> output_velocities = get_output_velocities(v, w, theta, goal_theta, k);
        return output_velocities;
    }

private:
  double b_;
  double zeta_;

};