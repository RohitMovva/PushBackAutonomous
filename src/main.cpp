#include "main.h"
#include "pros/colors.h"
#include "routes/routes.h"
#include "navigation/odometry.h"
#include "controllers/ramsete_controller.h"
#include "controllers/drivetrain_controller.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

// Global Vars

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-13, -2, -15});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({16, 19, 18});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::MotorGroup intake({8, -10});

pros::Imu imu_sensor(5);

const int GOAL_CLAMP_PORT = 8;
const int HANG_PORT = 7;
pros::ADIDigitalOut mogo_mech (GOAL_CLAMP_PORT, LOW);
pros::ADIDigitalOut hang_mech (HANG_PORT, LOW);

bool clampState = LOW;
float prev_heading;



pros::adi::Encoder side_encoder('A', 'B', false);  // Ports 'A' and 'B' for the shaft encoder
// Program types
// std::string program_type = "driver";
// std::string program_type = "autonomous";
std::string program_type = "autonomous";
// std::string program_type = "turn_test";
// std::string program_type = "calibrate_metrics";

// Routes
std::vector<std::vector<float>> route = auton_skills;
// std::vector<std::vector<float>> route = solo_awp;
// std::vector<std::vector<float>> route = mirrored_solo_awp;
// std::vector<std::vector<float>> route = mogo_rush;
// std::vector<std::vector<float>> route = mirrored_mogo_side;
// std::vector<std::vector<float>> route = close_side;
// std::vector<std::vector<float>> route = far_side;
// std::vector<std::vector<float>> route = skills;
// std::vector<std::vector<float>> route = {}; // Driver or Calibration



float initial_x;
float initial_y;
float initial_heading;
// pair<float> initial_coordinate;

// Robot parameters (needs to be tweaked later)
const float WHEEL_DIAMETER = 2.75;  // Diameter of the wheels in inches
// TODO update for blue cartridges on new bot
const float TICKS_PER_ROTATION = 300.0;  // Encoder ticks per wheel rotation for green cartridges
const float GEAR_RATIO = 36.0/48.0;  // Gear ratio of the drivetrain
const float WHEEL_BASE_WIDTH = 12.0;  // Distance between the left and right wheels in inches
const float DT = 0.025;  // Time step in seconds (25 ms)

// PID Code
// PID parameters (need to be tuned, especially heading)
// float kp_position = 1.6;
// float ki_position = 0.0;
// float kd_position = 0.075;
float kp_position = 1.9;
float ki_position = 0.0;
float kd_position = 0.0;
float kp_heading = .016;
float ki_heading = 0.0005;
float kd_heading = 0.0006;
// float kp_heading = .0175;
// float ki_heading = 0.0005;
// float kd_heading = 0.00075;

float kp_heading_op = .016;
float ki_heading_op = 0.0005;
float kd_heading_op = 0.0006;

float kp_position_op = 1.9;
float ki_position_op = 0.0;
float kd_position_op = 0.00;

// Structure to store the robot's position
struct Position {
    float x;
    float y;
    float heading;
};

bool dumb = false;

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float intergral_max=5)
        : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0), intergral_max(intergral_max) {}

    float compute(float setpoint, float current_value, float dt, bool headingCentered=false) {
        float error = setpoint - current_value;
        if (headingCentered){
            if (error < -180.0){
                error += 360.0;
            } else if (error > 180.0){
                error -= 360.0;
            }
        }
        integral += error * dt;
        if (integral > 5){ // Anti integral wind up, may need tweaking
            integral = 0;
        }
        float derivative = (error - previous_error) / dt;
        
        float output = kp * error + ki * integral + kd * derivative;
        previous_error = error;
        
        return output;
    }

private:
    float kp, ki, kd;
    float integral;
    float intergral_max;
public:
    float previous_error;
};


PIDController x_pid(kp_position, ki_position, kd_position);
PIDController y_pid(kp_position, ki_position, kd_position);
PIDController heading_pid(kp_heading, ki_heading, kd_heading);
PIDController heading_pid_op(kp_heading_op, ki_heading_op, kd_heading_op);
PIDController dist_pid(kp_position_op, ki_position_op, kd_position_op);


// Function to convert encoder ticks to distance in inches
float ticks_to_feet(int ticks) {
    return (ticks / TICKS_PER_ROTATION) * (M_PI * WHEEL_DIAMETER) * GEAR_RATIO / 12.0;
}

// Function to convert velocity from feet per second to RPM
float ft_per_sec_to_rpm(float velocity_ft_per_sec) {
    float wheel_circumference_ft = (M_PI * WHEEL_DIAMETER) / 12.0;
    return (velocity_ft_per_sec * 60.0) / wheel_circumference_ft;
}

// Function to get the robot's current position using encoders
Position get_robot_position(Position current_position, bool reverse, float setpoint_heading=NULL) {
    // Get the encoder values
	std::vector<double> left_ticks = left_mg.get_position_all();
	std::vector<double> right_ticks = right_mg.get_position_all();

    // Calculate averages of every motor's tracked ticks
	double left_tick_avg = 0;
	for (auto& tick: left_ticks){
		left_tick_avg += tick;
	}
	left_tick_avg /= left_ticks.size();

	double right_tick_avg = 0;
	for (auto& tick: right_ticks){
		right_tick_avg += tick;
	}
	right_tick_avg /= right_ticks.size();

    // int side_ticks = side_encoder.get_value();
    pros::lcd::print(0, "Tick stuff: %f, %f", left_tick_avg, right_tick_avg);

    // Calculate distances
    float left_distance = ticks_to_feet(left_tick_avg);
    float right_distance = ticks_to_feet(right_tick_avg);
    // float side_distance = ticks_to_feet(side_ticks);

    // Calculate the forward and lateral displacement
    float forward_distance = (left_distance + right_distance) / 2.0;
    float lateral_distance = 0;
    // float lateral_distance = side_distance;

    // Get the current heading from the inertial sensor
    // float current_heading = setpoint_heading;
    float current_heading;
    if (setpoint_heading == NULL){
        // current_heading = -1*(float(imu_sensor.get_heading()) - initial_heading);
        current_heading = -1*(float(imu_sensor.get_heading()) - initial_heading);
    } else {
        current_heading = setpoint_heading;
    }

    // pros::lcd::print(6, "Munch: %2.f, %2.f, %2.f", current_heading, imu_sensor.get_heading(), (-1*float(imu_sensor.get_heading())));
    // if (reverse){
    //     current_heading += 180;
    // }
    if (current_heading < -180){
        current_heading += 360;
    } else if (current_heading > 180){
        current_heading -= 360;
    }

    // Calculate the change in position
    float delta_x = forward_distance * cos(current_heading * M_PI / 180.0) - lateral_distance * sin(current_heading * M_PI / 180.0);
    float delta_y = forward_distance * sin(current_heading * M_PI / 180.0) + lateral_distance * cos(current_heading * M_PI / 180.0);
    // pros::lcd::print(1, "DISTNACE TRAVELLED: %f", forward_distance);

    // if (reverse){
        // delta_x
        // continue;
    // }
    // Update the total position
    current_position.x += delta_x;
    current_position.y += delta_y;
    current_position.heading = current_heading;

    // Reset encoders after reading
    left_mg.tare_position_all();
    right_mg.tare_position_all();
    side_encoder.reset();

    return current_position;
}

void apply_control_signal(float linear_velocity, float angular_velocity) {
    float left_speed = ft_per_sec_to_rpm(linear_velocity - angular_velocity);
    float right_speed = ft_per_sec_to_rpm(linear_velocity + angular_velocity);
    // Don't think we need this stuff anymore hopefully, I'll keep it for now just in case
    // float maxVal = std::max(left_speed, right_speed);
    // if (maxVal > 200){
    //     left_speed *= (200/maxVal);
    //     right_speed *= (200/maxVal);
    // }
    // pros::lcd::print(1, "Left velocity: %2.f", left_speed);
    pros::lcd::print(2, "Right velocity: %2.f", right_speed);
    left_mg.move_velocity(left_speed);
    right_mg.move_velocity(right_speed);
}

void turn_on_point(double turn_val){
    double actual_heading = imu_sensor.get_heading();
    // pros::lcd::print(3, "og actual heading: %f", actual_heading);
    // pros::lcd::print(4, "og target heading: %f", turn_val);
    if (actual_heading > 180){
        actual_heading -= 360;
    } else if (actual_heading < -180){
        actual_heading += 360;
    }
    double target_heading = actual_heading + turn_val;
    if (target_heading > 180){
        target_heading -= 360;
    } else if (target_heading < -180){
        target_heading += 360;
    }
    int close_times = 0;
    while (true){
        if (fabs(target_heading - actual_heading) < 1.0){
            close_times++;
        } else {
            close_times = 0;
        }
        if (close_times > 2){
            break;
        }
        float heading_control_signal = -1*(heading_pid_op.compute(float(target_heading), float(actual_heading), DT));

        apply_control_signal(0.0, heading_control_signal);

        pros::delay(DT * 1000);

        actual_heading = imu_sensor.get_heading();
        if (actual_heading > 180.0){
            actual_heading -= 360.0;
        } else if (actual_heading < -180.0){
            actual_heading += 360.0;
        }
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
    // pros::lcd::print(5, "actual heading: %f", actual_heading);
    // pros::lcd::print(6, "target heading: %f", target_heading);
}

void travel_distance(double dist){
    double dist_traveled = 0.0;
    int close_times = 0;
    while (true){
        if (fabs(dist - dist_traveled) < 0.25){
            close_times++;
        } else {
            close_times = 0;
        }

        // if (close_times > 2){
        //     break;
        // }
        std::vector<double> left_ticks = left_mg.get_position_all();
        std::vector<double> right_ticks = right_mg.get_position_all();

        // Calculate averages of every motor's tracked ticks
        double left_tick_avg = 0;
        for (auto& tick: left_ticks){
            left_tick_avg += tick;
        }
        left_tick_avg /= left_ticks.size();

        double right_tick_avg = 0;
        for (auto& tick: right_ticks){
            right_tick_avg += tick;
        }
        right_tick_avg /= right_ticks.size();

        // int side_ticks = side_encoder.get_value();
        pros::lcd::print(0, "Tick stuff: %f, %f", left_tick_avg, right_tick_avg);

        // Calculate distances
        float left_distance = ticks_to_feet(left_tick_avg);
        float right_distance = ticks_to_feet(right_tick_avg);
        // float side_distance = ticks_to_feet(side_ticks);

        // Calculate the forward displacement (in feet)
        float forward_distance = (left_distance + right_distance) / 2.0;

        dist_traveled += forward_distance;

        float controlsignal = (dist_pid.compute(float(dist), float(dist_traveled), DT));
        pros::lcd::print(1, "Distances: %f, %f", dist, dist_traveled);

        apply_control_signal(controlsignal, 0.0);

        left_mg.tare_position_all();
        right_mg.tare_position_all();

        pros::delay(DT * 1000);
    }
}

void PID_controller(){
    pros::lcd::print(0, "PIDPIDPIDPID");
	double dt = DT;  // Time step in seconds (25 ms)

    double goal_x = 0.0;
    double goal_y = 0.0;
	initial_heading = route[1][2];
    // if (route[0][2] == 1){
    //     initial_heading += 180;
    //     // initial_heading = 180 - initial_heading;
    //     if (initial_heading > 180){
    //         initial_heading -= 360;
    //     } else if (initial_heading < -180){
    //         initial_heading += 360;
    //     }
    // }
    float old_heading = initial_heading;
    // pros::lcd::print(0, "Route size: %i", route.size());
    Position current_position;
    current_position.heading = 0;
    current_position.x = route[1][0] / 12.0;
    current_position.y = route[1][1] / 12.0;

    int index = 0;  // Index to iterate over the velocity_heading vector

    left_mg.tare_position_all();
    right_mg.tare_position_all();
    side_encoder.reset();
    pros::lcd::print(3, "Route size: %i", route.size());
    bool correcting_heading = false;
    bool reversed = false;
    bool mogo_mech_state = LOW;
    int kill_timer = -1;
    bool special_help_reverse = false;
    int overshot_timer = 0;
    while (index < route.size()) { //  || x_pid.previous_error > 0.05 || y_pid.previous_error > 0.05 || heading_pid.previous_error > 2
        // if (kill_timer == 0) break;
        if (index == 0){
            current_position.heading = 0;
            current_position.x = route[1][0] / 12.0;
            if (dumb){
                current_position.x *= -1;
            }
            current_position.y = route[1][1] / 12.0;
        }
        // Get the setpoints from the velocity_heading vector
        while (index < route.size() && route[index].size() > 3){ // Normal is {x, y, heading} if we have more in sub vector then it is a node
            if (route[index].size() == 6){
                // special_help_reverse = true;
                kill_timer = 10;
            }
            // Turn here
            if (route[index][3] != 0){
                double turn_val = double(route[index][3]);
                turn_val *= -1;
                // REMINDER uncomment this block if we switch back over the path planner
                // if (reversed){
                //     turn_val *= -1;
                // }
                // turn_on_point(double(turn_val));
            }
            
            intake.move(127*route[index][0]); // Move here

             // Toggle reverse
            if (route[index][2] == 1){
                reversed = !reversed;
            }

            if (route[index][4] != 0){ // Double check if we need this part
                left_mg.move_velocity(0);
                right_mg.move_velocity(0);
            }
            pros::delay(route[index][4]*1000*0); // Wait for time

            // Clamp goal here
            if (route[index][1]){
                mogo_mech_state = !mogo_mech_state;
                mogo_mech.set_value(mogo_mech_state);
            }

            index++;
        }
        if (index == route.size()){
            break;
        }
        current_position = get_robot_position(current_position, reversed);
        if (reversed){
            current_position.heading -= 180;
            if (current_position.heading < -180){
                current_position.heading += 360;
            }
        }
        goal_x = route[index][0] / 12.0;
        goal_y = route[index][1] / 12.0;
        float setpoint_heading = route[index][2];
        if (reversed){
            setpoint_heading += 180;
            if (setpoint_heading > 180){
                setpoint_heading -= 360;
            } else if (setpoint_heading < -180){
                setpoint_heading += 360;
            }
        }
        if (dumb){
            goal_x *= -1;
            setpoint_heading = 180 - setpoint_heading;
            if (setpoint_heading > 180){
                setpoint_heading -= 360;
            } else if (setpoint_heading < -180){
                setpoint_heading += 360;
            }
        }

        // Center heading around 0 degrees
        if (setpoint_heading > 180){
            setpoint_heading -= 360;
        } if (setpoint_heading < -180){
            setpoint_heading += 360;
        }

        pros::lcd::print(5, "Headings: %f, %f, %f", setpoint_heading, current_position.heading, old_heading);
        pros::lcd::print(6, "X positions: %f, %f", goal_x*12, current_position.x*12);
        pros::lcd::print(7, "Y positions: %f, %f", goal_y*12, current_position.y*12);

        // Compute the control signals for x, y, and heading
        float x_control_signal = x_pid.compute(goal_x, current_position.x, dt);
        float y_control_signal = y_pid.compute(goal_y, current_position.y, dt);

        if (current_position.heading > 0 && current_position.heading < 90
        ){
            if (current_position.x - goal_x > 0){
                overshot_timer++;
                x_control_signal *= 0.5;
                if (overshot_timer > 1){
                    x_control_signal = 0;
                }
            } if (current_position.y - goal_y > 0){
                overshot_timer++;
                y_control_signal *= 0.5;
                if (overshot_timer > 1){
                    y_control_signal = 0;
                }
            }
            if (!(current_position.x - goal_x > 0) && !(current_position.y - goal_y > 0)){
                overshot_timer = 0;
            }
        } if (current_position.heading < 180 && current_position.heading > 90){
            if (current_position.x - goal_x < 0){
                overshot_timer++;
                x_control_signal *= 0.5;
                if (overshot_timer > 1){
                    x_control_signal = 0;
                }
            } if (current_position.y - goal_y > 0){
                overshot_timer++;
                y_control_signal *= 0.5;
                if (overshot_timer > 1){
                    y_control_signal = 0;
                }
            }
            if (!(current_position.x - goal_x < 0) && !(current_position.y - goal_y > 0)){
                overshot_timer = 0;
            }
        } if (current_position.heading < -90 && current_position.heading > -180){
            if (current_position.x - goal_x < 0){
                overshot_timer++;
                x_control_signal *= 0.5;
                if (overshot_timer > 1){
                    x_control_signal = 0;
                }
            } if (current_position.y - goal_y < 0){
                overshot_timer++;
                y_control_signal *= 0.5;
                if (overshot_timer > 1){
                    y_control_signal = 0;
                }
            }
            if (!(current_position.x - goal_x < 0) && !(current_position.y - goal_y < 0)){
                overshot_timer = 0;
            }
        } if (current_position.heading < 0 && current_position.heading > -90){
            if (current_position.x - goal_x > 0){
                overshot_timer++;
                x_control_signal *= 0.5;
                if (overshot_timer > 1){
                    x_control_signal = 0;
                }
            } if (current_position.y - goal_y < 0){
                overshot_timer++;
                y_control_signal *= 0.5;
                if (overshot_timer > 1){
                    y_control_signal = 0;
                }
            }
            if (!(current_position.x - goal_x > 0) && !(current_position.y - goal_y < 0)){
                overshot_timer = 0;
            }
        }
        float heading_control_signal = heading_pid.compute(setpoint_heading, current_position.heading, dt, true);

        pros::lcd::print(4, "Control Signals: %f, %f, %f", x_control_signal, y_control_signal, heading_control_signal);
       
        // Combine x and y control signals to get the overall linear velocity
        float linear_velocity = sqrt(pow(x_control_signal, 2) + pow(y_control_signal, 2));
        if ((goal_x > current_position.x) && (goal_y > current_position.y)){
            
        }
        if (reversed){
            linear_velocity *= -1;
        }
        // Apply the control signals to the motors
        apply_control_signal(linear_velocity, heading_control_signal);

        old_heading = current_position.heading;

        // Sleep for the time step duration
        pros::delay(dt * 1000);

        // Move to the next setpoint
        index++;

        kill_timer--;
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
    pros::delay(1000);
}

/**
 * @brief Follow a 2D motion profile using RAMSETE and drivetrain controllers
 * 
 * @param route Vector of trajectory points {t, x, y, theta, v, w}
 * @param odometry Odometry instance for position tracking
 * @param ramsete RAMSETE controller for trajectory following
 * @param drivetrain Drivetrain controller for motor control
 * @param left_mg Left motor group
 * @param right_mg Right motor group
 * @param timeout Maximum time to spend following trajectory (ms)
 * @return bool True if trajectory was completed successfully
 */
bool followTrajectory(const std::vector<std::vector<double>>& route,
                     Odometry& odometry,
                     RamseteController& ramsete,
                     DrivetrainController& drivetrain,
                     pros::MotorGroup& left_mg,
                     pros::MotorGroup& right_mg,
                     int timeout = 5000) {
    
    if (route.empty()) return false;
    
    const double START_TIME = pros::millis();
    const double DT = 20;  // 20ms fixed timestep matching motion profile
    const double track_width = 15.0; // inches, adjust based on your robot
    size_t trajectory_index = 0;
    
    // Main control loop
    while (trajectory_index < route.size()) {
        const double current_time = pros::millis() - START_TIME;
        
        // Timeout check
        if (current_time > timeout) {
            return false;
        }
        
        // Get current state from odometry
        odometry.update();
        Pose current_pose = odometry.getPose();
        auto velocities = odometry.getFilteredVelocities();
        
        // Check if the current waypoint is a node instead of a trajectory point
        while (trajectory_index < route.size() && route[trajectory_index].size() < 6) {
            // Handle node
            const auto& node = route[trajectory_index];
            
            // Move intake (value is either -1, 0, or 1)
            intake.move(127*node[0]);

            // Toggle clamp state
            if (node[1]){
                clampState = !clampState;
                mogo_mech.set_value(clampState);
            }

            // Toggle reverse
            // if (node[2] == 1){
            //     reversed = !reversed;
            // }

            trajectory_index++;
        }
        if (trajectory_index >= route.size()) {
            break;
        }

        // Get desired state from trajectory
        const auto& waypoint = route[trajectory_index];
        const double goal_x = waypoint[1];
        const double goal_y = waypoint[2];
        const double goal_theta = waypoint[3];
        const double goal_v = waypoint[4];
        const double goal_w = waypoint[5];
        
        // Calculate accelerations using next waypoint
        double left_accel = 0.0;
        double right_accel = 0.0;
        if (trajectory_index + 1 < route.size()) {
            const auto& next = route[trajectory_index + 1];
            
            // Calculate wheel velocities at current and next timestep
            const double current_left = goal_v - (goal_w * track_width / 2.0);
            const double current_right = goal_v + (goal_w * track_width / 2.0);
            
            const double next_v = next[4];
            const double next_w = next[5];
            const double next_left = next_v - (next_w * track_width / 2.0);
            const double next_right = next_v + (next_w * track_width / 2.0);
            
            // Calculate acceleration over the fixed timestep
            left_accel = (next_left - current_left) / (DT / 1000.0);  // Convert ms to seconds
            right_accel = (next_right - current_right) / (DT / 1000.0);
        }
        
        // Get RAMSETE controller output
        auto ramsete_output = ramsete.calculate(
            current_pose.x, current_pose.y, current_pose.theta,
            goal_x, goal_y, goal_theta,
            goal_v, goal_w
        );
        
        // Convert RAMSETE output to wheel velocities
        auto wheel_velocities = ramsete.calculate_wheel_velocities(
            ramsete_output[0], // linear velocity
            ramsete_output[1], // angular velocity
            2.75,             // wheel diameter (inches)
            48.0/36.0         // gear ratio
        );
        
        // Calculate motor voltages using drivetrain controller
        auto voltages = drivetrain.calculateVoltages(
            wheel_velocities[0], // left velocity setpoint
            wheel_velocities[1], // right velocity setpoint
            velocities.first,    // current left velocity
            velocities.second,   // current right velocity
            left_accel,         // left acceleration
            right_accel         // right acceleration
        );
        
        // Apply voltages to motors
        left_mg.move(voltages.left);
        right_mg.move(voltages.right);
        
        // Check if we're at the final waypoint
        if (trajectory_index == route.size() - 1) {
            const double position_tolerance = 1.0;  // inches
            const double heading_tolerance = 0.1;   // radians
            
            const double position_error = std::sqrt(
                std::pow(current_pose.x - goal_x, 2) +
                std::pow(current_pose.y - goal_y, 2)
            );
            const double heading_error = std::abs(current_pose.theta - goal_theta);
            
            if (position_error < position_tolerance && heading_error < heading_tolerance) {
                // Stop motors
                left_mg.move(0);
                right_mg.move(0);
                return true;
            }
        }
        
        // Increment trajectory index based on fixed timestep
        if (current_time >= (trajectory_index + 1) * DT) {
            trajectory_index++;
        }
        
        // Wait until next timestep
        const double elapsed = pros::millis() - START_TIME;
        const double next_timestep = (trajectory_index + 1) * DT;
        const double delay_time = next_timestep - elapsed;
        if (delay_time > 0) {
            pros::delay(delay_time);
        }
    }
    
    // Stop motors
    left_mg.move(0);
    right_mg.move(0);
    return true;
}

// Structure to store velocity, acceleration, and jerk
struct MotionMetrics {
    float max_velocity;
    float max_acceleration;
    float max_jerk;
};

// Function to measure motion metrics
void measure_motion_metrics() {
    std::vector<float> velocities;
    std::vector<float> accelerations;

    float max_velocity = INT_MIN;
    float max_acceleration = INT_MIN;
    float max_jerk = INT_MIN;

    // Start the motors at maximum speed
    left_mg.move_velocity(200);  // Max velocity in RPM for VEX V5 motors
    right_mg.move_velocity(200);

    for (int i = 0; i < 100; ++i) {  // Run for 2 seconds (200 * 10ms)
        std::vector<double> left_ticks = left_mg.get_position_all();
		std::vector<double> right_ticks = right_mg.get_position_all();

		float left_tick_avg = 0;
		for (auto& tick: left_ticks){
			left_tick_avg += tick;
		}
		left_tick_avg /= left_ticks.size();

		float right_tick_avg = 0;
		for (auto& tick: right_ticks){
			right_tick_avg += tick;
		}
		right_tick_avg /= right_ticks.size();

        pros::lcd::print(3, "Tick Things: ", left_tick_avg, " ", right_tick_avg);
        float left_distance = ticks_to_feet(left_tick_avg);
        float right_distance = ticks_to_feet(right_tick_avg);

        float velocity = (left_distance + right_distance) / 2.0 / (DT/10);
        velocities.push_back(velocity);

        if (velocity > max_velocity) {
            max_velocity = velocity;
        }

        pros::delay((DT/10) * 1000);  // Delay for DT seconds

        left_mg.tare_position_all();
        right_mg.tare_position_all();
    }

    // Stop the motors
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);

    // Calculate accelerations
    for (size_t i = 1; i < velocities.size(); ++i) {
        float acceleration = (velocities[i] - velocities[i - 1]) / (DT/10); // Was I high when I wrote this?
        accelerations.push_back(acceleration);

        if (acceleration > max_acceleration) {
            max_acceleration = acceleration;
        }
    }

    // Calculate jerk
    for (size_t i = 1; i < accelerations.size(); ++i) { // Don't calculate max jerk lmao
        float jerk = (accelerations[i] - accelerations[i - 1]) / (DT/10); // Same with this lmao wtf?

        if (jerk > max_jerk) {
            max_jerk = jerk;
        }
    }

	pros::lcd::initialize();
    pros::lcd::print(0, "Max Velocity: %.2f ft/s", max_velocity);
    pros::lcd::print(1, "Max Acceleration: %.2f ft/s^2", max_acceleration);
    pros::lcd::print(2, "Max Jerk: %.2f ft/s^3", max_jerk);
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);

	left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    // pinMode(1, OUTPUT);

	// Calibrate the inertial sensor
    imu_sensor.reset();

	// autonomous() // For outside of competition testing purposes
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	if (program_type == "calibrate_metrics"){ // Extremely broken function lol
		measure_motion_metrics();
	} else if (program_type == "autonomous"){
    	PID_controller();
	} else if (program_type == "turn_test"){
        turn_on_point(-90.0);
    } else if (program_type == "dist_test"){
        travel_distance(2.0);
    }
}

int joystickCurve(int x, double a=2.5){
    return 
    int(((127.0 * std::pow(double(x), std::abs(a)))/(std::pow(127.0, a)))
     * (double(x)/std::abs(double(x))));
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    bool fired = false;
    bool hang_deployed = false;

    bool intake_forward = false;
    bool intake_reverse = false;
	while (true) {
		// Arcade control scheme
		int dir = (master.get_analog(ANALOG_LEFT_Y));    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage

        if (master.get_digital(DIGITAL_R1)){
            if (!intake_forward){
                intake.move_velocity(200);
            }
            intake_forward = true;
        } else {
            intake_forward = false;
        }

        if (master.get_digital(DIGITAL_R2)){
                intake.move_velocity(-200);
        }

        else if (!intake_forward){
            intake.move_velocity(0);
        }

        if (master.get_digital(DIGITAL_L1)){
            if (!fired){
                if (clampState){
                    master.rumble("-");
                } else {
                    master.clear_line(0);
                    master.clear_line(1);
                    master.clear_line(2);
                }
                mogo_mech.set_value(clampState);
                clampState = !clampState;
                
            }
            fired = true;
        } else {
            fired = false;
        }

        if (clampState){
            // pros::screen::set_pen(pros::Color::red);
            
        } else {
            pros::screen::set_pen(pros::Color::green);
        }

        if (master.get_digital(DIGITAL_B) && !hang_deployed){
            hang_mech.set_value(HIGH);
            hang_deployed = true;
        }
        
		pros::delay(20);                               // Run for 20 ms then update
	}
}