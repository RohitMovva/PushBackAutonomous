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
const int DOINKER_PORT = 7;
const int COLOR_SORT_PORT = 6;
pros::adi::DigitalOut mogo_mech (GOAL_CLAMP_PORT, LOW);
pros::adi::DigitalOut doinker (DOINKER_PORT, LOW);
pros::adi::DigitalOut color_sort (COLOR_SORT_PORT, LOW);

bool clamp_state = LOW;
bool doinker_state = LOW;
bool color_sort_state = LOW;
float prev_heading;



pros::Rotation side_encoder(4); // Lateral tracking encoder (PORT MIGHT BE WRONG)
// Program types
// std::string program_type = "driver";
// std::string program_type = "autonomous";
std::string program_type = "autonomous";
// std::string program_type = "turn_test";
// std::string program_type = "calibrate_metrics";

// Routes
std::vector<std::vector<double>> route = test;
// std::vector<std::vector<float>> route = {}; // Driver or Calibration

// Robot parameters (needs to be tweaked later)
const float WHEEL_DIAMETER = 2.75;  // Diameter of the wheels in inches
// TODO update for blue cartridges on new bot
const float TICKS_PER_ROTATION = 300.0;  // Encoder ticks per wheel rotation for green cartridges
const float GEAR_RATIO = 36.0/48.0;  // Gear ratio of the drivetrain
const double WHEEL_BASE_WIDTH = 12.5;  // Distance between the left and right wheels in inches
const float DT = 0.025;  // Time step in seconds (25 ms)

// Function to convert encoder ticks to distance in inches
float ticks_to_feet(int ticks) {
    return (ticks / TICKS_PER_ROTATION) * (M_PI * WHEEL_DIAMETER) * GEAR_RATIO / 12.0;
}

// Function to convert velocity from feet per second to RPM
float ft_per_sec_to_rpm(float velocity_ft_per_sec) {
    float wheel_circumference_ft = (M_PI * WHEEL_DIAMETER) / 12.0;
    return (velocity_ft_per_sec * 60.0) / wheel_circumference_ft;
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
                     int timeout = 1e9) {
    
    if (route.empty()) return false;
    
    const double START_TIME = pros::millis();
    const double DT = 25;  // 25ms fixed timestep matching motion profile
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
                clamp_state = !clamp_state;
                mogo_mech.set_value(clamp_state);
            }
            // Doinker prolly

            // Lady brown

            // Color sort maybe

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
	if (program_type == "autonomous"){
        Odometry odometry(left_mg, right_mg, side_encoder, imu_sensor, WHEEL_BASE_WIDTH, 0.0, false, false, false);
        RamseteController ramsete(2.0, 0.7, 1.0, 1.0, 1.0);
        DrivetrainController drivetrain(5.0, 0.2, 0.05, 0.1, 0.001, 0.01);
        followTrajectory(route, odometry, ramsete, drivetrain, left_mg, right_mg);
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
    bool doinker_fired = false;
    bool hang_deployed = false;
    bool color_sort_fired = false;

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
                if (clamp_state){
                    master.rumble("-");
                } else {
                    master.clear_line(0);
                    master.clear_line(1);
                    master.clear_line(2);
                }
                clamp_state = !clamp_state;
                mogo_mech.set_value(clamp_state);
            }
            fired = true;
        } else {
            fired = false;
        }
        // color_sort_state
        if (master.get_digital(DIGITAL_DOWN)){
            if (!color_sort_fired){
                color_sort_state = !color_sort_state;
                color_sort.set_value(color_sort_state);
            }
            color_sort_fired = true;
        } else {
            color_sort_fired = false;
        }

        if (master.get_digital(DIGITAL_B)){
            if (!doinker_fired){
                doinker_state = !doinker_state;
                doinker.set_value(doinker_state);
            }
            doinker_fired = true;
        } else {
            doinker_fired = false;
        }
        
		pros::delay(20);                               // Run for 20 ms then update
	}
}