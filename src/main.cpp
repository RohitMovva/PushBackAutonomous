#include "main.h"
#include "routes.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

// Global Vars

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-10, -12});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({9, 11});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor intake(6);
const int GOAL_CLAMP_PORT = 8;
pros::ADIDigitalOut sensor (GOAL_CLAMP_PORT, LOW);
bool clampState = LOW;

pros::Imu imu_sensor(21);

pros::adi::Encoder side_encoder('A', 'B', false);  // Ports 'A' and 'B' for the shaft encoder

// Program types
// std::string program_type = "driver";
std::string program_type = "autonomous";
// std::string program_type = "calibrate_metrics";

// Routes
std::vector<std::vector<float>> route = testroute5;
// std::vector<std::vector<float>> route = close_side;
// std::vector<std::vector<float>> route = far_side;
// std::vector<std::vector<float>> route = skills;
// std::vector<std::vector<float>> route = {}; // Driver or Calibration



float initial_heading;
// pair<float> initial_coordinate;

// Robot parameters (needs to be tweaked later)
const float WHEEL_DIAMETER = 4.0;  // Diameter of the wheels in inches
// TODO update for blue cartridges on new bot
const float TICKS_PER_ROTATION = 900.0;  // Encoder ticks per wheel rotation for green cartridges
const float WHEEL_BASE_WIDTH = 12.0;  // Distance between the left and right wheels in inches
const float DT = 0.025;  // Time step in seconds (25 ms)

// PID Code
// PID parameters (need to be tuned, especially heading)
float kp_position = 1.48;
float ki_position = 0.14;
float kd_position = 0.3;

float kp_heading = .0375;
float ki_heading = 0.0;
float kd_heading = 0.0;

// Structure to store the robot's position
struct Position {
    float x;
    float y;
    float heading;
};

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float intergral_max=5)
        : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0), intergral_max(intergral_max) {}

    float compute(float setpoint, float current_value, float dt) {
        float error = setpoint - current_value;
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


// Function to convert encoder ticks to distance in inches
float ticks_to_feet(int ticks) {
    return (ticks / TICKS_PER_ROTATION) * (M_PI * WHEEL_DIAMETER) * (7.0/5.0) / 12.0;
}

// Function to convert velocity from feet per second to RPM
float ft_per_sec_to_rpm(float velocity_ft_per_sec) {
    float wheel_circumference_ft = (M_PI * WHEEL_DIAMETER) / 12.0;
    return (velocity_ft_per_sec * 60.0) / wheel_circumference_ft;
}

// Function to get the robot's current position using encoders
Position get_robot_position(Position current_position) {
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
    float current_heading = (-1*imu_sensor.get_heading()) + initial_heading;
    if (current_heading < -180){
        current_heading += 360;
    }

    // Calculate the change in position
    float delta_x = forward_distance * cos(current_heading * M_PI / 180.0) - lateral_distance * sin(current_heading * M_PI / 180.0);
    float delta_y = forward_distance * sin(current_heading * M_PI / 180.0) + lateral_distance * cos(current_heading * M_PI / 180.0);

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
    pros::lcd::print(1, "Left velocity: %2.f", left_speed);
    pros::lcd::print(2, "Right velocity: %2.f", right_speed);
    left_mg.move_velocity(left_speed);
    right_mg.move_velocity(right_speed);
}

void PID_controller(){
	double dt = DT;  // Time step in seconds (25 ms)

    double goal_x = 0.0;
    double goal_y = 0.0;
	initial_heading = route[1][2];
	// initial_coordinates = std::make_pair(route[1][0], route[1][1]);
    // pros::lcd::print(0, "Route size: %i", route.size());
    Position current_position;
    current_position.heading = 0;
    current_position.y = route[1][1] / 12.0;
    current_position.x = route[1][0] / 12.0;

    int index = 0;  // Index to iterate over the velocity_heading vector

    left_mg.tare_position_all();
    right_mg.tare_position_all();
    side_encoder.reset();
    pros::lcd::print(3, "Route size: %i", route.size());
    bool correcting_heading = false;
    bool reversed = false;
    while (index < route.size()) { //  || x_pid.previous_error > 0.05 || y_pid.previous_error > 0.05 || heading_pid.previous_error > 2
        if (index == 0){
            current_position.heading = 0;
            current_position.y = route[1][1] / 12.0;
            current_position.x = route[1][0] / 12.0;
        }
        // Get the setpoints from the velocity_heading vector
        while (route[index].size() > 3){ // Normal is {x, y, heading} if we have more in sub vector then it is a node
            // Turn here
            intake.move(127*route[index][0]); // Move here
            // Clamp goal here

            if (route[index][2] == 1){
                reversed = !reversed; // Toggle reverse
                self.current_position.heading -= 180; // We are going in the other direction now, need to flip headings to match with target headings
            }

            if (route[index][4] != 0){ // Double check if we need this part
                left_mg.move_velocity(0);
                right_mg.move_velocity(0);
            }
            pros::delay(route[index][4]*1000); // Wait for time

            index++;
        }
        current_position = get_robot_position(current_position);
        // float setpoint_velocity = route[index][0];
        // float setpoint_heading = route[index][1];
        goal_x = route[index][0] / 12.0;
        goal_y = route[index][1] / 12.0;
        float setpoint_heading = route[index][2];

        // Center heading around 0 degrees
        if (setpoint_heading > 180){
            setpoint_heading -= 360;
        } if (setpoint_heading < -180){
            setpoint_heading += 360;
        }

        // // Update the goal position based on setpoint_velocity and setpoint_heading
        // if (!correcting_heading){
        //     goal_x += setpoint_velocity * DT * cos(setpoint_heading * M_PI / 180.0);
        //     goal_y += setpoint_velocity * DT * sin(setpoint_heading * M_PI / 180.0);
        // }

        // Get the current position and heading
        float old_heading = route[std::max(index-1, 1)][1];
        pros::lcd::print(5, "Headings: %f, %f, %f", setpoint_heading, current_position.heading, old_heading);
        pros::lcd::print(6, "X positions: %f, %f", goal_x*12, current_position.x*12);
        pros::lcd::print(7, "Y positions: %f, %f", goal_y*12, current_position.y*12);

        // Compute the control signals for x, y, and heading
        float x_control_signal = x_pid.compute(goal_x, current_position.x, dt);
        float y_control_signal = y_pid.compute(goal_y, current_position.y, dt);
        float heading_control_signal = heading_pid.compute(setpoint_heading, current_position.heading, dt);
        pros::lcd::print(4, "Index: %i", index);
        // pros::lcd::print(4, "Control Signals: %f, %f", x_control_signal, y_control_signal);
        // Combine x and y control signals to get the overall linear velocity
        float linear_velocity = sqrt(pow(x_control_signal, 2) + pow(y_control_signal, 2));
        if (reversed){ // Check this since there is a good chance this is incorrect
            linear_velocity *= -1
            heading_control_signal *= -1
        }
        // Apply the control signals to the motors
        apply_control_signal(linear_velocity, heading_control_signal);

        // Sleep for the time step duration
        pros::delay(dt * 1000);

        // Move to the next setpoint
        index++;
    }
    left_mg.move_velocity(0);
    right_mg.move_velocity(0);
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
	}
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
	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs
        
		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage


        if (master.get_digital(DIGITAL_L1)){
            intake.move_velocity(200);
        } else if (master.get_digital(DIGITAL_L2)){
            intake.move_velocity(-200);
        }  else {
            intake.move_velocity(0);
        }

        if (master.get_digital(DIGITAL_R1)){
            if (!fired){
                sensor.set_value(clampState);
                clampState = !clampState;
            }
            fired = true;
        } else{
            fired = false;
        }
        
		pros::delay(20);                               // Run for 20 ms then update
	}
}