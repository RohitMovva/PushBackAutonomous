#include "main.h"
#include <cstring>
#include <iostream>

class LadyBrown {
private:
    pros::Motor lift_motor;
    std::atomic<int> target_state{0};
    int reset_countdown = 0;
    
    // PID constants - these may need tuning
    const double kP = 1.3;
    const double kI = 0.3;
    const double kD = 0.7;
    
    // Position setpoints in degrees (adjusted for gear ratio)
    // With 12:64 ratio, multiply desired angles by (64/12) = 5.33
    const double STATE_POSITIONS[3] = {
        0.0,
        16.00 * (64.0/12.0),    // ~53.3 degrees at motor
        117.5 * (64.0/12.0)    // ~586.7 degrees at motor
    };
    
    // PID variables
    double integral = 0;
    double prev_error = 0;
    
    // Helper function to calculate PID
    double calculatePID(double current_pos, double target_pos) {
        double error = target_pos - current_pos;
        
        // Update integral with anti-windup
        integral = integral + error;
        if (std::abs(integral) > 50) {  // Limit integral windup
            integral = (integral > 0) ? 50 : -50;
        }
        
        // Calculate derivative
        double derivative = error - prev_error;
        prev_error = error;
        
        // Calculate PID output
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        
        // Limit output to motor voltage range (-127 to 127)
        return std::clamp(output, -127.0, 127.0);
    }

public:
    LadyBrown(int motor_port, bool reverse = false) : 
        lift_motor(motor_port) {
        // Configure motor settings
        lift_motor.set_gearing(pros::E_MOTOR_GEARSET_36); // 36:1 green cartridge
        lift_motor.set_reversed(reverse);
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        lift_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        lift_motor.tare_position();
    }
    
    void setState(int new_state) {
        if (new_state >= 0 && new_state <= 2) {
            target_state = new_state;
        }
        if (new_state == 0) {
            reset_countdown = 50;
        }
    }
    
    int getState() const {
        return target_state;
    }
    
    // This should be run in a separate task
    void update() {
        int cnt = 0;
        while (true) {
            int current_state = target_state.load();
            if (current_state == 0 && false)  {
                if (reset_countdown > 0) {
                    reset_countdown--;
                    lift_motor.move(-127);  // Move motor down to reset position
                } else {
                    lift_motor.move(0);  // Stop motor in resting state
                    integral = 0;  // Reset integral when resting
                    prev_error = 0;
                }
                lift_motor.tare_position();  // Reset encoder position
            } else {
                double current_pos = lift_motor.get_position();
                double target_pos = STATE_POSITIONS[current_state];
                
                double output = calculatePID(current_pos, target_pos);
                lift_motor.move(output);
            }
            cnt++;
            pros::delay(20);  // Run at 50Hz
        }
    }
    
    // Helper method to start the control loop
    static void startTask(void* param) {
        LadyBrown* instance = static_cast<LadyBrown*>(param);
        instance->update();
    }
    
    void start() {
        pros::Task control_task(startTask, this, "LadyBrown");
    }
};
// Global Vars

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-13, -6, -15});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({16, 19, 18});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::MotorGroup intake({-1, 12});
LadyBrown lady_brown(2);  // Replace 1 with your motor port
int lady_brown_state = 0;

pros::Imu imu_sensor(5);

const int GOAL_CLAMP_PORT = 8;
const int DOINKER_PORT = 7;
const int COLOR_SORT_PORT = 6;
EnhancedDigitalOut mogo_mech (GOAL_CLAMP_PORT, LOW);
EnhancedDigitalOut doinker (DOINKER_PORT, LOW);
EnhancedDigitalOut color_sort (COLOR_SORT_PORT, LOW);
pros::Vision vision_sensor(2);

bool clamp_state = LOW;
bool doinker_state = LOW;
bool color_sort_state = LOW;
float prev_heading;



pros::Rotation side_encoder(10); // Lateral tracking encoder (PORT MIGHT BE WRONG)
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
const double WHEEL_BASE_WIDTH = 12.7;  // Distance between the left and right wheels in inches
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

// Structure to hold color signature information
struct ColorInfo {
    int32_t red_sig;   // Signature ID for red objects
    int32_t blue_sig;  // Signature ID for blue objects
    pros::vision_signature_s_t* red_sig_ptr;
    pros::vision_signature_s_t* blue_sig_ptr;
};

// Function to check color and control piston
void color_piston_control(void* param) {
    const char* target_color = (const char*)param;
    
    // Initialize devices
    
    // Initialize color signatures (you need to configure these using Vision Utility)
    pros::vision_signature_s_t RED_SIG =
        pros::Vision::signature_from_utility(1, 8000, 10000, 9000, -1000, 1000, 0, 3.000, 0);
    pros::vision_signature_s_t BLUE_SIG =
        pros::Vision::signature_from_utility(2, -3000, -1000, -2000, 3000, 5000, 4000, 3.000, 0);
    
    ColorInfo colors = {1, 2, &RED_SIG, &BLUE_SIG};
    
    while (true) {
        // Determine which signature to look for
        int32_t sig_id = (strcmp(target_color, "red") == 0) ? colors.red_sig : colors.blue_sig;
        pros::vision_signature_s_t* sig_ptr = 
            (strcmp(target_color, "red") == 0) ? colors.red_sig_ptr : colors.blue_sig_ptr;
        
        // Configure vision sensor with the appropriate signature
        vision_sensor.set_signature(sig_id, sig_ptr);
        
        // Get the largest object of the specified color
        pros::vision_object_s_t detected_obj = 
            vision_sensor.get_by_sig(0, sig_id);
        
        // Check if object is detected and meets size threshold
        if (detected_obj.signature == sig_id && detected_obj.width > 10) {
            // Wait for 250ms to ensure stable detection
            pros::delay(250);
            
            // Fire piston
            color_sort_state = HIGH;
            color_sort.set_value(color_sort_state);
            
            // Hold for 500ms
            pros::delay(500);
            
            // Retract piston
            color_sort_state = LOW;
            color_sort.set_value(color_sort_state);
        }
        
        // Add a small delay to prevent CPU hogging
        pros::delay(20);
    }
}

class ColorDetectionManager {
private:
    pros::Task* detection_task = nullptr;
    const char* current_color = nullptr;

public:
    // Constructor
    ColorDetectionManager() = default;
    
    // Start detection with specified color
    void start(const char* color) {
        // If task is already running, stop it
        if (detection_task != nullptr) {
            stop();
        }
        
        // Allocate new color parameter
        current_color = color;
        
        // Create new task
        detection_task = new pros::Task(color_piston_control, 
                                      (void*)current_color, 
                                      "Color Detection");
    }
    
    // Stop the detection task
    void stop() {
        if (detection_task != nullptr) {
            detection_task->remove();
            delete detection_task;
            detection_task = nullptr;
        }
    }
    
    // Check if task is running
    bool is_running() {
        return detection_task != nullptr;
    }
    
    // Destructor
    ~ColorDetectionManager() {
        stop();
    }
};


// Function to start the color detection thread
void start_color_detection(const char* color) {
    // Validate color parameter
    if (strcmp(color, "red") != 0 && strcmp(color, "blue") != 0) {
        std::cout << "Invalid color parameter. Use 'red' or 'blue'" << std::endl;
        return;
    }
    
    // Allocate memory for the color parameter
    char* color_param = (char*)malloc(strlen(color) + 1);
    strcpy(color_param, color);
    
    // Create the thread
    pros::Task color_detection_task(color_piston_control, (void*)color_param, "Color Detection");
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
                     int timeout = 1e9) { // Not using timeout for now
    
    if (route.empty()) return false;
    
    const double START_TIME = pros::millis();
    const double DT = 25;  // 25ms fixed timestep matching motion profile
    const double track_width = 12.7; // inches
    size_t trajectory_index = 1;
    Pose new_pos = {route[1][1], route[1][2], route[1][3]};
    odometry.setPose(new_pos);

    
    // Main control loop
    while (trajectory_index < route.size()) {
        pros::lcd::print(0, "Trajectory index: %d", trajectory_index);
        pros::lcd::print(1, "Pose %f %f %f", odometry.getPose().x, odometry.getPose().y, odometry.getPose().theta);
        odometry.update();
        pros::lcd::print(2, "New %f %f %f", odometry.getPose().x, odometry.getPose().y, odometry.getPose().theta);
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
        int next_index = trajectory_index + 1;
        while (next_index < route.size() && route[next_index].size() < 6) {
            next_index++;
        }
        if (next_index < route.size()) {
            const auto& next = route[next_index];
            
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
        
        pros::lcd::print(3, "Goal %f %f %f", goal_x, goal_y, goal_theta);
        // Get RAMSETE controller output
        auto ramsete_output = ramsete.calculate(
            current_pose.x, current_pose.y, current_pose.theta,
            goal_x, goal_y, goal_theta,
            goal_v, goal_w
        );
        pros::lcd::print(4, "Ramsete %f %f", ramsete_output[0], ramsete_output[1]);
        
        // Convert RAMSETE output to wheel velocities
        auto wheel_velocities = ramsete.calculate_wheel_velocities(
            goal_v, // linear velocity
            goal_w, // angular velocity
            2.75,             // wheel diameter (inches)
            48.0/36.0,         // gear ratio
            track_width       // track width (inches)
        );
        pros::lcd::print(5, "Wheel %f %f", wheel_velocities[0], wheel_velocities[1]);
        
        // Calculate motor voltages using drivetrain controller
        auto voltages = drivetrain.calculateVoltages(
            wheel_velocities[0], // left velocity setpoint
            wheel_velocities[1], // right velocity setpoint
            velocities.first,    // current left velocity
            velocities.second,   // current right velocity
            left_accel,         // left acceleration
            right_accel         // right acceleration
        );
        pros::lcd::print(6, "Voltage %d %d", voltages.left, voltages.right);
        
        // Apply voltages to motors
        left_mg.move(voltages.left);
        right_mg.move(voltages.right);

        std::cout << wheel_velocities[0] << " " << velocities.first << "\n";
        
        // Check if we're at the final waypoint
        // if (trajectory_index == route.size() - 1) {
        //     const double position_tolerance = 1.0;  // inches
        //     const double heading_tolerance = 0.1;   // radians
            
        //     const double position_error = std::sqrt(
        //         std::pow(current_pose.x - goal_x, 2) +
        //         std::pow(current_pose.y - goal_y, 2)
        //     );
        //     const double heading_error = std::abs(current_pose.theta - goal_theta);
            
        //     if (position_error < position_tolerance && heading_error < heading_tolerance) {
        //         // Stop motors
        //         left_mg.move(0);
        //         right_mg.move(0);
        //         return true;
        //     }
        // }
        
        // Increment trajectory index based on fixed timestep
        trajectory_index++;
        
        // Wait until next timestep
        // const double elapsed = pros::millis() - START_TIME;
        // const double next_timestep = (trajectory_index + 1) * DT;
        // const double delay_time = next_timestep - elapsed;
        // if (delay_time > 0) {
        // if (trajectory_index == 40){
        //     break;
        // }
        pros::delay(DT);
        // }
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

    std::cin.tie(0);

	left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
	right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    // pinMode(1, OUTPUT);

    lady_brown.start();  // Start the control task

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
        RamseteController ramsete(2.0, 0.7, 5.35*12, 5.0, 0.0254000508);
        DrivetrainController drivetrain(15.25, 0.05, 0, 0, 0, 0);
        // std::vector<double> velos =  drivetrain.calculateVoltages(5, 5, 0, 0, 0, 0);
        // left_mg.move_velocity(); 
        // pros::delay(1000);
        // while (true){
        //     pros::lcd::print(1, "Lateral encoder position: %d", side_encoder.get_position());
        //     pros::delay(20);
        // }
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
    bool prev_lady_brown_state = false;
	while (true) {
		// Arcade control scheme
		int dir = (master.get_analog(ANALOG_LEFT_Y));    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage

        if (master.get_digital(DIGITAL_R1)) {
            intake.move_velocity(200);
        } else if (master.get_digital(DIGITAL_R2)) {
            intake.move_velocity(-200);
        } else {
            intake.move_velocity(0);
        }

        bool old_state = mogo_mech.get_state();
        mogo_mech.input_toggle(master.get_digital(DIGITAL_L1));
        if (!old_state && mogo_mech.get_state()){
            master.rumble("-");
        }

        if (!prev_lady_brown_state && master.get_digital(DIGITAL_L2)){
            lady_brown_state = (lady_brown_state + 1) % 3;
            lady_brown.setState(lady_brown_state);
        }
        prev_lady_brown_state = master.get_digital(DIGITAL_L2);
        color_sort.input_toggle(master.get_digital(DIGITAL_DOWN));

        doinker.input_toggle(master.get_digital(DIGITAL_B));

		pros::delay(20);                               // Run for 20 ms then update
	}
}