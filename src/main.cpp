#include "main.h"
#include "routes/routes.h"
#include "utilities/logger.h"
#include "pros/rotation.hpp"
#include "filters/slew_rate_limiter.h"

// TODO test straight line
// TODO test right curve and left curves (and compare them)

// TODO test odometry by driving
// TODO test with and without RAMSETE and compare performance


class Vector2 {
public:
  Vector2(float x, float y): x(x), y(y) {}
  std::string latex() const {
    std::ostringstream oss;
    oss << "(" << std::fixed << this->x << "," << std::fixed << this->y << ")";
    return oss.str();
  }
  
  float x;
  float y;
};



class LadyBrown {
private:
    std::atomic<int> target_state{0};
    
    // PID constants - these may need tuning
    const double kP = 1.5;
    const double kI = 0.07;
    const double kD = 0.0;
    const double kG = 1.0;

    pros::Rotation rot_sensor;

    bool manual_control = false;
    
    // Position setpoints in degrees (adjusted for gear ratio)
    // With 12:64 ratio, multiply desired angles by (64/12) = 5.33
    double STATE_POSITIONS[5] = {
        0.0,
        -34.0 + 9.00,    // ~53.3 degrees at motor
        -200.5,    // ~586.7 degrees at motor
        -160.0 + 10,    // Descore position
        0.0, // Placeholder for manual control
    };
    
    // PID variables
    double integral = 0;
    double prev_error = 0;
    
    // Helper functon to calculate PID
    double calculatePID(double current_pos, double target_pos) {
        double error = target_pos - current_pos;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double cp = current_pos;
        
        // while (cp > 180) cp -= 360;
        // while (cp < -180) cp += 360;
        pros::lcd::print(2, "Current pos: %f", cp);

        if (cp > -270 && cp < -180 && std::abs(error) > 50.0){
            error += 360.0;
        }
        
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
    pros::Motor lift_motor;

    LadyBrown(int motor_port, bool reverse, pros::Rotation sensor) : 
        lift_motor(motor_port),
        rot_sensor(sensor) {
        // Configure motor settings
        lift_motor.set_gearing(pros::E_MOTOR_GEARSET_36); // 36:1 green cartridge
        lift_motor.set_reversed(reverse);
        lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        lift_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        lift_motor.tare_position();
        rot_sensor.reset();



    }
    
    void setState(int new_state) {
        // if (new_state >= 0 && new_state <= 3) {
        target_state = new_state;
        // }
    }
    
    int getState() const {
        return target_state;
    }
    
    // This should be run in a separate task
    void update() {
        while (true) {
            // pros::lcd::print(0, "LadyBrown Task: %d", manual_control);
            if (manual_control) {
                // Skip PID control if in manual mode
                pros::delay(20);
                continue;
            }
            int current_state = target_state.load();
            // pros::lcd::print(1, "Current state: %d", current_state);
            double current_pos = rot_sensor.get_position() / 100.0;
            double target_pos = STATE_POSITIONS[current_state];
            // pros::lcd::print(2, "Current pos: %f", current_pos);
            // pros::lcd::print(3, "Target pos: %f", target_pos);

            double output = calculatePID(current_pos, target_pos);
            lift_motor.move(output);
            pros::delay(20);  // Run at 50Hz
        }
    }

    pros::Motor getMotor() {
        return lift_motor;
    }

    void setManualControl(bool manual) {
        if (manual != manual_control) {
            lift_motor.move_velocity(0);
            setManualPosition();
        }
        manual_control = manual;
    }

    void setManualPosition() {
        STATE_POSITIONS[4] = rot_sensor.get_position() / 100.0;
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
pros::Rotation lb_encoder(6); // Later/al tracking encoder (PORT MIGHT BE WRONG)

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-15, -10, -18});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({19, 16, 11});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
// Set to blue cartridges
// left_mg.set_gearing(pros::E_MOTOR_GEARSET_36);
pros::Motor lower_intake(-13);
pros::Motor upper_intake(4);
pros::MotorGroup intake({-13, 4});
LadyBrown lady_brown(17, true, lb_encoder);  // Replace 1 with your motor port
int lady_brown_state = 0;
// right arrow is 0
// down is activated
pros::Imu imu_sensor(9);

const int GOAL_CLAMP_PORT = 1;
const int DOINKER_PORT = 2;
const int INTAKE_LIFT_PORT = 6;
const int OPTICAL_SENSOR_PORT = 1;
// const int OPTICAL = 5;

EnhancedDigitalOut mogo_mech (GOAL_CLAMP_PORT, LOW);
EnhancedDigitalOut doinker (DOINKER_PORT, LOW);
EnhancedDigitalOut intake_lift (INTAKE_LIFT_PORT, LOW);
// pros::Optical optical_sensor(OPTICAL_SENSOR_PORT);
// EnhancedDigitalOut color_sort (COLOR_SORT_PORT, LOW);
pros::Vision vision_sensor(2);

bool clamp_state = LOW;
bool doinker_state = LOW;
bool color_sort_state = LOW;
bool intake_lift_state = LOW;
float prev_heading;


Logger* logger = Logger::getInstance();


pros::Rotation side_encoder(5); // Lateral tracking encoder (PORT MIGHT BE WRONG)
// Program types
// std::string program_type = "driver";
// std::string program_type = "autonomous";
std::string program_type = "autonomous";
// std::string program_type = "turn_test";
// std::string program_type = "calibrate_metrics";

// Routes
std::vector<std::vector<double>> route = skills; // used to be skills
// std::vector<std::vector<double>> route = {}; // Driver or Calibration

// Robot parameters (needs to be tweaked later)
const float WHEEL_DIAMETER = 2.75;  // Diameter of the wheels in inches
const float TICKS_PER_ROTATION = 300.0;  // Encoder ticks per wheel rotation for blue cartridges
const float GEAR_RATIO = 36.0/48.0;  // Gear ratio of the drivetrain
const double WHEEL_BASE_WIDTH = 12.7;  // Distance between the left and right wheels in inches
const float DT = 0.025;  // Time step in seconds (25 ms)

// PID Code
// PID parameters (need to be tuned, especially heading)
// float kp_position = 1.6;
// float ki_position = 0.0;
// float kd_position = 0.075;
float kp_position = 1.2;
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

double velocityToTicks(double velocityInchesPerSec) {
    return (velocityInchesPerSec * TICKS_PER_ROTATION) / (WHEEL_DIAMETER * M_PI);
}

double ticksToInches(double ticks) {
    return (ticks / TICKS_PER_ROTATION) * WHEEL_DIAMETER * M_PI * GEAR_RATIO;
}



class PIDController {
public:
    PIDController(float kp, float ki, float kd, float intergral_max=5)
        : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0), intergral_max(intergral_max) {}

    float compute(float setpoint, float current_value, float dt, bool headingCentered=false) {
        float error = setpoint - current_value;
        if (headingCentered){

            if (error < -180.0){
                error += 360.0;
            }
            if (error > 180.0){
                error -= 360.0;
            }
        }
        if (current_value < 180.0){
            error -= 360.0;
        }
        // if (error > )
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

// void color_motor_control(const char* target_color, ColorDetectionManager* manager);
class ColorDetectionManager;

// Move the function declaration before the class
void color_motor_control(const char* target_color, ColorDetectionManager* manager);

// Modified struct to remove vision sensor specific elements
struct ColorInfo {
    int32_t red_hue_min;    // Minimum hue value for red detection
    int32_t red_hue_max;    // Maximum hue value for red detection
    int32_t blue_hue_min;   // Minimum hue value for blue detection
    int32_t blue_hue_max;   // Maximum hue value for blue detection
    double_t saturation_threshold; // Minimum saturation for valid color detection
};

class ColorDetectionManager {
    private:
        pros::Task* detection_task = nullptr;
        const char* current_color = nullptr;
        std::atomic<bool> color_sorting_{false}; // Use atomic for thread safety
    
    public:
        // Add getter for color_sorting state
        bool color_sorting() const {
            return color_sorting_.load();
        }

        void set_color_sorting(bool state) {
            color_sorting_.store(state);
        }
        
        ColorDetectionManager() = default;
        
        void start(const char* color) {
            if (detection_task != nullptr) {
                stop();
            }
            
            current_color = color;
            
            // Create a struct to pass both color and class pointer to task
            struct TaskParams {
                const char* color;
                ColorDetectionManager* manager;
            };
            
            auto* params = new TaskParams{color, this};
            
            // Modified task creation to use new parameter struct
            detection_task = new pros::Task(
                [](void* param) {
                    auto* params = static_cast<TaskParams*>(param);
                    color_motor_control(params->color, params->manager);
                    delete params; // Clean up parameters when task ends
                },
                params,
                "Color Detection"
            );
        }
        
        void stop() {
            if (detection_task != nullptr) {
                detection_task->remove();
                delete detection_task;
                detection_task = nullptr;
                color_sorting_ = false; // Reset sorting state when stopped
            }
        }
        
        bool is_running() {
            return detection_task != nullptr;
        }
        
        ~ColorDetectionManager() {
            stop();
        }
    };
    
// Modified control function to update color_sorting state
void color_motor_control(const char* target_color, ColorDetectionManager* manager) {
    // Initialize devices
    pros::Optical optical_sensor(OPTICAL_SENSOR_PORT);
    
    ColorInfo colors = {
        355, 25,   // Red hue range
        180, 240,  // Blue hue range
        0.5         // Minimum saturation threshold
    };

    int i = 0;
    
    while (true) {
        i++;
        optical_sensor.set_integration_time(20.0);
        optical_sensor.set_led_pwm(100);
        // pros::lcd::print(5, "Integration time: %f", optical_sensor.get_integration_time());
        // pros::lcd::print(6, "LED PWM: %d", optical_sensor.get_led_pwm());
        
        double hue = optical_sensor.get_hue();
        // pros::lcd::print(0, "Hue: %f", hue);
        double saturation = optical_sensor.get_saturation();
        // pros::lcd::print(1, "Saturation: %f", saturation);
        // pros::lcd::print(2, "i: %d", i);
        
        bool color_detected = false;
        
        if (saturation > colors.saturation_threshold) {
            if (strcmp(target_color, "red") == 0) {
                if (hue >= colors.red_hue_min || hue <= colors.red_hue_max) {
                    color_detected = true;
                }
            } else {
                if (hue >= colors.blue_hue_min && hue <= colors.blue_hue_max) {
                    color_detected = true;
                }
            }
        }
        // pros::lcd::print(3, "Color detected: %d", color_detected);
        
        if (color_detected) {
            pros::delay(245);
            
            // Use setter method instead of direct access
            manager->set_color_sorting(true);
            
            upper_intake.move_voltage(-1000);
            pros::delay(100);
            
            upper_intake.move_voltage(12000);
            
            // Use setter method instead of direct access
            manager->set_color_sorting(false);
            
        }
        // pros::lcd::print(4, "Color sorting: %d", manager->color_sorting());
        
        pros::delay(15);
    }
}    

ColorDetectionManager color_manager;

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
    logger->log("Following trajectory");
    std::cout << "Following trajectory" << std::endl;
    // pros::delay(2000);
    // left_mg.getveloc
    
    const double START_TIME = pros::millis();
    const double DT = 25;  // 25ms fixed timestep matching motion profile
    // TODO retune track width
    const double track_width = 13.5; // inches

    lady_brown.setState(route[0][6]);
    size_t trajectory_index = 0;
    Pose new_pos = {route[1][2], route[1][3], route[1][4]};
    odometry.setPose(new_pos);

    logger->log("Initial pose: %f %f %f", new_pos.x, new_pos.y, new_pos.theta);
    // Main control loop
    logger->log("Route size: %d", route.size());
    // lady_brown.setState(2);
    // pros::delay(1000);
    // lady_brown.setState(0)
    const auto& snode = route[0];
    if (snode[1] == 1){
        intake.move(127);
    } else if (snode[1] == -1){
        // pros::Motor 
        lower_intake.move(127);
        upper_intake.move(0);
    } else {
        intake.move(0);
    }

    // Toggle clamp state
    if (snode[2]){
        clamp_state = !clamp_state;
        mogo_mech.set_value(clamp_state);
    }

    lady_brown.setState(snode[6]);

    if (snode[3]){
        doinker_state = !doinker_state;
        doinker.set_value(doinker_state);
    }
    
    
    while (trajectory_index < route.size()) {
        pros::lcd::print(0, "Trajectory index: %d", trajectory_index);
        // logger->log("Trajectory index: %d", trajectory_index);
        odometry.update();
        // logger->log("Odom Updated");
        const double current_time = pros::millis() - START_TIME;
        
        // Timeout check
        if (current_time > timeout) {
            return false;
        }
        // logger->log("Current time: %f", current_time);
        
        // Get current state from odometry
        Pose current_pose = odometry.getPose();
        pros::lcd::print(1, "Current pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
        // logger->log("Got pose");
        auto velocities = odometry.getFilteredVelocities();
        // logger->log("Velocities: %f %f", velocities.first, velocities.second);
        
        // Check if the current waypoint is a node instead of a trajectory point
        while (trajectory_index < route.size() && route[trajectory_index][0] == 1) {
            // Handle node
            const auto& node = route[trajectory_index];
            
            // Move intake (value is either -1, 0, or 1)
            if (node[1] == 1){
                intake.move(127);
            } else if (node[1] == -1){
                // pros::Motor 
                lower_intake.move(127);
                upper_intake.move(0);
            } else {
                intake.move(0);
            }

            // Toggle clamp state
            if (node[2]){
                clamp_state = !clamp_state;
                mogo_mech.set_value(clamp_state);
            }

            lady_brown.setState(node[6]);

            if (node[3]){
                doinker_state = !doinker_state;
                doinker.set_value(doinker_state);
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
        const double goal_x = waypoint[2];
        const double goal_y = waypoint[3];
        const double goal_theta = waypoint[4];
        const double goal_v = waypoint[5];
        const double goal_w = waypoint[6];


        logger->log("Pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
        logger->log("Goal: %f %f %f", goal_x, goal_y, goal_theta);

        // Calculate accelerations using next waypoint
        double left_accel = 0.0;
        double right_accel = 0.0;
        int next_index = trajectory_index + 1;
        while (next_index < route.size() && route[next_index][0] == 1) {
            next_index++;
        }
        if (next_index < route.size()) {
            const auto& next = route[next_index];
            
            // Calculate wheel velocities at current and next timestep
            const double current_left = goal_v - (goal_w * track_width / 2.0);
            const double current_right = goal_v + (goal_w * track_width / 2.0);
            
            const double next_v = next[5];
            const double next_w = next[6];

            const double next_left = next_v - (next_w * track_width / 2.0);
            const double next_right = next_v + (next_w * track_width / 2.0);
            
            // Calculate acceleration over the fixed timestep
            left_accel = (next_left - current_left) / (DT / 1000.0);  // Convert ms to seconds
            right_accel = (next_right - current_right) / (DT / 1000.0);

            // Clamp to [-4.5, 4.5] in/s^2
            // left_accel = std::clamp(left_accel, -4.5, 4.5);
            // right_accel = std::clamp(right_accel, -4.5, 4.5);

            // Debugging left acceleration
            logger->log("Velocities: %f %f", goal_v, next_v);
            logger->log("Next left: %f, Current left: %f", next_left, current_left);
            logger->log("Left accel: %f, Right accel: %f", left_accel, right_accel);
            logger->log("Next v: %f, Next w: %f", next_v, next_w);
            // logger->log("Next v: %f, Next w: %f", next_v, next_w);
        }
        
        // if (goal)
        // Get RAMSETE controller output
        auto ramsete_output = ramsete.calculate(
            current_pose.x, current_pose.y, current_pose.theta,
            goal_x, goal_y, goal_theta,
            goal_v, goal_w
        );
        logger->log("Ramsete Output: %f %f", ramsete_output[0], ramsete_output[1]);
        logger->log("MP Output: %f %f", goal_v, goal_w);
        velocities = odometry.getFilteredVelocities();
        
        // Convert RAMSETE output to wheel velocities
        auto wheel_velocities = ramsete.calculate_wheel_velocities(
            ramsete_output[0], // linear velocity
            ramsete_output[1], // angular velocity
            2.75,             // wheel diameter (inches)
            48.0/36.0,         // gear ratio
            track_width       // track width (inches)
        );
        logger->log("Left wheel velocities: %f %f", wheel_velocities[0], odometry.getLeftVelocity().linear);
        logger->log("Right wheel velocities: %f %f", wheel_velocities[1], odometry.getRightVelocity().linear);
        
        // Calculate motor voltages using drivetrain controller
        auto voltages = drivetrain.calculateVoltages(
            wheel_velocities[0], // left velocity setpoint
            wheel_velocities[1], // right velocity setpoint
            odometry.getLeftVelocity().linear,    // current left velocity
            odometry.getRightVelocity().linear,   // current right velocity
            left_accel,         // left acceleration
            right_accel         // right acceleration
        );
        logger->log("Voltages: %d %d", voltages.left, voltages.right);
        
        // Apply voltages to motors
        left_mg.move(voltages.left);
        right_mg.move(voltages.right);
        // logger->log("Wheel velocities: %f %f", wheel_velocities[0], odometry.getLeftVelocity());
        
        // Increment trajectory index based
        trajectory_index++;
        
        pros::delay(DT);
        // }
    }
    
    // Stop motors
    left_mg.move(0);
    right_mg.move(0);
    return true;
}

void collect_velocity_vs_voltage_data() {
    logger->log("Collecting velocity vs voltage data");
    std::vector<float> inputs = {0.f, 10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f, 90.f, 100.f, 110.f, 120.f, 127.f};
    std::vector<float> outputs = {0.f};
    outputs.reserve(inputs.size());

    float direction = 1;
    for (auto & input: inputs) {
            if (input == 0)
                    continue;
            
            // chassis.drive_with_voltage(direction * input, direction * input);
            left_mg.move(direction * input);
            right_mg.move(direction * input);
            
            // Sleep for 1000 ms
            pros::delay(1000);

            int n;
            float v_sum = 0;
            for (n = 0; n < 10; ++n){
                // Get position delta of the motors
                left_mg.tare_position_all();
                right_mg.tare_position_all();
                pros::delay(10);
                std::vector<double> left_delta_velocity_ticks = left_mg.get_position_all();
                std::vector<double> right_delta_velocity_ticks = right_mg.get_position_all();

                // Calculate the average velocity of the motors
                double left_delta_velocity = (left_delta_velocity_ticks[0] + left_delta_velocity_ticks[1] + left_delta_velocity_ticks[2]) / 3.0;
                double right_delta_velocity = (right_delta_velocity_ticks[0] + right_delta_velocity_ticks[1] + right_delta_velocity_ticks[2]) / 3.0;

                // Convert velocities from delta ticks to delta inches
                left_delta_velocity = ticksToInches(left_delta_velocity);
                right_delta_velocity = ticksToInches(right_delta_velocity);

                // Scale velocities by dt
                double left_velocity = left_delta_velocity / 0.01;
                double right_velocity = right_delta_velocity / 0.01;

                v_sum += (left_velocity + right_velocity) / 2.0;

                logger->log("Left velocity: %f, Right velocity: %f", left_velocity, right_velocity);
            }
            outputs.emplace_back(direction * (v_sum / (float) n));
            auto v = input * direction;
            while (fabsf(v) > 0.3) {
                    v *= 0.9;
                    left_mg.move_voltage(v);
                    right_mg.move_voltage(v);
                    pros::delay(10);
            }
            direction = -direction;
    }

    left_mg.move_voltage(0);
    right_mg.move_voltage(0);

    for (int i = 0; i < inputs.size(); ++i) {
            std::cout << Vector2(inputs[i], outputs[i]).latex() << ",";
            logger->log(Vector2(inputs[i], outputs[i]).latex().c_str());
    }
    std::cout << "\b" << std::endl;
    logger->log("Data collection complete");
}

void collect_velocity_vs_constant_voltage_data() {
    logger->log("Collecting velocity vs constant voltage data");
    float input = 58.2062493625;
    SlewRateLimiter slew_limiter(200.0, 200.0);
    std::vector<float> outputs = {0.f};

    float direction = 1;
    for (int i = 0; i < 250; ++i) {
            if (input == 0)
                    continue;
            
            left_mg.move(direction * input);
            right_mg.move(direction * input);
            
            float v_act = 0;

            std::vector<double> left_delta_velocity_ticks = left_mg.get_position_all();
            std::vector<double> right_delta_velocity_ticks = right_mg.get_position_all();

            // Calculate the average velocity of the motors
            double left_delta_velocity = (left_delta_velocity_ticks[0] + left_delta_velocity_ticks[1] + left_delta_velocity_ticks[2]) / 3.0;
            double right_delta_velocity = (right_delta_velocity_ticks[0] + right_delta_velocity_ticks[1] + right_delta_velocity_ticks[2]) / 3.0;

            // Convert velocities from delta ticks to delta inches
            left_delta_velocity = ticksToInches(left_delta_velocity);
            right_delta_velocity = ticksToInches(right_delta_velocity);

            // Scale velocities by dt
            double left_velocity = left_delta_velocity / 0.01;
            double right_velocity = right_delta_velocity / 0.01;

            v_act = (left_velocity + right_velocity) / 2.0;
            v_act = slew_limiter.calculate(v_act, 0.01);

            // outputs.emplace_back(velocityToTicks(direction * (v_act)));
            outputs.emplace_back(direction * (v_act));
            left_mg.tare_position_all();
            right_mg.tare_position_all();
            pros::delay(10);
    }

    left_mg.move_voltage(0);
    right_mg.move_voltage(0);

    for (int i = 0; i < outputs.size(); ++i) {
            std::cout << Vector2(10*i, outputs[i]).latex() << ",";
            logger->log(Vector2((10*i)/1000.0, outputs[i]).latex().c_str());
    }
    std::cout << "\b" << std::endl;
    logger->log("Data collection complete");
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

    color_manager.start("blue");


    lady_brown.start();  // Start the control task

	// Calibrate the inertial sensor
    imu_sensor.reset();

    lb_encoder.reset();

    side_encoder.set_reversed(true);

	// autonomous() // For outside of competition testing purposes
    logger->log("Robot initialized");
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
    logger->log("Program type: %s", program_type.c_str());
	if (program_type == "autonomous"){
        Odometry odometry(left_mg, right_mg, side_encoder, imu_sensor, WHEEL_BASE_WIDTH, 2.419, false, true, false);
        RamseteController ramsete(2.0, 0.7, 4.5*12, 5.0, 0.0254000508);
        // DrivetrainController drivetrain(6.45734320655, 1.67811141649, 0.266620951361, 3.64408225075, 0.00, 0.0); // 
        DrivetrainController drivetrain(2.5, 1.85, 0.3, 9.0, 0.00, 0.0); // 
        logger->log("Starting autonomous");

        // 003
        // std::vector<double> velos =  drivetrain.calculateVoltages(5, 5, 0, 0, 0, 0);
        // left_mg.move_velocity();
        // pros::delay(1000);
        // while (true){
        //     pros::delay(20);
        // }
        // double test_vel = 2.0;
        // left_mg.move_velocity(test_vel);
        // right_mg.move_velocity(test_vel);
        // PID_controller();
        followTrajectory(route, odometry, ramsete, drivetrain, left_mg, right_mg);
        // collect_velocity_vs_voltage_data();
        // collect_velocity_vs_constant_voltage_data();
    }
}

int joystickCurve(int x, double a=2.5){
    return 
    int(((127.0 * std::pow(std::abs(double(x)), std::abs(a)))/(std::pow(127.0, a)))
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
    printf("Starting opcontrol\n");
    lady_brown.setState(4); // Set to manual control
    bool manual_control = false;
    pros::lcd::print(5, "hallo");
    // Odometry odometry(left_mg, right_mg, side_encoder, imu_sensor, WHEEL_BASE_WIDTH, 3.0, false, true, false); // 2.419
    pros::lcd::print(4, "hai");
    Pose new_pos = {0, 0, 0};
    double left_pos = 0;
    double right_pos = 0;

    left_mg.tare_position_all();
    right_mg.tare_position_all();

    
    // odometry.setPose(new_pos);

    pros::Motor lb = lady_brown.getMotor();
	while (true) {
		// Arcade control scheme
        // double hue = optical_sensor.get_hue();

        // pros::lcd::print(0, "Hue: %f", hue);
        pros::lcd::print(3, "Got here");
        // odometry.update();
        pros::lcd::print(4, "odom here");
        // Pose current_pose = odometry.getPose();
        // pros::lcd::print(1, "Current pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
		int dir = (master.get_analog(ANALOG_LEFT_Y));    // Gets amount forward/backward from left joystick
		int turn = joystickCurve(master.get_analog(ANALOG_RIGHT_X));  // Gets the turn left/right from right joystick
		left_mg.move(dir + turn);                      // Sets left motor voltage
		right_mg.move(dir - turn);                     // Sets right motor voltage
        
        if (color_manager.color_sorting()){
            ;
        }
        else if (master.get_digital(DIGITAL_R1)) {
            intake.move_voltage(12000);
        }
        else if (master.get_digital(DIGITAL_R2)) {
            intake.move_voltage(-12000);
        } else if (master.get_digital(DIGITAL_A)) {
            upper_intake.move_voltage(0);
            lower_intake.move_voltage(12000);
        }
        else {
            intake.move_voltage(0);
        }

        bool old_state = mogo_mech.get_state();
        mogo_mech.input_toggle(master.get_digital(DIGITAL_L1));
        if (!old_state && mogo_mech.get_state()){
            master.rumble("-");
        }


        else if (master.get_digital(DIGITAL_L2)){

            lady_brown.setState(4); // Set to manual control
            lb.move(-127);
            lady_brown.setManualPosition();
            lady_brown.setManualControl(true);
        } else if (master.get_digital(DIGITAL_Y)){
            lady_brown.setState(4); // Set to manual control
            lb.move(127);
            lady_brown.setManualPosition();
            lady_brown.setManualControl(true);
        } else {
            // lb.move(0);
            // lady_brown.setManualPosition();
            lady_brown.setManualControl(false);
        }

        if (master.get_digital_new_press(DIGITAL_DOWN)){
            lady_brown.setState(1);
        }
        if (master.get_digital_new_press(DIGITAL_RIGHT)){
            lady_brown.setState(0);
        }

        if (master.get_digital_new_press(DIGITAL_UP)){
            color_manager.stop();
        }

        if (master.get_digital_new_press(DIGITAL_X)){
            lady_brown.setState(3);
        } else if (master.get_digital_new_press(DIGITAL_LEFT)){
            lady_brown.setState(2);
        }

        intake_lift.input_toggle(master.get_digital(DIGITAL_DOWN));

        doinker.input_toggle(master.get_digital(DIGITAL_B));

        std::vector<double> left_positions = left_mg.get_position_all();
        std::vector<double> right_positions = right_mg.get_position_all();

        left_pos = (left_positions[0] + left_positions[1] + left_positions[2]) / 3.0;
        right_pos = (right_positions[0] + right_positions[1] + right_positions[2]) / 3.0;

        // Convert ticks to inches
        left_pos = ticksToInches(left_pos);
        right_pos = ticksToInches(right_pos);

        pros::lcd::print(0, "Left pos: %f, Right pos: %f", left_pos, right_pos);

		pros::delay(20);                               // Run for 20 ms then update
	}
}
