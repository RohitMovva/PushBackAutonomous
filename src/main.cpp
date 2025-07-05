#include "main.h"

// Global Vars

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-15, -10, -18}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({19, 16, 11});   // Creates a motor group with forwards port 5 and reversed ports 4 & 6

Odometry odometry;
DrivetrainController drive_controller;
RamseteController ramsete_controller;
Robot robot;
Trajectory trajectory;

pros::Imu imu_sensor(9);

Logger *logger = Logger::getInstance();

pros::Rotation side_encoder;

// Program types
std::string program_type = "autonomous";

// Routes
std::vector<std::vector<double>> route;
std::string route_name = "test2"; // used to be skills

// Robot parameters (needs to be tweaked later)
const float WHEEL_DIAMETER = 2.75;             // Diameter of the wheels in inches
const float TICKS_PER_ROTATION = 300.0;        // Encoder ticks per wheel rotation for blue cartridges
const float GEAR_RATIO = 36.0 / 48.0;          // Gear ratio of the drivetrain
const double WHEEL_BASE_WIDTH = 14.1966209238; // Distance between the left and right wheels in inches
const float DT = 0.01;                         // 10ms in seconds

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{

    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    // Calibrate the inertial sensor
    imu_sensor.reset();

    side_encoder.set_reversed(true);

    Odometry odometry(left_mg, right_mg, side_encoder, imu_sensor, WHEEL_BASE_WIDTH, 2.419, false, true, false);
    RamseteController ramsete_controller(2.0, 0.7, 4.5 * 12, 5.0, 0.0254000508);
    DrivetrainController drive_controller(2.5, 1.85, 0.3, 9.0, 0.00, 0.0);

    Robot robot(&left_mg, &right_mg, &imu_sensor, &drive_controller, &ramsete_controller, &odometry);

    Trajectory trajectory;
    trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");
    if (trajectory.empty())
    {
        logger->log("Error: Route file is empty or could not be opened.");
        return;
    }

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
void competition_initialize()
{
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
void autonomous()
{
    logger->log("Program type: %s", program_type.c_str());

    if (program_type == "autonomous")
    {
        logger->log("Starting autonomous");

        robot.followTrajectory(trajectory);
    }
}

int joystickCurve(int x, double a = 2.5)
{
    return int(((127.0 * std::pow(std::abs(double(x)), std::abs(a))) / (std::pow(127.0, a))) * (double(x) / std::abs(double(x))));
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
void opcontrol()
{
    while (true)
    {
        int dir = (master.get_analog(ANALOG_LEFT_Y));                // Gets amount forward/backward from left joystick
        int turn = joystickCurve(master.get_analog(ANALOG_RIGHT_X)); // Gets the turn left/right from right joystick
        left_mg.move(dir + turn);                                    // Sets left motor voltage
        right_mg.move(dir - turn);                                   // Sets right motor voltage

        pros::delay(10); // Run for 10 ms then update
    }
}
