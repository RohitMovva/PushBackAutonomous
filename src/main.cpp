#include "main.h"

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -12, -13}); // Creates a motor group with forwards ports 1 & 3 and reversed port 2
pros::MotorGroup right_mg({20, 19, 18});   // Creates a motor group with forwards port 5 and reversed ports 4 & 6
pros::Motor indexer(6);
pros::Motor intake(9);
pros::Motor top_intake(-10);

Trajectory trajectory;

pros::Imu imu_sensor(9);

Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

// Program types
std::string program_type = "autonomous";

// Routes
std::vector<std::vector<double>> route;
std::string route_name = "test2"; // used to be skills

// Robot parameters (needs to be tweaked later)

Odometry odometry;
RamseteController ramsete_controller;
DrivetrainController drive_controller;

Robot robot;


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

    Odometry odometry(left_mg, right_mg, side_encoder, imu_sensor, false, true);
    
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
    pros::lcd::print(0, "Program type: %s", program_type.c_str());
    while (true)
    {
        int dir = (master.get_analog(ANALOG_LEFT_Y));                // Gets amount forward/backward from left joystick
        int turn = joystickCurve(master.get_analog(ANALOG_RIGHT_X)); // Gets the turn left/right from right joystick
        pros::lcd::print(1, "Left: %d, Right: %d", dir + turn, dir - turn);
        left_mg.move(dir + turn);                                    // Sets left motor voltage
        right_mg.move(dir - turn);                                   // Sets right motor voltage
        
        if (master.get_digital(DIGITAL_R1)) // If R1 is pressed
        {
            intake.move_velocity(12000); // Run intake at full speed
            top_intake.move_velocity(-12000);
            indexer.move_velocity(12000); // Run indexer at full speed
        }
        else if (master.get_digital(DIGITAL_R2)) // If R2 is pressed
        {
            intake.move_velocity(12000); // Run intake in reverse at full speed
            indexer.move_voltage(-12000); // Run indexer in reverse at full speed
            top_intake.move_voltage(12000); // Run top intake at full speed
        }
        else if (master.get_digital(DIGITAL_L1)) // If L1 is pressed
        {
            intake.move_voltage(-12000); // Run intake at full speed
            top_intake.move_voltage(12000);
            indexer.move_voltage(-12000); // Run indexer at full speed
        }
        else if (master.get_digital(DIGITAL_L2)) // If L2 is pressed
        {
            intake.move_voltage(-12000); // Run intake in reverse at full speed
            indexer.move_voltage(-12000); // Run indexer in reverse at full speed
            top_intake.move_voltage(12000); // Run top intake at full speed
        }
        else
        {
            intake.move_voltage(0); // Stop intake
            top_intake.move_voltage(0); // Stop top intake
            indexer.move_voltage(0); // Stop indexer
        }

        // if (master.get_digital(DIGITAL_L1)) // If L1 is pressed
        // {
        //     indexer.move_velocity(127); // Run indexer at full speed
        // }
        // else if (master.get_digital(DIGITAL_L2)) // If L2 is pressed
        // {
        //     indexer.move_velocity(-127); // Run indexer in reverse at full speed
        // }
        // else
        // {
        //     indexer.move_velocity(0); // Stop indexer
        // }

        pros::delay(Config::DT);
    }
}
