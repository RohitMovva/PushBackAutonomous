#include "main.h"

/**
 * @brief Structure to hold distance sensor and its offset
 */
// struct DistanceSensor
// {
//     pros::Distance* sensor;
//     double offsetX;
//     double offsetY;

//     DistanceSensor(pros::Distance* s = nullptr, double oX = 0.0, double oY = 0.0) 
//         : sensor(s), offsetX(oX), offsetY(oY) {}
// };

// Robot config
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -13, -15});
pros::MotorGroup right_mg({19, 18, 17});
pros::Motor lower_intake_m(-2);
pros::Motor middle_intake_m(6);
pros::Motor upper_intake_m(-3);
pros::Optical optical_sensor(21);
pros::Distance left_distance_sensor(8);

DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, 6.9, -2.75);


EnhancedDigitalOut stopper_ear(8, false);
EnhancedDigitalOut little_will(7, false);
EnhancedDigitalOut ear(6, false);
Intake intake(lower_intake_m, middle_intake_m, upper_intake_m, stopper_ear, ear, optical_sensor);

Trajectory trajectory;

pros::Imu imu_sensor(5);

Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

// Program types
std::string program_type = "autonomous";

// Routes
std::vector<std::vector<double>> route;
std::string route_name = "straight_line"; 

RamseteController* ramsete_controller;
DrivetrainController* drive_controller;

Robot* robot;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    // pros::lcd::clear()

    left_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);
    right_mg.set_encoder_units_all(pros::E_MOTOR_ENCODER_COUNTS);

    left_mg.tare_position_all();
    right_mg.tare_position_all();

    intake.start_color_sorting("blue");

    // Calibrate the inertial sensor
    imu_sensor.reset();

    side_encoder.set_reversed(true);

    DistanceResetOdometry::DistanceSensors distance_sensors = {nullptr, nullptr, left_sensor, nullptr};

    // Create localization manager with odometry
    auto localization = LocalizationManager::create(
        LocalizationType::DISTANCE_RESET_ODOMETRY,
        left_mg, right_mg, side_encoder, imu_sensor, distance_sensors,
        false, true
    );
    // logger->log("Localization init: %s",  ? "Success" : "Failed");
    
    ramsete_controller = new RamseteController(2.0, 0.7, 4.5 * 12, 5.0, 0.0254000508);
    drive_controller = new DrivetrainController(2.5, 1.7, 0.3, 3.0, 0.00, 0.0);

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, drive_controller, ramsete_controller, std::move(localization), 
                      &little_will, &intake);

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

        robot->followTrajectory(trajectory);
        // robot->tuneTrackWidth();
    }
}

int joystickCurve(int x, double a = 2.5)
{
    return int(((127.0 * std::pow(std::abs(double(x)), std::abs(a))) / (std::pow(127.0, a))) * (double(x) / std::abs(double(x))));
}



void testDistanceReset()
{
    double sensorReadingMillimeters = left_distance_sensor.get();
    if (sensorReadingMillimeters >= 9999) {
        Logger::getInstance()->logWarning("Invalid left sensor reading: %f", sensorReadingMillimeters);
        return;
    }

    double sensorReadingInches = sensorReadingMillimeters / 25.4;
    Logger::getInstance()->log("Left sensor reading: %f in", sensorReadingInches);

    // Get current pose and heading
    double robotHeading = imu_sensor.get_heading() * (M_PI / 180.0); // Convert to radians
    pros::lcd::print(1, "Heading: %.2f", robotHeading * (180.0 / M_PI));

    double directionOffset = 90.0; // 90 bc left

    int headingDeg = (int)(robotHeading * (180.0 / M_PI) + directionOffset);

    headingDeg = headingDeg % 360;

    bool resettingX = false;
    double wallSign = 1.0;

    if (315 <= headingDeg || headingDeg <= 45) // Right wall 
    {
        resettingX = true;
        wallSign = 1.0;
    }
    else if (45 <= headingDeg && headingDeg < 135) // Top wall
    {
        resettingX = false;
        wallSign = -1.0;
    }
    else if (135 <= headingDeg && headingDeg < 225) // Left wall
    {
        resettingX = true;
        wallSign = -1.0;
    }
    else { // Bottom wall
        resettingX = false;
        wallSign = 1.0;
    }


    if (resettingX) {
        robotHeading += M_PI / 2;
    }

    double sensorToWall = std::cos(robotHeading) * sensorReadingInches;

    double sensorToRobot = left_sensor.offsetX * std::cos(robotHeading) + left_sensor.offsetY * std::sin(robotHeading);

    double robotToWall = sensorToWall + sensorToRobot;

    double actualPos = wallSign * (70.7 - robotToWall);

    pros::lcd::print(3, "Distance From Wall: %.2f", robotToWall);

    if (resettingX){
        pros::lcd::print(2, "Current X: %.2f", actualPos);
    }
    else {
        pros::lcd::print(2, "Current Y: %.2f", actualPos);
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
void opcontrol()
{
    pros::lcd::print(0, "Program type: %s", program_type.c_str());
    while (true)
    {
        int dir = (master.get_analog(ANALOG_LEFT_Y));                // Gets amount forward/backward from left joystick
        int turn = joystickCurve(master.get_analog(ANALOG_RIGHT_X)); // Gets the turn left/right from right joystick
        left_mg.move(dir + turn);                                    // Sets left motor voltage
        right_mg.move(dir - turn);                                   // Sets right motor voltage
        
        if (master.get_digital(DIGITAL_R1)) // Intake but hold
        {
            intake.set_state(1);
        }
        else if (master.get_digital(DIGITAL_R2)) // Outake to bottom goal
        {
            intake.set_state(2);
        }
        else if (master.get_digital(DIGITAL_L2)) // Outake into middle goal
        {
            intake.set_state(3);
        }
        else if (master.get_digital(DIGITAL_L1)) // Outake to high goal
        {
            intake.set_state(4);
        }
        else
        {
            intake.set_state(0); // Stop intake if no buttons are pressed
        }

        little_will.input_toggle(master.get_digital(DIGITAL_DOWN));

        stopper_ear.input_toggle(master.get_digital(DIGITAL_B));
        ear.input_toggle(master.get_digital(DIGITAL_B));

        std::cout << "test" << std::endl;

        pros::delay(Config::DT);

        // testDistanceReset();
    }

}