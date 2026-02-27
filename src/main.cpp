#include "main.h"

// Robot config
bool skills = false;
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11, -12, -13});
pros::MotorGroup right_mg({18, 19, 20});
pros::Motor lower_intake_m(4);
pros::Motor middle_intake_m(-14);
pros::Motor upper_intake_m(1);
pros::Optical color_sort_os(21);
// pros::Distance park_dist(10);
pros::Distance left_distance_sensor(3);
pros::Distance right_distance_sensor(5);
pros::Distance front_distance_sensor(16);

DistanceResetOdometry::DistanceSensor left_sensor(&left_distance_sensor, 5.9272, 0);
DistanceResetOdometry::DistanceSensor right_sensor(&right_distance_sensor, 6.3209, 0);
DistanceResetOdometry::DistanceSensor front_sensor(&front_distance_sensor, 2.8, -4.5);

EnhancedDigitalOut stopper(8, false);
EnhancedDigitalOut little_will(7, false);
EnhancedDigitalOut intake_lift(6, false);
EnhancedDigitalOut ear2(5, false);
EnhancedDigitalOut rake(4, false);
Intake intake(lower_intake_m, middle_intake_m, upper_intake_m, stopper, intake_lift, color_sort_os, skills);

Trajectory trajectory;
Trajectory goal_to_matchloader;
Trajectory low_goal;

pros::Imu imu_sensor(15);


Logger *logger = Logger::getInstance();

pros::Rotation side_encoder(5);

std::string program_type = "autonomous";



// Routes
std::vector<std::vector<double>> route;
// std::string route_name = "";
std::string route_name = "low_7ball";
// std::vector< std::string > routes = std::vector< std::string >{"skills8.1", "skills8.2"};
std::vector< std::string > routes;
std::vector< Trajectory > trajectories;

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

    if (!skills){
        ;
        intake.start_color_sorting("red");
    }

    imu_sensor.reset();

    side_encoder.set_reversed(true);

    DistanceResetOdometry::DistanceSensors distance_sensors = {front_sensor, nullptr, left_sensor, right_sensor};

    auto localization = LocalizationManager::create(
        LocalizationType::DISTANCE_RESET_ODOMETRY,
        left_mg, right_mg, side_encoder, imu_sensor, distance_sensors,
        false, true
    );
    
    ramsete_controller = new RamseteController(2.2, 0.7, 4.5 * 12, 5.0, 0.0254000508);
    drive_controller = new DrivetrainController(2.5, 1.45, 0.275, 3.0, 0.00, 0.0);

    robot = new Robot(&left_mg, &right_mg, &imu_sensor, &master, drive_controller, ramsete_controller, std::move(localization), 
                      &little_will, &rake, &intake);

    pros::delay(5);
    trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");

    if (routes.size() > 0){
        for (auto& route: routes){
            Trajectory traj;
            traj.loadFromFile("/usd/routes/" + route + ".txt");
            trajectories.push_back(traj);
        }
    }

    goal_to_matchloader.loadFromFile("/usd/routes/goal_to_matchloader.txt");
    low_goal.loadFromFile("/usd/routes/low_goal.txt");
    // trajectory.loadFromVector();
    // if (trajectory.empty())
    // {
    //     logger->log("Error: Route file is empty or could not be opened.");
    //     return;
    // }


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


void testDistanceReset()
{
    double sensorReadingMillimeters = front_distance_sensor.get();
    if (sensorReadingMillimeters >= 9999) {
        Logger::getInstance()->logWarning("Invalid right sensor reading: %f", sensorReadingMillimeters);
        return;
    }

    double sensorReadingInches = sensorReadingMillimeters / 25.4;
    pros::lcd::print(0, "sensorReadingInches: %.2f", sensorReadingInches);
    // Logger::getInstance()->log("Right sensor reading: %f in", sensorReadingInches);

    // Get current pose and heading
    double robotHeading = imu_sensor.get_heading() * (M_PI / 180.0); // Convert to radians
    pros::lcd::print(1, "Heading: %.2f", robotHeading * (180.0 / M_PI));

    double directionOffset = 0.0; // 90 bc left

    int headingDeg = (int)(robotHeading * (180.0 / M_PI) + directionOffset);

    // headingDeg = headingDeg % 360;

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

    if (resettingX && int(directionOffset)%180 == 90) {
        robotHeading += M_PI / 2;
    } else if (!resettingX && int(directionOffset)%180 == 0){
        robotHeading += M_PI / 2;
    }

    if (robotHeading > M_PI){
        robotHeading -= 2*M_PI;
    }
    pros::lcd::print(5, "Heading: %.2f", robotHeading);

    double sensorToWall = std::cos(robotHeading) * sensorReadingInches;
    pros::lcd::print(4, "Sesnor to wall: %.2f", sensorToWall);

    double sensorToRobot = front_sensor.offsetX * std::cos(robotHeading) + front_sensor.offsetY * std::sin(robotHeading);

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
    // trajectory.loadFromFile("/usd/routes/" + route_name + ".txt");
    ear2.input_toggle(true);

    if (program_type == "autonomous")
    {
        // intake.set_state(1);
        // intake.update();
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // intake.update();
        // pros::delay(500);
        // left_mg.move_voltage(-3000);
        // right_mg.move_voltage(-3000);
        // pros::delay(200);
        // left_mg.move_voltage(0);
        // right_mg.move_voltage(0);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(3000*i);
        //     right_mg.move_voltage(3000*i*-1);
        //     pros::delay(250);
        //     if (i==1)pros::delay(150);
        // }
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(3000*i);
        //     right_mg.move_voltage(3000*i*-1);
        //     pros::delay(250);
        //     if (i==1)pros::delay(100);
        // }
        // left_mg.move_voltage(3000);
        // right_mg.move_voltage(3000);
        // pros::delay(500);
        // left_mg.move_voltage(-5000);
        // right_mg.move_voltage(-5000);
        // pros::delay(1000);

        // left_mg.move_voltage(2000);
        // right_mg.move_voltage(2000);
        // pros::delay(1000);

        // left_mg.move_voltage(0);
        // right_mg.move_voltage(0);
        // left_mg.move_voltage(8000);
        // right_mg.move_voltage(8000);
        // pros::delay(500);
        // for (int i = -1; i < 2; i +=2){
        //     left_mg.move_voltage(5000*i);
        //     right_mg.move_voltage(5000*i*-1);
        //     pros::delay(250);
        // }
        logger->log("Starting autonomous");

        if (trajectories.size() == 0){
            robot->followTrajectory(trajectory);
        } else {
            robot->followTrajectories(trajectories);
        }
        // robot->tuneTrackWidth();
    }
    // left_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
    // right_mg.set_brake_mode_all(MOTOR_BRAKE_BRAKE);
}

int joystickCurve(int x, double a = 3.0)
{
    return int(((127.0 * std::pow(std::abs(double(x)), std::abs(a))) / (std::pow(127.0, a))) * (double(x) / std::abs(double(x))));
}

int joystickCurveSkills(int x, double a = 3.0)
{
    return int(((std::pow((double(x) / 5.7), 2.0) -std::abs(std::pow((double(x) / 5.7), 3.0) / 6.0) +(std::pow((double(x) / 5.7), 4.0)/120.0) )*double(x)) / (3.0 * std::abs(double(x))));
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
    // ear.input_toggle(true);
    ear2.input_toggle(true);
    double drivetrain_slow = 0.5;
    left_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    right_mg.set_brake_mode_all(MOTOR_BRAKE_COAST);
    while (true)
    {
        // ------------- START SKILLS DRIVER CONTROLS --------------------
        if (skills) {
            int dir = (master.get_analog(ANALOG_LEFT_Y));
            int turn = joystickCurveSkills(master.get_analog(ANALOG_RIGHT_X));
            left_mg.move((dir + turn) * drivetrain_slow);
            right_mg.move((dir - turn) * drivetrain_slow);

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
                intake.set_state(15);
                intake.state_decay(15, 16, 600);
            }
            else if (master.get_digital(DIGITAL_L1)) // Outake to high goal
            {
                intake.set_state(4);
            }
            // else if (master.get_digital(DIGITAL_UP)){
            //     intake.set_state(7);
            // }
            else
            {
                intake.set_state(0); // Stop intake if no buttons are pressed
            }

            // if (master.get_digital(DIGITAL_A)){
            //     intake.stop_color_sorting();
            // }

            if (master.get_digital(DIGITAL_RIGHT)){
                intake.set_intake_speed(6000);
                drivetrain_slow = 0.5;
            }
            else {
                intake.set_intake_speed(120000);
                drivetrain_slow = 1.0;
            }

            if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_L1)){
                robot->followTrajectory(goal_to_matchloader);
            } else if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_R1)){
                robot->followTrajectory(low_goal);
            }

            intake.set_shift(master.get_digital(DIGITAL_Y));

            intake.update();

            little_will.input_toggle(master.get_digital(DIGITAL_DOWN));

            ear2.input_toggle(master.get_digital(DIGITAL_RIGHT));



            pros::delay(Config::DT);
            // ------------- END SKILLS DRIVER CONTROLS --------------------
        } else {
            // ------------- START MATCH DRIVER CONTROLS --------------------

            int dir = (master.get_analog(ANALOG_LEFT_Y));
            int turn = joystickCurveSkills(master.get_analog(ANALOG_RIGHT_X));
            left_mg.move((dir + turn) * drivetrain_slow);
            right_mg.move((dir - turn) * drivetrain_slow);

            if (master.get_digital(DIGITAL_R1)) // Intake but hold
            {
                intake.set_state(1);
            }
            else if (master.get_digital(DIGITAL_R2)) // Outake to bottom goal
            {
                intake.set_state(2);
            }
            else if (master.get_digital(DIGITAL_DOWN)) // Outake into middle goal
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

            if (master.get_digital(DIGITAL_A)){
                intake.stop_color_sorting();
            }

            if (master.get_digital(DIGITAL_RIGHT)){
                intake.set_intake_speed(6000);
                drivetrain_slow = 0.5;
            }
            else {
                intake.set_intake_speed(120000);
                drivetrain_slow = 1.0;
            }

            intake.set_shift(master.get_digital(DIGITAL_Y));

            intake.update();

            little_will.input_toggle(master.get_digital(DIGITAL_RIGHT));

            intake_lift.set_state(master.get_digital(DIGITAL_B));
            ear2.set_state(!master.get_digital(DIGITAL_L2));

            pros::delay(Config::DT);
        }
        // ------------- END MATCH DRIVER CONTROLS --------------------
    }
}