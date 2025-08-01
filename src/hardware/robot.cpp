#include "hardware/robot.hpp"
#include "controllers/pid_controller.hpp"
#include "utilities/math/units.hpp"
#include "utilities/math/angle.hpp"
#include <stdexcept>

Robot::Robot()
    : m_leftDrivetrain(nullptr), m_rightDrivetrain(nullptr), m_inertial(nullptr), m_driveController(nullptr), m_ramseteController(nullptr), m_localization(nullptr), m_isFollowingTrajectory(false)
{
    // Default constructor initializes all pointers to null
    // Robot will not be functional until proper initialization
}

Robot::Robot(pros::MotorGroup *leftDrivetrain,
             pros::MotorGroup *rightDrivetrain,
             pros::Imu *inertial,
             DrivetrainController *driveController,
             RamseteController *ramseteController,
             std::unique_ptr<ILocalization> localization)
    : m_leftDrivetrain(leftDrivetrain), m_rightDrivetrain(rightDrivetrain), m_inertial(inertial), m_driveController(driveController), m_ramseteController(ramseteController), m_localization(std::move(localization)), m_isFollowingTrajectory(false)
{
    // Validate all pointer parameters
    if (!leftDrivetrain)
    {
        throw std::invalid_argument("Left drivetrain pointer cannot be null");
    }
    if (!rightDrivetrain)
    {
        throw std::invalid_argument("Right drivetrain pointer cannot be null");
    }
    if (!inertial)
    {
        throw std::invalid_argument("Inertial sensor pointer cannot be null");
    }
    if (!driveController)
    {
        throw std::invalid_argument("Drive controller pointer cannot be null");
    }
    if (!ramseteController)
    {
        throw std::invalid_argument("Ramsete controller pointer cannot be null");
    }
    if (!m_localization)
    {
        throw std::invalid_argument("Localization object cannot be null");
    }
}

Robot::~Robot()
{
    // Stop any active trajectory following before destruction
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    // Note: We don't delete the raw pointers as this class doesn't own them
    // The localization object is automatically cleaned up via unique_ptr
    // The calling code is responsible for managing the lifetime of other objects
}

bool Robot::isFollowingTrajectory() const
{
    return m_isFollowingTrajectory;
}

void Robot::stopTrajectory()
{
    if (!isInitialized())
    {
        return;
    }

    // This flag is checked in the trajectory following logic
    m_isFollowingTrajectory = false;

    // Stop both drivetrain sides
    if (m_leftDrivetrain)
    {
        m_leftDrivetrain->brake();
    }
    if (m_rightDrivetrain)
    {
        m_rightDrivetrain->brake();
    }
}

auto Robot::getCurrentPosition() const -> decltype(m_localization->getPose())
{
    if (!m_localization)
    {
        throw std::runtime_error("Localization system not initialized");
    }

    return m_localization->getPose();
}

void Robot::resetPosition(const Pose &position)
{
    if (!m_localization)
    {
        throw std::runtime_error("Localization system not initialized");
    }

    // Stop any active trajectory following before resetting position
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    m_localization->setPose(position);
}

bool Robot::isInitialized() const
{
    return (m_leftDrivetrain != nullptr &&
            m_rightDrivetrain != nullptr &&
            m_inertial != nullptr &&
            m_driveController != nullptr &&
            m_ramseteController != nullptr &&
            m_localization != nullptr);
}

void Robot::processTrajectory(const TrajectoryPoint &tp)
{
    // Process trajectory
    Pose current_pose = m_localization->getPose();

    Logger::getInstance()->log("Pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
    Logger::getInstance()->log("Goal: %f %f %f", tp.x, tp.y, tp.theta);

    // Calculate accelerations using next waypoint
    double left_accel = tp.linear_accel - (tp.angular_accel * Config::WHEEL_BASE_WIDTH / 2.0);
    double right_accel = tp.linear_accel + (tp.angular_accel * Config::WHEEL_BASE_WIDTH / 2.0);

    // Get RAMSETE controller output
    auto ramsete_output = m_ramseteController->calculate(
        current_pose.x, current_pose.y, current_pose.theta,
        tp.x, tp.y, tp.theta,
        tp.linear_vel, tp.angular_vel);
    Logger::getInstance()->log("Ramsete Output: %f %f", ramsete_output[0], ramsete_output[1]);
    Logger::getInstance()->log("MP Output: %f %f", tp.linear_vel, tp.angular_vel);

    // Convert RAMSETE output to wheel velocities using configuration
    auto wheel_velocities = m_ramseteController->calculate_wheel_velocities(
        ramsete_output[0],              // linear velocity
        ramsete_output[1],              // angular velocity
        Config::WHEEL_DIAMETER,         // wheel diameter from config
        Config::GEAR_RATIO,             // gear ratio from config
        Config::WHEEL_BASE_WIDTH        // track width from config
    );
    Logger::getInstance()->log("Left wheel velocities: %f %f", wheel_velocities[0], m_localization->getLeftVelocity().linear);
    Logger::getInstance()->log("Right wheel velocities: %f %f", wheel_velocities[1], m_localization->getRightVelocity().linear);

    // Calculate motor voltages using drivetrain controller
    auto voltages = m_driveController->calculateVoltages(
        wheel_velocities[0],                       // left velocity setpoint
        wheel_velocities[1],                       // right velocity setpoint
        m_localization->getLeftVelocity().linear,  // current left velocity
        m_localization->getRightVelocity().linear, // current right velocity
        left_accel,                                // left acceleration
        right_accel                                // right acceleration
    );
    Logger::getInstance()->log("Voltages: %d %d", voltages.left, voltages.right);

    // Apply voltages to motors
    m_leftDrivetrain->move(voltages.left);
    m_rightDrivetrain->move(voltages.right);
}

void Robot::processAction(const ActionPoint &ap)
{
    // Process action point
    Logger::getInstance()->log("Executing action with %zu parameters", ap.actions.size());

    // Placeholder
    for (const auto &action : ap.actions)
    {
        Logger::getInstance()->log("Action: %f", action);
    }

    // TODO: Implement action processing logic after bot is built
}

bool Robot::followTrajectory(Trajectory &trajectory)
{
    Logger::getInstance()->log("Starting trajectory following");
    if (!isInitialized())
    {
        Logger::getInstance()->logError("Robot is not initialized, cannot follow trajectory");
        return false;
    }
    if (trajectory.empty())
    {
        Logger::getInstance()->logError("Trajectory is empty, cannot follow");
        return false;
    }

    // Set the flag to indicate we are following a trajectory
    m_isFollowingTrajectory = true;

    Logger::getInstance()->log("Following trajectory with %s localization",
                               m_localization->getTypeName().c_str());

    const double START_TIME = pros::millis();

    size_t trajectory_index = 0;
    try
    {
        const TrajectoryPoint &first_trajectory_point = std::get<TrajectoryPoint>(trajectory.getByIndex(1));

        Pose initial_pos = {first_trajectory_point.x, first_trajectory_point.y, first_trajectory_point.theta};
        resetPosition(initial_pos);
        Logger::getInstance()->log("Initial pose: %f %f %f", initial_pos.x, initial_pos.y, initial_pos.theta);
    }
    catch (const std::bad_variant_access &e)
    {
        Logger::getInstance()->logError("Error: Second point is not a TrajectoryPoint!");
    }

    // Main control loop
    Logger::getInstance()->log("Route size: %d", trajectory.size());
    Logger::getInstance()->log("Has next: %d", trajectory.hasNext());

    while (trajectory.hasNext())
    {
        Logger::getInstance()->log("Processing trajectory point %zu", trajectory_index);

        // Update localization
        m_localization->update();

        // Check if the current waypoint is a trajectory point or action point
        const DataPoint *point = trajectory.getNext();
        std::visit([this](const auto &p)
                   {
            if constexpr (std::is_same_v<std::decay_t<decltype(p)>, TrajectoryPoint>) {
                this->processTrajectory(p);
            } else {
                this->processAction(p);
            } }, *point);

        trajectory_index++;

        // Use delta time from configuration
        pros::delay(Config::DT); // Convert to milliseconds
    }

    // Stop motors
    m_leftDrivetrain->move(0);
    m_rightDrivetrain->move(0);

    m_isFollowingTrajectory = false;

    Logger::getInstance()->log("Trajectory following completed");
    return true; // Indicate that trajectory following completed successfully
}

void Robot::tuneTrackWidth()
{
    PIDController(2.0, 0.0, 0.0, -127.0, 127.0, 50.0);
    Logger::getInstance()->log("Track width tuning started");

    double heading = Angles::degreesToRadians(m_inertial->get_heading() * -1);

    double target_heading = Angles::degreesToRadians(heading + 180.0);

    double left_arc_length = 0.0;
    double right_arc_length = 0.0;
    Logger::getInstance()->log("heading: %f", heading);
    Logger::getInstance()->log("Target heading: %f", target_heading);

    while (abs(Angles::angleDifference(target_heading, heading)) > 0.1)
    {
        // Get current heading
        heading = Angles::degreesToRadians(m_inertial->get_heading() * -1);
        Logger::getInstance()->log("Current heading: %f", heading);

        // Calculate error
        double error = Angles::angleDifference(target_heading, heading);

        // Calculate control output
        double output = 100.0 * error; // Proportional control

        // Apply output to motors
        m_leftDrivetrain->move(-output);
        m_rightDrivetrain->move(output);

        pros::delay(10);
    }

    Logger::getInstance()->log("Track width tuning completed");
    m_leftDrivetrain->move(0);
    m_rightDrivetrain->move(0);

    left_arc_length = Units::ticksToDistance(m_leftDrivetrain->get_position(), Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);
    right_arc_length = Units::ticksToDistance(m_rightDrivetrain->get_position(), Config::TICKS_PER_ROTATION, Config::WHEEL_DIAMETER, Config::GEAR_RATIO);

    Logger::getInstance()->log("Left arc length: %f", left_arc_length);
    Logger::getInstance()->log("Right arc length: %f", right_arc_length);


}
