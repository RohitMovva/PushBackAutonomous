#include "hardware/robot.hpp"
#include <stdexcept>

Robot::Robot()
    : m_leftDrivetrain(nullptr), m_rightDrivetrain(nullptr), m_inertial(nullptr), m_driveController(nullptr), m_ramseteController(nullptr), m_odometry(nullptr)
{
    // Default constructor initializes all pointers to null
    // Robot will not be functional until proper initialization
}

Robot::Robot(pros::MotorGroup *leftDrivetrain,
             pros::MotorGroup *rightDrivetrain,
             pros::Imu *inertial,
             DrivetrainController *driveController,
             RamseteController *ramseteController,
             Odometry *odometry)
    : m_leftDrivetrain(leftDrivetrain), m_rightDrivetrain(rightDrivetrain), m_inertial(inertial), m_driveController(driveController), m_ramseteController(ramseteController), m_odometry(odometry)
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
    if (!odometry)
    {
        throw std::invalid_argument("Odometry pointer cannot be null");
    }
}

Robot::~Robot()
{
    // Stop any active trajectory following before destruction
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    // Note: We don't delete the pointers as this class doesn't own them
    // The calling code is responsible for managing the lifetime of these objects
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

auto Robot::getCurrentPosition() const -> decltype(m_odometry->getPose())
{
    if (!m_odometry)
    {
        throw std::runtime_error("Odometry system not initialized");
    }

    return m_odometry->getPose();
}

void Robot::resetPosition(const Pose &position)
{
    if (!m_odometry)
    {
        throw std::runtime_error("Odometry system not initialized");
    }

    // Stop any active trajectory following before resetting position
    if (isFollowingTrajectory())
    {
        stopTrajectory();
    }

    m_odometry->setPose(position);
}

bool Robot::isInitialized() const
{
    return (m_leftDrivetrain != nullptr &&
            m_rightDrivetrain != nullptr &&
            m_inertial != nullptr &&
            m_driveController != nullptr &&
            m_ramseteController != nullptr &&
            m_odometry != nullptr);
}

void Robot::processTrajectory(const TrajectoryPoint &tp)
{
    // Process trajectory
    Pose current_pose = m_odometry->getPose();

    Logger::getInstance()->log("Pose: %f %f %f", current_pose.x, current_pose.y, current_pose.theta);
    Logger::getInstance()->log("Goal: %f %f %f", tp.x, tp.y, tp.theta);

    // Calculate accelerations using next waypoint
    double left_accel = tp.linear_accel - (tp.angular_accel * m_odometry->getTrackWidth() / 2.0);
    double right_accel = tp.linear_accel + (tp.angular_accel * m_odometry->getTrackWidth() / 2.0);

    // Get RAMSETE controller output
    auto ramsete_output = m_ramseteController->calculate(
        current_pose.x, current_pose.y, current_pose.theta,
        tp.x, tp.y, tp.theta,
        tp.linear_vel, tp.angular_vel);
    Logger::getInstance()->log("Ramsete Output: %f %f", ramsete_output[0], ramsete_output[1]);
    Logger::getInstance()->log("MP Output: %f %f", tp.linear_vel, tp.angular_vel);

    // Convert RAMSETE output to wheel velocities
    auto wheel_velocities = m_ramseteController->calculate_wheel_velocities(
        ramsete_output[0],          // linear velocity
        ramsete_output[1],          // angular velocity
        2.75,                       // wheel diameter (inches)
        48.0 / 36.0,                // gear ratio
        m_odometry->getTrackWidth() // track width (inches)
    );
    Logger::getInstance()->log("Left wheel velocities: %f %f", wheel_velocities[0], m_odometry->getLeftVelocity().linear);
    Logger::getInstance()->log("Right wheel velocities: %f %f", wheel_velocities[1], m_odometry->getRightVelocity().linear);

    // Calculate motor voltages using drivetrain controller
    auto voltages = m_driveController->calculateVoltages(
        wheel_velocities[0],                   // left velocity setpoint
        wheel_velocities[1],                   // right velocity setpoint
        m_odometry->getLeftVelocity().linear,  // current left velocity
        m_odometry->getRightVelocity().linear, // current right velocity
        left_accel,                            // left acceleration
        right_accel                            // right acceleration
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
    if (!isInitialized())
    {
        return false; // Cannot follow trajectory if robot is not initialized
    }
    if (trajectory.empty())
    {
        return false;
    }

    // Set the flag to indicate we are following a trajectory
    m_isFollowingTrajectory = true;

    Logger::getInstance()->log("Following trajectory");

    const double START_TIME = pros::millis();
    const double DT = 10;                     // 10ms
    const double track_width = 14.1966209238; // inches

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
        Logger::getInstance()->logError("Error: Second point is not an TrajectoryPoint!");
    }

    // Main control loop
    Logger::getInstance()->log("Route size: %d", trajectory.size());

    while (trajectory.hasNext())
    {
        pros::lcd::print(0, "Trajectory index: %d", trajectory_index);
        m_odometry->update();
        const double current_time = pros::millis() - START_TIME;

        // Check if the current waypoint is a node instead of a trajectory point
        const DataPoint *point = trajectory.getNext();
        std::visit([this](const auto &p)
                   {
            if constexpr (std::is_same_v<std::decay_t<decltype(p)>, TrajectoryPoint>) {
                this->processTrajectory(p);
            } else {
                this->processAction(p);
            } }, *point);
        trajectory_index++;

        pros::delay(DT);
    }

    // Stop motors
    m_leftDrivetrain->move(0);
    m_rightDrivetrain->move(0);

    m_isFollowingTrajectory = false;

    return true; // Indicate that trajectory following has started successfully
}