/**
 * @file robot.hpp
 * @brief Robot class encompassing all hardware and control systems
 * @author Rohit Movva
 * @date 2025
 */

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "main.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"

#include "controllers/ramsete_controller.hpp"
#include "controllers/drivetrain_controller.hpp"
#include "controllers/pid_controller.hpp"

#include "navigation/odometry.hpp"
#include "navigation/trajectory.hpp"

#include "utilities/logger.hpp"

/**
 * @class Robot
 * @brief Main robot class for VEX competition robot using PROS
 *
 * This class provides high-level control for a VEX robot, including
 * drivetrain control, trajectory following, and autonomous motion.
 * It integrates with PROS motion control systems including Ramsete
 * controller and odometry for precise autonomous navigation.
 */
class Robot
{
private:
    pros::MotorGroup *m_leftDrivetrain;      ///< Left side drivetrain motor group
    pros::MotorGroup *m_rightDrivetrain;     ///< Right side drivetrain motor group
    pros::Imu *m_inertial;                   ///< Right side drivetrain motor group
    DrivetrainController *m_driveController; ///< Drivetrain velocity controller
    RamseteController *m_ramseteController;  ///< Ramsete controller for trajectory following
    Odometry *m_odometry;                    ///< Odometry system for position tracking

    bool m_isFollowingTrajectory; ///< Flag indicating if trajectory following is active
    /**
     * @brief Process an trajectory point from the trajectory
     * @param tp Trajectory point containing kinematic data
     */
    void processTrajectory(const TrajectoryPoint &tp);

    /**
     * @brief Process an action point from the trajectory
     * @param ap Action point containing actions to execute
     */
    void processAction(const ActionPoint &ap);

public:
    /**
     * @brief Default constructor for Robot class
     */
    Robot();

    /**
     * @brief Constructor with full initialization parameters
     *
     * Initializes the robot with all necessary control systems for
     * autonomous operation including drivetrain, controllers, and odometry.
     *
     * @param leftDrivetrain Pointer to left side motor group
     * @param rightDrivetrain Pointer to right side motor group
     * @param inertial Pointer to inertial sensor for heading and orientation
     * @param driveController Pointer to drivetrain velocity controller
     * @param ramseteController Pointer to Ramsete controller for path following
     * @param odometry Pointer to odometry system for localization
     *
     * @pre All pointer parameters must be valid and properly initialized
     * @post Robot is fully initialized and ready for autonomous operation
     *
     * @throws std::invalid_argument if any pointer parameter is null
     */
    Robot(pros::MotorGroup *leftDrivetrain,
          pros::MotorGroup *rightDrivetrain,
          pros::Imu *inertial,
          DrivetrainController *driveController,
          RamseteController *ramseteController,
          Odometry *odometry);
    /**
     * @brief Destructor for Robot class
     *
     * Properly cleans up resources. Note that this class does not
     * own the passed pointers, so they are not deleted here.
     */
    ~Robot();

    /**
     * @brief Follow a predefined trajectory using motion profiling
     *
     * Executes autonomous motion along a specified trajectory using
     * the Ramsete controller and motion profiling. The robot will
     * follow the path while maintaining velocity and acceleration
     * constraints defined in the motion profile.
     *
     * @param trajectory The trajectory to follow
     *
     * @return true if trajectory execution started successfully, false otherwise
     *
     * @pre Robot must be properly initialized with all control systems
     * @pre Trajectory must be valid follow robot constraints
     * @post Robot begins following the specified trajectory
     *
     * @warning Ensure the trajectory is within the robot's physical limits
     *          and field boundaries before execution
     */

    bool followTrajectory(Trajectory &trajectory);

    /**
     * @brief Check if robot is currently following a trajectory
     *
     * @return true if trajectory following is active, false otherwise
     */
    bool isFollowingTrajectory() const;

    /**
     * @brief Stop current trajectory following
     *
     * Immediately stops any active trajectory following and
     * brings the robot to a controlled stop.
     *
     * @post Robot stops moving and trajectory following is disabled
     */
    void stopTrajectory();

    /**
     * @brief Get current robot position from odometry
     *
     * @return Current robot position and heading
     * @pre Odometry system must be properly initialized
     */
    auto getCurrentPosition() const -> decltype(m_odometry->getPose());

    /**
     * @brief Reset robot odometry to specified position
     *
     * @param position New position to set for odometry
     * @pre Odometry system must be properly initialized
     */
    void resetPosition(const Pose &position);

    /**
     * @brief Check if robot systems are properly initialized
     *
     * @return true if all required systems are initialized, false otherwise
     */
    bool isInitialized() const;

    // Disable copy constructor and assignment operator
    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;
};

#endif