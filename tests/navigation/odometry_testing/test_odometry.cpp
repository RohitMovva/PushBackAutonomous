#include "gtest/gtest.h"
#include "mock_pros.h"
#include "navigation/odometry.h"
#include <memory>
#include <thread>
#include <cmath>

// The rest of the test file remains largely the same, but we'll add a main function
// and modify the build instructions

class OdometryTest : public ::testing::Test {
protected:
    // Robot specifications
    const double WHEEL_DIAMETER = 2.75; // inches
    const double TRACK_WIDTH = 11.5; // inches
    const double LATERAL_OFFSET = 5.0; // inches
    const double MAX_VELOCITY = (450.0 * 2.75 * M_PI) / (60.0);

    std::unique_ptr<pros::Motor> leftMotor1;
    std::unique_ptr<pros::Motor> leftMotor2;
    std::unique_ptr<pros::Motor> rightMotor1;
    std::unique_ptr<pros::Motor> rightMotor2;
    std::unique_ptr<pros::MotorGroup> leftDrive;
    std::unique_ptr<pros::MotorGroup> rightDrive;
    std::unique_ptr<pros::Encoder> lateralEncoder;
    std::unique_ptr<pros::Imu> imu;
    std::unique_ptr<Odometry> odometry;

    void SetUp() override {
        // Initialize the time properly
        pros::set_millis(0);  // Reset mock time
        odometry->reset();    // Reset odometry with clean initial state

        // In simulateMotorMovement()
        // Add time increment simulation
        pros::increment_millis(static_cast<int>(stepTime * 1000));

        // Initialize motors (port numbers are arbitrary for testing)
        leftMotor1 = std::make_unique<pros::Motor>(1);
        leftMotor2 = std::make_unique<pros::Motor>(2);
        rightMotor1 = std::make_unique<pros::Motor>(3);
        rightMotor2 = std::make_unique<pros::Motor>(4);

        // Create motor groups
        std::vector<pros::Motor*> leftMotors = {leftMotor1.get(), leftMotor2.get()};
        std::vector<pros::Motor*> rightMotors = {rightMotor1.get(), rightMotor2.get()};
        leftDrive = std::make_unique<pros::MotorGroup>(leftMotors);
        rightDrive = std::make_unique<pros::MotorGroup>(rightMotors);

        // Initialize sensors
        lateralEncoder = std::make_unique<pros::Encoder>(1, 2);
        imu = std::make_unique<pros::Imu>(10);

        // Create odometry instance with filters disabled
        odometry = std::make_unique<Odometry>(*leftDrive, *rightDrive, *lateralEncoder, *imu,
                                            TRACK_WIDTH, LATERAL_OFFSET,
                                      void SetUp() override {
        // Initialize the time properly
        pros::set_millis(0);  // Reset mock time
        odometry->reset();    // Reset odometry with clean initial state

        // In simulateMotorMovement()
        // Add time increment simulation
        pros::increment_millis(static_cast<int>(stepTime * 1000));

        // Initialize motors (port numbers are arbitrary for testing)
        leftMotor1 = std::make_unique<pros::Motor>(1);
        leftMotor2 = std::make_unique<pros::Motor>(2);
        rightMotor1 = std::make_unique<pros::Motor>(3);
        rightMotor2 = std::make_unique<pros::Motor>(4);

        // Create motor groups
        std::vector<pros::Motor*> leftMotors = {leftMotor1.get(), leftMotor2.get()};
        std::vector<pros::Motor*> rightMotors = {rightMotor1.get(), rightMotor2.get()};
        leftDrive = std::make_unique<pros::MotorGroup>(leftMotors);
        rightDrive = std::make_unique<pros::MotorGroup>(rightMotors);

        // Initialize sensors
        lateralEncoder = std::make_unique<pros::Encoder>(1, 2);
        imu = std::make_unique<pros::Imu>(10);

        // Create odometry instance with filters disabled
        odometry = std::make_unique<Odometry>(*leftDrive, *rightDrive, *lateralEncoder, *imu,
                                            TRACK_WIDTH, LATERAL_OFFSET,
                                            false, false, false);
    }

    void TearDown() override {
        odometry.reset();
        imu.reset();
        lateralEncoder.reset();
        leftDrive.reset();
        rightDrive.reset();
        leftMotor1.reset();
        leftMotor2.reset();
        rightMotor1.reset();
        rightMotor2.reset();
    }

    // Helper function to simulate motor movement
    // First, let's update the test helper function to better handle rotations:
    // Also let's modify the test helper function:
    void simulateMotorMovement(double leftDistance, double rightDistance, double lateralDistance, double heading) {
        const double GEAR_RATIO = 36.0 / 48.0;
        double ticksPerInch = (900.0) / (WHEEL_DIAMETER * M_PI * GEAR_RATIO);
        
        std::cout << "\nSimulating movement:" << std::endl;
        std::cout << "Left distance: " << leftDistance << " inches" << std::endl;
        std::cout << "Right distance: " << rightDistance << " inches" << std::endl;
        std::cout << "Lateral distance: " << lateralDistance << " inches" << std::endl;
        std::cout << "Target heading: " << heading << " degrees" << std::endl;
        
        int leftTicks = static_cast<int>(leftDistance * ticksPerInch);
        int rightTicks = static_cast<int>(rightDistance * ticksPerInch);
        int lateralTicks = static_cast<int>(lateralDistance * ticksPerInch);

        std::cout << "Left ticks: " << leftTicks << std::endl;
        std::cout << "Right ticks: " << rightTicks << std::endl;
        std::cout << "Lateral ticks: " << lateralTicks << std::endl;

        // Simulate gradual movement in smaller steps
        const int STEPS = 20;  // Increased number of steps
        for (int i = 1; i <= STEPS; i++) {
            double progress = static_cast<double>(i) / STEPS;
            
            int currentLeftTicks = static_cast<int>(leftTicks * progress);
            int currentRightTicks = static_cast<int>(rightTicks * progress);
            int currentLateralTicks = static_cast<int>(lateralTicks * progress);
            double currentHeading = heading * progress;

            for (const auto& motor : {leftMotor1.get(), leftMotor2.get()}) {
                motor->set_position(currentLeftTicks);
            }
            for (const auto& motor : {rightMotor1.get(), rightMotor2.get()}) {
                motor->set_position(currentRightTicks);
            }
            lateralEncoder->set_value(currentLateralTicks);
            imu->set_rotation(currentHeading);

            // Update odometry
            odometry->update();
            
            // Use a longer delay between updates
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

};

// Test initialization
TEST_F(OdometryTest, InitializationTest) {
    EXPECT_DOUBLE_EQ(odometry->getX(), 0.0);
    EXPECT_DOUBLE_EQ(odometry->getY(), 0.0);
    EXPECT_DOUBLE_EQ(odometry->getHeading(), 0.0);
}

// Test straight line motion
TEST_F(OdometryTest, StraightLineMotion) {
    // First, ensure we're starting from a known state
    odometry->reset();
    
    // Simulate moving forward 24 inches
    simulateMotorMovement(24.0, 24.0, 0.0, 0.0);
    
    // Important: Need multiple updates to properly track motion
    for(int i = 0; i < 10; i++) { // Update several times to simulate continuous motion
        odometry->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_NEAR(odometry->getX(), 24.0, 0.1);
    EXPECT_NEAR(odometry->getY(), 0.0, 0.1);
    EXPECT_NEAR(odometry->getHeading(), 0.0, 0.1);
}

// Test pure rotation
TEST_F(OdometryTest, PureRotation) {
    odometry->reset();
    
    // For a 90-degree turn, calculate the correct arc lengths
    double turnRadius = TRACK_WIDTH / 2.0;
    double angleRad = M_PI / 2.0; // 90 degrees
    double leftArc = angleRad * (turnRadius + TRACK_WIDTH/2.0);  // Outside wheel
    double rightArc = -angleRad * (turnRadius - TRACK_WIDTH/2.0); // Inside wheel (negative for CCW)
    
    simulateMotorMovement(leftArc, rightArc, 0.0, 90.0);

    EXPECT_NEAR(odometry->getX(), 0.0, 0.1);
    EXPECT_NEAR(odometry->getY(), 0.0, 0.1);
    EXPECT_NEAR(odometry->getHeading(), M_PI/2.0, 0.1);
}

// Test arc motion
TEST_F(OdometryTest, ArcMotion) {
    odometry->reset();
    
    // Calculate a quarter circle with 24-inch radius
    double radius = 24.0;
    double angleRad = M_PI / 2.0; // 90 degrees
    
    // Calculate the arc lengths for inner and outer wheels
    double innerRadius = radius - (TRACK_WIDTH / 2.0);
    double outerRadius = radius + (TRACK_WIDTH / 2.0);
    double innerArc = angleRad * innerRadius;
    double outerArc = angleRad * outerRadius;
    
    // Simulate the motion in smaller steps
    simulateMotorMovement(outerArc, innerArc, 0.0, 90.0);

    EXPECT_NEAR(odometry->getX(), radius, 0.5);
    EXPECT_NEAR(odometry->getY(), radius, 0.5);
    EXPECT_NEAR(odometry->getHeading(), M_PI/2.0, 0.1);
}

// Test lateral motion contribution
TEST_F(OdometryTest, LateralMotion) {
    // Simulate forward motion with lateral drift
    simulateMotorMovement(24.0, 24.0, 6.0, 0.0);
    odometry->update();

    EXPECT_NEAR(odometry->getX(), 24.0, 0.1);
    EXPECT_NEAR(odometry->getY(), 6.0, 0.1);
    EXPECT_NEAR(odometry->getHeading(), 0.0, 0.1);
}

// Test reset functionality
TEST_F(OdometryTest, ResetTest) {
    // First move the robot
    simulateMotorMovement(24.0, 24.0, 6.0, 45.0);
    odometry->update();

    // Then reset
    odometry->reset();

    EXPECT_DOUBLE_EQ(odometry->getX(), 0.0);
    EXPECT_DOUBLE_EQ(odometry->getY(), 0.0);
    EXPECT_DOUBLE_EQ(odometry->getHeading(), 0.0);
}

// Test setPose functionality
TEST_F(OdometryTest, SetPoseTest) {
    Pose newPose(10.0, 15.0, M_PI/4.0);
    odometry->setPose(newPose);

    EXPECT_DOUBLE_EQ(odometry->getX(), 10.0);
    EXPECT_DOUBLE_EQ(odometry->getY(), 15.0);
    EXPECT_DOUBLE_EQ(odometry->getHeading(), M_PI/4.0);
}

// Test velocity calculations
TEST_F(OdometryTest, VelocityTest) {
    odometry->reset();
    
    // Calculate a reasonable velocity for testing
    // 450 RPM = 7.5 RPS = 7.5 * (2.75π * 36/48) ≈ 48.7 inches/sec max
    const double targetVelocity = 6.84; // Much lower than max for testing
    const double moveTime = 0.5; // seconds
    const double targetDistance = targetVelocity * moveTime;
    
    // Move in smaller increments
    const int STEPS = 10;
    const double stepDistance = targetDistance / STEPS;
    const double stepTime = moveTime / STEPS;
    
    for (int i = 0; i < STEPS; i++) {
        // Set encoder values
        double currentDistance = stepDistance * (i + 1);
        simulateMotorMovement(currentDistance, currentDistance, 0.0, 0.0);
        
        // Allow time for velocity calculation
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(stepTime * 1000)));
        odometry->update();
    }
    
    Velocity leftVel = odometry->getLeftVelocity();
    Velocity rightVel = odometry->getRightVelocity();
    
    EXPECT_NEAR(leftVel.linear, targetVelocity, targetVelocity * 0.1);
    EXPECT_NEAR(rightVel.linear, targetVelocity, targetVelocity * 0.1);
}

// Test max speed constraints
TEST_F(OdometryTest, MaxSpeedTest) {
    // Simulate movement at max speed (450 RPM with 2.75" wheels)
    double maxSpeed = (450.0 * WHEEL_DIAMETER * M_PI) / 60.0; // inches per second
    double timeStep = 0.1; // seconds
    double distance = maxSpeed * timeStep;
    
    simulateMotorMovement(distance, distance, 0.0, 0.0);
    odometry->update();
    
    Velocity leftVel = odometry->getLeftVelocity();
    EXPECT_LE(std::abs(leftVel.linear), maxSpeed);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}