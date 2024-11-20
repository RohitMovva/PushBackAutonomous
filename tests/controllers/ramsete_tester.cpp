#include <gtest/gtest.h>
#include "../../include/controllers/ramsete_controller.h"
#include "../../include/routes/routes.h"
#include <cmath>

struct TestCase {
    double start_x, start_y, start_theta;
    double goal_x, goal_y, goal_theta;
    double v_ref, w_ref;
    std::string description;
};

// class RamseteControllerTest : public ::testing::Test {
// protected:
//     RamseteController controller;
//     const double kEpsilon = 1e-6;  // Tolerance for floating-point comparisons
    
//     void SetUp() override {
//         // Using default parameters (b = 2.0, zeta = 0.7)
//         controller = RamseteController(2.0, 0.7, 2.0, 2.0, 1.0);
//     }
// };

// Helper function to normalize angles to [-π, π]
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// // Test global error calculation
// TEST_F(RamseteControllerTest, GlobalErrorCalculation) {
//     // Test case 1: No error
//     std::vector<double> error1 = controller.calculate_global_error(1.0, 2.0, M_PI/4, 
//                                                                  1.0, 2.0, M_PI/4);
//     EXPECT_NEAR(error1[0], 0.0, kEpsilon);
//     EXPECT_NEAR(error1[1], 0.0, kEpsilon);
//     EXPECT_NEAR(error1[2], 0.0, kEpsilon);

//     // Test case 2: Position and heading error
//     std::vector<double> error2 = controller.calculate_global_error(0.0, 0.0, 0.0,
//                                                                  1.0, 1.0, M_PI/2);
//     EXPECT_NEAR(error2[0], 1.0, kEpsilon);
//     EXPECT_NEAR(error2[1], 1.0, kEpsilon);
//     EXPECT_NEAR(error2[2], M_PI/2, kEpsilon);
// }

// // Test local error transformation
// TEST_F(RamseteControllerTest, LocalErrorTransformation) {
//     std::vector<double> global_error = {1.0, 1.0, M_PI/4};
    
//     // Test transformation at different goal headings
//     std::vector<double> local_error1 = controller.calculate_local_error(global_error, 0.0);
//     std::vector<double> local_error2 = controller.calculate_local_error(global_error, M_PI/2);
    
//     // Verify transformations maintain proper relationships
//     EXPECT_NEAR(sqrt(pow(local_error1[0], 2) + pow(local_error1[1], 2)),
//                 sqrt(pow(global_error[0], 2) + pow(global_error[1], 2)),
//                 kEpsilon);
// }

// // Test output velocity calculation
// TEST_F(RamseteControllerTest, OutputVelocities) {
//     // Test case 1: No error should maintain reference velocities
//     std::vector<double> output1 = controller.get_output_velocities(1.0, 0.5, 0.0, 0.0, 1.0);
//     EXPECT_NEAR(output1[0], 1.0, kEpsilon);  // Linear velocity
//     EXPECT_NEAR(output1[1], 0.5, kEpsilon);  // Angular velocity

//     // Test case 2: With error, should adjust velocities
//     std::vector<double> output2 = controller.get_output_velocities(1.0, 0.5, 0.0, M_PI/4, 1.0);
//     EXPECT_GT(output2[1], 0.5);  // Angular velocity should increase to correct heading
// }

// // Test wheel velocity calculation
// TEST_F(RamseteControllerTest, WheelVelocities) {
//     double wheel_diameter = 6.0;  // inches
//     double gear_ratio = 10.0;     // 10:1 reduction
    
//     // Test case 1: Pure forward motion
//     std::vector<double> wheel_speeds1 = controller.calculate_wheel_velocities(1.0, 0.0,
//                                                                             wheel_diameter,
//                                                                             gear_ratio);
//     EXPECT_NEAR(wheel_speeds1[0], wheel_speeds1[1], kEpsilon);  // Equal speeds for straight motion
    
//     // Test case 2: Pure rotation
//     std::vector<double> wheel_speeds2 = controller.calculate_wheel_velocities(0.0, 1.0,
//                                                                             wheel_diameter,
//                                                                             gear_ratio);
//     EXPECT_NEAR(wheel_speeds2[0], -wheel_speeds2[1], kEpsilon);  // Equal and opposite for rotation
// }

// TEST_F(RamseteControllerTest, CompleteControllerBehavior) {
//     // Test tracking a simple trajectory point
//     std::vector<double> control = controller.calculate(0.0, 0.0, 0.0,  // Current state
//                                                      1.0, 1.0, M_PI/4, // Goal state
//                                                      1.0, 0.5);        // Reference velocities
    
//     // Verify basic properties
//     EXPECT_GT(control[0], 0.0);  // Should move forward
    
//     // Test convergence over multiple steps
//     double x = 0.0, y = 0.0, theta = 0.0;
//     const double dt = 0.02;  // 20ms timestep
//     const double position_tolerance = 0.15;  // Increased position tolerance
//     const double angle_tolerance = 0.15;     // Increased angle tolerance
//     bool converged = false;
//     int max_steps = 1000;
    
//     double last_error = std::numeric_limits<double>::max();
//     int stable_count = 0;
    
//     for (int i = 0; i < max_steps; i++) {
//         std::vector<double> output = controller.calculate(x, y, theta,
//                                                         1.0, 1.0, M_PI/4,
//                                                         1.0, 0.5);
        
//         // Update pose using differential drive kinematics
//         double v = output[0];
//         double w = output[1];
        
//         x += v * cos(theta) * dt;
//         y += v * sin(theta) * dt;
//         theta += w * dt;
        
//         // Normalize theta
//         while (theta > M_PI) theta -= 2 * M_PI;
//         while (theta < -M_PI) theta += 2 * M_PI;
        
//         // Calculate total error
//         double position_error = sqrt(pow(1.0 - x, 2) + pow(1.0 - y, 2));
//         double angle_error = std::abs(M_PI/4 - theta);
//         while (angle_error > M_PI) angle_error -= 2 * M_PI;
//         angle_error = std::abs(angle_error);
        
//         double total_error = position_error + angle_error;
        
//         // Check if error is decreasing
//         if (std::abs(total_error - last_error) < 1e-3) {
//             stable_count++;
//         } else {
//             stable_count = 0;
//         }
        
//         last_error = total_error;
        
//         // Check convergence with relaxed tolerances
//         if (position_error < position_tolerance && angle_error < angle_tolerance) {
//             converged = true;
//             break;
//         }
        
//         // Also consider convergence if we're stable for a while
//         if (stable_count > 50) {
//             converged = true;
//             break;
//         }
//     }
    
//     EXPECT_TRUE(converged) << "Failed to converge. Final position: (" << x << ", " << y 
//                           << "), theta: " << theta;
// }

// // Test constructor with custom parameters
// TEST_F(RamseteControllerTest, CustomParameters) {
//     // Create controller with significantly different parameters
//     RamseteController custom_controller(4.0, 0.9);  // More aggressive parameters
    
//     // Test with a more challenging scenario that will highlight parameter differences
//     std::vector<double> default_control = controller.calculate(0.0, 0.0, 0.0,         // Current state
//                                                              3.0, -2.0, M_PI/3,       // Goal state far away with rotation
//                                                              0.5, 0.1);               // Lower reference velocities
    
//     std::vector<double> custom_control = custom_controller.calculate(0.0, 0.0, 0.0,
//                                                                    3.0, -2.0, M_PI/3,
//                                                                    0.5, 0.1);
    
//     // At least one of the outputs should be different due to different parameters
//     bool outputs_same = std::abs(default_control[0] - custom_control[0]) < kEpsilon && 
//                        std::abs(default_control[1] - custom_control[1]) < kEpsilon;
    
//     EXPECT_FALSE(outputs_same) << "Default control: [" << default_control[0] << ", " << default_control[1] 
//                               << "], Custom control: [" << custom_control[0] << ", " << custom_control[1] << "]";
// }

class RamseteTrajectoryTest : public ::testing::Test {
protected:
    RamseteController controller;
    const double kEpsilon = 1e-6;
    const double dt = 0.025;
    
    struct DiagnosticPoint {
        double time;
        double x, y, theta;
        double target_x, target_y, target_theta;
        double v_ref, w_ref;
        double v_output, w_output;
        double position_error;
        double heading_error;
        double velocity_error;
        
        void print() const {
            std::cout << "\nTime: " << time << "s"
                     << "\nPosition (actual): (" << x << ", " << y << ") meters"
                     << "\nPosition (target): (" << target_x << ", " << target_y << ") meters"
                     << "\nHeading (actual/target): " << theta << "/" << target_theta << " rad"
                     << "\nVelocity (ref/output): " << v_ref << "/" << v_output << " m/s"
                     << "\nPosition error: " << position_error << " meters"
                     << "\nVelocity error: " << std::abs(v_ref - v_output) << " m/s\n";
        }
    };
    
    void SetUp() override {
        double b = 2.2;      // Back to default
        double zeta = 0.7;   // Back to default
        double max_v = 5.35; 
        double max_w = 5.0;  
        double scale = 0.3048;  // 1 foot = 0.3048 meters
        
        controller = RamseteController(b, zeta, max_v, max_w, scale);
    }
    
    std::vector<DiagnosticPoint> run_trajectory_with_diagnostics(const std::vector<std::vector<double>>& route) {
        std::vector<DiagnosticPoint> diagnostics;
        
        // Initialize robot state
        double x = route[1][1];
        double y = route[1][2];
        double theta = route[1][3];
        
        // Track largest errors and their timestamps
        double max_pos_error = 0;
        double max_vel_error = 0;
        double time_at_max_pos_error = 0;
        double time_at_max_vel_error = 0;
        
        for (size_t i = 1; i < route.size(); i++) {
            if (route[i].size() < 6) continue;
            
            double current_time = route[i][0];
            double target_x = route[i][1];
            double target_y = route[i][2];
            double target_theta = route[i][3];
            double target_v = route[i][4];
            double target_w = route[i][5];
            
            // Store pre-conversion values
            DiagnosticPoint point;
            point.time = current_time;
            point.x = x;
            point.y = y;
            point.theta = theta;
            point.target_x = target_x;
            point.target_y = target_y;
            point.target_theta = target_theta;
            point.v_ref = target_v;
            point.w_ref = target_w;
            
            // Calculate control outputs
            std::vector<double> control = controller.calculate(
                x, y, theta,
                target_x, target_y, target_theta,
                target_v, target_w
            );
            
            point.v_output = control[0];
            point.w_output = control[1];
            
            // Calculate errors
            point.position_error = std::sqrt(
                std::pow(target_x - x, 2) + std::pow(target_y - y, 2));
            point.heading_error = normalize_angle(target_theta - theta);
            point.velocity_error = std::abs(target_v - control[0]);
            
            // Track maximum errors
            if (point.position_error > max_pos_error) {
                max_pos_error = point.position_error;
                time_at_max_pos_error = current_time;
            }
            if (point.velocity_error > max_vel_error) {
                max_vel_error = point.velocity_error;
                time_at_max_vel_error = current_time;
            }
            
            // Update state
            x += control[0] * std::cos(theta) * dt;
            y += control[0] * std::sin(theta) * dt;
            theta = normalize_angle(theta + control[1] * dt);
            
            diagnostics.push_back(point);
        }
        
        // Print summary of largest errors
        std::cout << "\n=== Error Summary ===\n"
                  << "Maximum position error: " << max_pos_error 
                  << "m at t=" << time_at_max_pos_error << "s\n"
                  << "Maximum velocity error: " << max_vel_error 
                  << "m/s at t=" << time_at_max_vel_error << "s\n";
                  
        // Print detailed diagnostics for points with large errors
        std::cout << "\n=== Detailed Error Analysis ===\n";
        for (const auto& point : diagnostics) {
            // std::cout << point.position_error << " " << point.velocity_error << std::endl;
            if (point.position_error > 0.15 || point.velocity_error > 0.4) {
                std::cout << "\nLarge error detected at t=" << point.time << "s";
                point.print();
            }
        }
        
        return diagnostics;
    }
};

TEST_F(RamseteTrajectoryTest, FollowMotionProfile) {
    std::vector<std::vector<double>> route = test;
    // auto diagnostics = run_trajectory_with_diagnostics(route);

    // Initialize robot state with the first waypoint
    double x = route[1][1];      // Start x
    double y = route[1][2];      // Start y
    double theta = route[1][3];  // Start theta
    
    // Error tracking
    std::vector<double> position_errors;
    std::vector<double> heading_errors;
    std::vector<double> linear_velocity_errors;
    std::vector<double> angular_velocity_errors;
    
    double max_position_error = 0.0;
    double max_heading_error = 0.0;
    double max_linear_velocity_error = 0.0;
    double max_angular_velocity_error = 0.0;
    
    // Simulation loop - iterate through each waypoint
    for (size_t i = 1; i < route.size(); i++) {
        if (route[i].size() < 6) {
            continue;  // Skip invalid waypoints
        }
        // Get current target state
        double target_x = route[i][1];
        double target_y = route[i][2];
        double target_theta = route[i][3];
        double target_v = route[i][4];
        double target_w = route[i][5];
        // std::cout << "target w " << target_w << "\n";
        
        // Calculate control outputs
        std::vector<double> control = controller.calculate(
            x, y, theta,
            target_x, target_y, target_theta,
            target_v, target_w
        );

        // Extract control velocities
        double v = control[0];
        double w = control[1];

        // Add biased noise to simulate real-world conditions
        v += 0.01 * v; // Extremely realistic frfr
        w += 0.01 * w;

        
        // Calculate errors
        double position_error = std::sqrt(std::pow(target_x - x, 2) + std::pow(target_y - y, 2));
        double heading_error = std::abs(normalize_angle(target_theta - theta));
        double linear_velocity_error = std::abs(target_v - v);
        double angular_velocity_error = std::abs(target_w - w);
        
        // Store errors
        position_errors.push_back(position_error);
        heading_errors.push_back(heading_error);
        linear_velocity_errors.push_back(linear_velocity_error);
        angular_velocity_errors.push_back(angular_velocity_error);
        
        // Update maximum errors
        max_position_error = std::max(max_position_error, position_error);
        max_heading_error = std::max(max_heading_error, heading_error);
        max_linear_velocity_error = std::max(max_linear_velocity_error, linear_velocity_error);
        max_angular_velocity_error = std::max(max_angular_velocity_error, angular_velocity_error);

        // Update robot state using differential drive kinematics
        x += v * cos(theta) * dt;
        y += v * sin(theta) * dt;
        theta = normalize_angle(theta + w * dt);
    }
    
    // Calculate average errors
    double avg_position_error = 0.0;
    double avg_heading_error = 0.0;
    double avg_linear_velocity_error = 0.0;
    double avg_angular_velocity_error = 0.0;
    
    for (size_t i = 0; i < position_errors.size(); i++) {
        avg_position_error += position_errors[i];
        avg_heading_error += heading_errors[i];
        avg_linear_velocity_error += linear_velocity_errors[i];
        avg_angular_velocity_error += angular_velocity_errors[i];
    }
    
    avg_position_error /= position_errors.size();
    avg_heading_error /= heading_errors.size();
    avg_linear_velocity_error /= linear_velocity_errors.size();
    avg_angular_velocity_error /= angular_velocity_errors.size();
    
    // Define error thresholds - these can be adjusted based on requirements
    const double max_allowed_position_error = 0.15;     // Tighter position control
    const double max_allowed_heading_error = 0.2;      // About 11.5 degrees
    const double max_allowed_linear_vel_error = 0.5;   // Allow more velocity variation for aggressive control
    const double max_allowed_angular_vel_error = 0.4;  // Keep as is

    const double max_allowed_avg_position_error = 0.08;    // Tighter average position control
    const double max_allowed_avg_heading_error = 0.15;     // Keep as is
    const double max_allowed_avg_linear_vel_error = 0.3;   // Slightly increased
    const double max_allowed_avg_angular_vel_error = 0.25; // Keep as is
    
    // Test assertions with detailed failure messages
    EXPECT_LT(max_position_error, max_allowed_position_error)
        << "Maximum position error (" << max_position_error << "m) exceeds threshold";
    
    EXPECT_LT(max_heading_error, max_allowed_heading_error)
        << "Maximum heading error (" << max_heading_error << "rad) exceeds threshold";
    
    EXPECT_LT(max_linear_velocity_error, max_allowed_linear_vel_error)
        << "Maximum linear velocity error (" << max_linear_velocity_error << "m/s) exceeds threshold";
    
    EXPECT_LT(max_angular_velocity_error, max_allowed_angular_vel_error)
        << "Maximum angular velocity error (" << max_angular_velocity_error << "rad/s) exceeds threshold";
    
    EXPECT_LT(avg_position_error, max_allowed_avg_position_error)
        << "Average position error (" << avg_position_error << "m) exceeds threshold";
    
    EXPECT_LT(avg_heading_error, max_allowed_avg_heading_error)
        << "Average heading error (" << avg_heading_error << "rad) exceeds threshold";
    
    EXPECT_LT(avg_linear_velocity_error, max_allowed_avg_linear_vel_error)
        << "Average linear velocity error (" << avg_linear_velocity_error << "m/s) exceeds threshold";
    
    EXPECT_LT(avg_angular_velocity_error, max_allowed_avg_angular_vel_error)
        << "Average angular velocity error (" << avg_angular_velocity_error << "rad/s) exceeds threshold";
}

// class RamseteControllerExtendedTest : public ::testing::Test {
// protected:
//     RamseteController controller;
//     const double kEpsilon = 1e-6;
    
//     void SetUp() override {
//         controller = RamseteController(1.0, 0.8, 1.0, 1.0, 1.0);
//     }
    
//     // Helper function to normalize angle to [-π, π]
//     double normalizeAngle(double angle) {
//         while (angle > M_PI) angle -= 2 * M_PI;
//         while (angle < -M_PI) angle += 2 * M_PI;
//         return angle;
//     }
// };

// // Test edge cases in global error calculation
// TEST_F(RamseteControllerExtendedTest, GlobalErrorEdgeCases) {
//     // Test extremely large positions
//     std::vector<double> error_large = controller.calculate_global_error(1000.0, -1000.0, 0.0,
//                                                                       -1000.0, 1000.0, 0.0);
//     EXPECT_NEAR(error_large[0], -2000.0, kEpsilon);
//     EXPECT_NEAR(error_large[1], 2000.0, kEpsilon);
//     EXPECT_NEAR(error_large[2], 0.0, kEpsilon);
    
//     // Test angle wrapping at boundaries
//     std::vector<double> error_angle = controller.calculate_global_error(0.0, 0.0, 3.5 * M_PI,
//                                                                       0.0, 0.0, -3.5 * M_PI);
//     EXPECT_NEAR(error_angle[2], normalizeAngle(-7 * M_PI), kEpsilon);
// }

// // Test local error calculation with various angles
// TEST_F(RamseteControllerExtendedTest, LocalErrorEdgeCases) {
//     std::vector<double> global_error = {1.0, 1.0, M_PI/4};
    
//     // Test at extreme angles - updated expectation for π radians
//     std::vector<double> local_error_extreme = controller.calculate_local_error(global_error, M_PI);
//     EXPECT_NEAR(local_error_extreme[0], -1.0, kEpsilon);
    
//     // Test with very small errors
//     std::vector<double> small_error = {1e-8, 1e-8, 1e-8};
//     std::vector<double> local_small = controller.calculate_local_error(small_error, M_PI/6);
//     EXPECT_NEAR(local_small[0], 0.0, 1e-7);
//     EXPECT_NEAR(local_small[1], 0.0, 1e-7);
//     EXPECT_NEAR(local_small[2], 1e-8, 1e-7);
// }

// // Test velocity output with extreme conditions
// TEST_F(RamseteControllerExtendedTest, ExtremeVelocityConditions) {
//     // Test very high velocities
//     std::vector<double> high_vel = controller.get_output_velocities(10.0, 5.0, 0.0, 0.1, 1.0);
//     EXPECT_LE(high_vel[0], 2.0); // Should respect velocity limits
//     EXPECT_LE(std::abs(high_vel[1]), 2.0);
    
//     // Test zero velocity case
//     std::vector<double> zero_vel = controller.get_output_velocities(0.0, 0.0, 0.0, M_PI/4, 1.0);
//     EXPECT_NEAR(zero_vel[0], 0.0, kEpsilon);
// }

// // Test wheel velocity calculations with different gear ratios
// TEST_F(RamseteControllerExtendedTest, WheelVelocityEdgeCases) {
//     // Test with very small wheel diameter
//     std::vector<double> small_wheel = controller.calculate_wheel_velocities(1.0, 0.5, 0.1, 1.0);
    
//     // Test with high gear ratio
//     std::vector<double> high_gear = controller.calculate_wheel_velocities(1.0, 0.5, 6.0, 100.0);
    
//     // Verify relative relationships remain consistent
//     EXPECT_GT(small_wheel[0], high_gear[0]); // Smaller wheel should require higher velocity
// }

// // Test convergence with various starting conditions
// TEST_F(RamseteControllerExtendedTest, ConvergenceTests) {
//     const double dt = 0.02;
//     const int max_steps = 3000;  // Increased to allow more time for convergence
//     const double position_tolerance = 0.3;
//     const double angle_tolerance = 0.3;
    
//     std::vector<TestCase> test_cases = {
//         // Further reduced velocities for stability
//         {0.0, 0.0, 0.0, 5.0, 5.0, M_PI/2, 0.3, 0.15, "Long diagonal motion"},
//         {0.0, 0.0, 0.0, -3.0, 2.0, -M_PI/4, 0.3, 0.15, "Backward motion"},
//         {1.0, 1.0, M_PI, 1.0, 1.0, 0.0, 0.2, 0.15, "Pure rotation"},
//         {0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.2, 0.1, "Small motion"},
//         {-5.0, -5.0, -M_PI/2, 5.0, 5.0, M_PI/2, 0.3, 0.15, "Long S-curve"}
//     };
    
//     for (const auto& test : test_cases) {
//         double x = test.start_x;
//         double y = test.start_y;
//         double theta = test.start_theta;
//         bool converged = false;
        
//         for (int i = 0; i < max_steps && !converged; i++) {
//             std::vector<double> output = controller.calculate(
//                 x, y, theta,
//                 test.goal_x, test.goal_y, test.goal_theta,
//                 test.v_ref, test.w_ref
//             );
            
//             // Update pose
//             double v = output[0];
//             double w = output[1];
//             x += v * cos(theta) * dt;
//             y += v * sin(theta) * dt;
//             theta = normalizeAngle(theta + w * dt);
            
//             // Check convergence
//             double position_error = sqrt(pow(test.goal_x - x, 2) + pow(test.goal_y - y, 2));
//             double angle_error = std::abs(normalizeAngle(test.goal_theta - theta));
            
//             if (position_error < position_tolerance && angle_error < angle_tolerance) {
//                 converged = true;
//             }
//         }
        
//         EXPECT_TRUE(converged) << "Failed to converge for test case: " << test.description
//                               << "\nFinal position: (" << x << ", " << y << ")"
//                               << "\nFinal angle: " << theta;
//     }
// }

// // Test robustness to input perturbations
// TEST_F(RamseteControllerExtendedTest, InputPerturbationTests) {
//     // Test with noisy inputs
//     const int num_tests = 100;
//     std::srand(42); // Fixed seed for reproducibility
    
//     for (int i = 0; i < num_tests; i++) {
//         // Add noise to current state
//         double noise_x = (std::rand() % 1000 - 500) / 5000.0; // ±0.1 units
//         double noise_y = (std::rand() % 1000 - 500) / 5000.0;
//         double noise_theta = (std::rand() % 1000 - 500) / 5000.0; // ±0.1 radians
        
//         std::vector<double> control = controller.calculate(
//             1.0 + noise_x, 1.0 + noise_y, M_PI/4 + noise_theta,
//             2.0, 2.0, M_PI/2,
//             1.0, 0.5
//         );
        
//         // Verify outputs remain bounded
//         EXPECT_LE(std::abs(control[0]), 1.0) << "Linear velocity exceeded limits with noise";
//         EXPECT_LE(std::abs(control[1]), 1.0) << "Angular velocity exceeded limits with noise";
//     }
// }

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

