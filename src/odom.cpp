#include "api.h"
#include <cmath>
#include <vector>
#include <algorithm>

struct Pose {
    double x;         // X position in inches
    double y;         // Y position in inches
    double theta;     // Heading in radians
    
    Pose(double x = 0, double y = 0, double theta = 0) 
        : x(x), y(y), theta(theta) {}
};

struct Velocity {
    double linear;    // Linear velocity in inches/sec
    double angular;   // Angular velocity in rad/sec
    
    Velocity(double linear = 0, double angular = 0)
        : linear(linear), angular(angular) {}
};

// 2D Kalman Filter for position tracking
class Kalman2D {
private:
    // State vector: [x, y, vx, vy]
    std::vector<double> x = std::vector<double>(4, 0.0);
    
    // State covariance matrix
    std::vector<std::vector<double>> P = std::vector<std::vector<double>>(4, std::vector<double>(4, 0.0));
    
    // Process noise
    std::vector<std::vector<double>> Q = std::vector<std::vector<double>>(4, std::vector<double>(4, 0.0));
    
    // Measurement noise
    std::vector<std::vector<double>> R = std::vector<std::vector<double>>(2, std::vector<double>(2, 0.0));

public:
    Kalman2D() {
        // Initialize covariance matrix
        for (int i = 0; i < 4; i++) {
            P[i][i] = 1.0;
        }
        
        // Initialize process noise
        Q[0][0] = Q[1][1] = 0.01; // Position noise
        Q[2][2] = Q[3][3] = 0.1;  // Velocity noise
        
        // Initialize measurement noise
        R[0][0] = R[1][1] = 0.1;  // Position measurement noise
    }
    
    void predict(double dt) {
        // State transition matrix
        std::vector<std::vector<double>> F = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        
        // Predict state
        std::vector<double> new_x(4, 0.0);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                new_x[i] += F[i][j] * x[j];
            }
        }
        x = new_x;
        
        // Predict covariance
        std::vector<std::vector<double>> new_P(4, std::vector<double>(4, 0.0));
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                for (int k = 0; k < 4; k++) {
                    new_P[i][j] += F[i][k] * P[k][j];
                }
            }
        }
        P = new_P;
        
        // Add process noise
        for (int i = 0; i < 4; i++) {
            P[i][i] += Q[i][i];
        }
    }
    
    void update(double meas_x, double meas_y) {
        // Measurement matrix
        std::vector<std::vector<double>> H = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };
        
        // Innovation
        std::vector<double> y = {meas_x - x[0], meas_y - x[1]};
        
        // Innovation covariance
        std::vector<std::vector<double>> S(2, std::vector<double>(2, 0.0));
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                S[i][j] = R[i][j];
                for (int k = 0; k < 4; k++) {
                    for (int l = 0; l < 4; l++) {
                        S[i][j] += H[i][k] * P[k][l] * H[j][l];
                    }
                }
            }
        }
        
        // Kalman gain
        std::vector<std::vector<double>> K(4, std::vector<double>(2, 0.0));
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 4; k++) {
                    K[i][j] += P[i][k] * H[j][k];
                }
            }
        }
        
        // Update state
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 2; j++) {
                x[i] += K[i][j] * y[j];
            }
        }
        
        // Update covariance
        std::vector<std::vector<double>> new_P(4, std::vector<double>(4, 0.0));
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                new_P[i][j] = P[i][j];
                for (int k = 0; k < 2; k++) {
                    new_P[i][j] -= K[i][k] * H[k][j];
                }
            }
        }
        P = new_P;
    }
    
    std::vector<double> getState() const {
        return x;
    }
};

// Exponential filter for velocity smoothing
class ExponentialFilter {
    double alpha;
    double prevValue = 0;
    bool initialized = false;

public:
    ExponentialFilter(double smoothingFactor) : alpha(smoothingFactor) {}
    
    double update(double newValue) {
        if (!initialized) {
            initialized = true;
            prevValue = newValue;
            return newValue;
        }
        double filtered = alpha * newValue + (1 - alpha) * prevValue;
        prevValue = filtered;
        return filtered;
    }
    
    void reset() {
        initialized = false;
        prevValue = 0;
    }
};

class Odometry {
private:
    // Constants
    const double WHEEL_DIAMETER = 2.75;  // inches
    const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
    const double GEAR_RATIO = 36.0 / 48.0;  // Output/Input
    const double TICKS_PER_ROTATION = 900.0; // For V5 encoders
    
    // Chassis measurements
    double track_width;        // Distance between left and right wheels
    double odom_wheel_offset;  // Distance from tracking center to lateral wheel
    
    // Hardware references
    pros::MotorGroup& leftDrive;
    pros::MotorGroup& rightDrive;
    pros::Encoder& lateralEncoder;
    pros::Imu& imu;
    
    // Filters
    Kalman2D positionFilter;
    ExponentialFilter leftVelocityFilter{0.7};
    ExponentialFilter rightVelocityFilter{0.7};
    ExponentialFilter headingFilter{0.8};
    
    // State variables
    Pose currentPose;
    double lastUpdateTime;
    std::vector<double> prevLeftPos;
    std::vector<double> prevRightPos;
    double prevLateralPos;
    double prevTime;
    
    // Velocity and acceleration tracking
    Velocity leftVelocity;
    Velocity rightVelocity;
    Velocity leftAccel;
    Velocity rightAccel;
    
    // Error detection thresholds
    const double MAX_VELOCITY_CHANGE = 100.0;  // inches/sec^2
    const double MAX_ENCODER_DEVIATION = 2.0;  // inches
    
    // Helper functions
    double ticksToInches(double ticks) {
        return (ticks / TICKS_PER_ROTATION) * WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    }
    
    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
    
    double getAveragePosition(const std::vector<double>& positions) {
        if (positions.empty()) return 0.0;
        return std::accumulate(positions.begin(), positions.end(), 0.0) / positions.size();
    }
    
    std::vector<double> getMotorPositionsInches(const std::vector<double>& motorPositions) {
        std::vector<double> positions;
        for (double pos : motorPositions) {
            positions.push_back(ticksToInches(pos));
        }
        return positions;
    }
    
    bool isValidEncoderReading(double newPos, double oldPos, double deltaTime) {
        double velocityChange = std::abs(newPos - oldPos) / deltaTime;
        return velocityChange <= MAX_VELOCITY_CHANGE;
    }
    
    double validateAndFilterEncoders(const std::vector<double>& positions, 
                                   const std::vector<double>& prevPositions, 
                                   double deltaTime) {
        std::vector<double> validPositions;
        for (size_t i = 0; i < positions.size(); i++) {
            if (isValidEncoderReading(positions[i], prevPositions[i], deltaTime)) {
                validPositions.push_back(positions[i]);
            }
        }
        
        if (validPositions.empty()) {
            // If all readings are invalid, use prediction from previous velocity
            return prevPositions[0] + (leftVelocity.linear * deltaTime);
        }
        
        return getAveragePosition(validPositions);
    }

public:
    Odometry(pros::MotorGroup& left, pros::MotorGroup& right, 
             pros::Encoder& lateral, pros::Imu& imuSensor,
             double chassis_track_width, double lateral_wheel_offset)
        : leftDrive(left), rightDrive(right), 
          lateralEncoder(lateral), imu(imuSensor),
          track_width(chassis_track_width),
          odom_wheel_offset(lateral_wheel_offset) {
        
        reset();
    }
    
    void reset() {
        currentPose = Pose();
        lastUpdateTime = pros::millis() / 1000.0;
        
        std::vector<double> leftPositions = leftDrive.get_positions();
        std::vector<double> rightPositions = rightDrive.get_positions();
        
        prevLeftPos = std::vector<double>(leftPositions.size(), 0);
        prevRightPos = std::vector<double>(rightPositions.size(), 0);
        prevLateralPos = 0;
        prevTime = lastUpdateTime;
        
        leftVelocityFilter.reset();
        rightVelocityFilter.reset();
        headingFilter.reset();
        
        leftDrive.tare_position();
        rightDrive.tare_position();
        lateralEncoder.reset();
        imu.tare_rotation();
    }
    
    void update() {
        double currentTime = pros::millis() / 1000.0;
        double deltaTime = currentTime - lastUpdateTime;
        
        // Get and validate encoder readings
        std::vector<double> leftPositions = getMotorPositionsInches(leftDrive.get_positions());
        std::vector<double> rightPositions = getMotorPositionsInches(rightDrive.get_positions());
        double lateralPos = ticksToInches(lateralEncoder.get_value());
        
        double currentLeftPos = validateAndFilterEncoders(leftPositions, prevLeftPos, deltaTime);
        double currentRightPos = validateAndFilterEncoders(rightPositions, prevRightPos, deltaTime);
        
        // Calculate position changes
        double deltaLeft = currentLeftPos - getAveragePosition(prevLeftPos);
        double deltaRight = currentRightPos - getAveragePosition(prevRightPos);
        double deltaLateral = lateralPos - prevLateralPos;
        
        // Update velocities with filtering
        leftVelocity.linear = leftVelocityFilter.update(deltaLeft / deltaTime);
        rightVelocity.linear = rightVelocityFilter.update(deltaRight / deltaTime);
        
        // Calculate accelerations
        leftAccel.linear = (leftVelocity.linear - (prevLeftPos[0] - getAveragePosition(prevLeftPos)) / 
                           (prevTime - lastUpdateTime)) / deltaTime;
        rightAccel.linear = (rightVelocity.linear - (prevRightPos[0] - getAveragePosition(prevRightPos)) / 
                            (prevTime - lastUpdateTime)) / deltaTime;
        
        // Get and filter heading
        double rawHeading = degreesToRadians(imu.get_rotation());
        double filteredHeading = headingFilter.update(rawHeading);
        double deltaTheta = filteredHeading - currentPose.theta;
        
        // Calculate forward displacement
        double forwardDisplacement = (deltaLeft + deltaRight) / 2.0;
        
        // Local position update
        double deltaX, deltaY;
        
        if (std::abs(deltaTheta) > 0.001) {
            // Arc motion
            double turnRadius = forwardDisplacement / deltaTheta;
            double centralChord = 2.0 * turnRadius * sin(deltaTheta / 2.0);
            double lateralChord = deltaLateral - (odom_wheel_offset * deltaTheta);
            
            deltaX = centralChord * cos(currentPose.theta + deltaTheta / 2.0) - 
                    lateralChord * sin(currentPose.theta + deltaTheta / 2.0);
            deltaY = centralChord * sin(currentPose.theta + deltaTheta / 2.0) + 
                    lateralChord * cos(currentPose.theta + deltaTheta / 2.0);
        } else {
            // Straight motion
            deltaX = forwardDisplacement * cos(currentPose.theta) - 
                    deltaLateral * sin(currentPose.theta);
            deltaY = forwardDisplacement * sin(currentPose.theta) + 
                    deltaLateral * cos(currentPose.theta);
        }
        
        // Update Kalman filter
        positionFilter.predict(deltaTime);
        positionFilter.update(currentPose.x + deltaX, currentPose.y + deltaY);
        
        // Get filtered position
        std::vector<double> state = positionFilter.getState();
        currentPose.x = state[0];
        currentPose.y = state[1];
        currentPose.theta = filteredHeading;
        
        // Update previous values
        prevLeftPos = leftPositions;
        prevRightPos = rightPositions;
        prevLateralPos = lateralPos;
        prevTime = lastUpdateTime;
        lastUpdateTime = currentTime;
    }
    

    Pose getPose() const {
        return currentPose;
    }
    
    Velocity getLeftVelocity() const {
        return leftVelocity;
    }
    
    Velocity getRightVelocity() const {
        return rightVelocity;
    }
    
    Velocity getLeftAcceleration() const {
        return leftAccel;
    }
    
    Velocity getRightAcceleration() const {
        return rightAccel;
    }
    
    // Additional helper methods
    double getHeading() const {
        return currentPose.theta;
    }
    
    double getX() const {
        return currentPose.x;
    }
    
    double getY() const {
        return currentPose.y;
    }
    
    void setPose(const Pose& newPose) {
        currentPose = newPose;
        // Update Kalman filter state
        std::vector<double>& state = positionFilter.getState();
        state[0] = newPose.x;
        state[1] = newPose.y;
        // Reset velocity states to 0
        state[2] = 0;
        state[3] = 0;
    }
    
    // Get filtered velocities
    std::pair<double, double> getFilteredVelocities() const {
        std::vector<double> state = positionFilter.getState();
        return {state[2], state[3]}; // vx, vy from Kalman filter
    }
    
    // Get estimated position uncertainty
    std::pair<double, double> getPositionUncertainty() const {
        return {std::sqrt(P[0][0]), std::sqrt(P[1][1])};
    }
    
    // Check if odometry readings are reliable
    bool isReliable() const {
        // Check if position uncertainty is within acceptable bounds
        std::pair<double, double> uncertainty = getPositionUncertainty();
        const double MAX_UNCERTAINTY = 5.0; // inches
        
        return uncertainty.first < MAX_UNCERTAINTY && 
               uncertainty.second < MAX_UNCERTAINTY && 
               std::abs(leftVelocity.linear) < MAX_VELOCITY_CHANGE && 
               std::abs(rightVelocity.linear) < MAX_VELOCITY_CHANGE;
    }
    
    // Method to temporarily disable certain sensors
    void disableSensor(const std::string& sensor) {
        if (sensor == "left") {
            leftVelocityFilter.reset();
        } else if (sensor == "right") {
            rightVelocityFilter.reset();
        } else if (sensor == "lateral") {
            // Reset lateral tracking
            prevLateralPos = lateralEncoder.get_value();
        }
    }
    
    // Get tracking wheel offsets for calibration
    std::pair<double, double> getWheelOffsets() const {
        return {track_width, odom_wheel_offset};
    }
    
    // Set new tracking wheel offsets (for dynamic calibration)
    void setWheelOffsets(double new_track_width, double new_lateral_offset) {
        track_width = new_track_width;
        odom_wheel_offset = new_lateral_offset;
    }
    
    // Get debug information for tuning
    struct DebugInfo {
        double leftVelocityRaw;
        double rightVelocityRaw;
        double leftVelocityFiltered;
        double rightVelocityFiltered;
        double headingRaw;
        double headingFiltered;
        std::vector<double> kalmanState;
        std::pair<double, double> uncertainty;
    };
    
    DebugInfo getDebugInfo() const {
        return {
            leftVelocity.linear,
            rightVelocity.linear,
            leftVelocityFilter.update(leftVelocity.linear),
            rightVelocityFilter.update(rightVelocity.linear),
            degreesToRadians(imu.get_rotation()),
            currentPose.theta,
            positionFilter.getState(),
            getPositionUncertainty()
        };
    }
};