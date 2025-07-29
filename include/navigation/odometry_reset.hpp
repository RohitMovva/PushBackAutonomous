#ifndef ODOMETRY_RESET_H
#define ODOMETRY_RESET_H

#include "odometry.hpp"
#include "api.h"
#include "i_localization.hpp"
#include <unordered_map>
#include <string>

/**
 * @brief Wall types for distance sensor reset operations
 */
enum class WallType {
    FRONT,    // Robot facing forward toward wall
    BACK,     // Robot facing backward toward wall  
    LEFT,     // Robot's left side toward wall
    RIGHT     // Robot's right side toward wall
};

/**
 * @brief Distance sensor configuration for wall detection
 */
struct DistanceSensorConfig {
    pros::Distance* sensor;
    double offsetFromCenter;  // Distance from robot center to sensor (inches)
    double maxValidDistance;  // Maximum distance considered valid (inches)
    bool isActive;           // Whether this sensor is currently enabled
    
    DistanceSensorConfig(pros::Distance* s = nullptr, double offset = 0.0, 
                        double maxDist = 24.0, bool active = true)
        : sensor(s), offsetFromCenter(offset), maxValidDistance(maxDist), isActive(active) {}
};

/**
 * @brief Enhanced Odometry class with distance sensor reset capabilities
 * 
 * Extends the base Odometry class to provide automatic position correction
 * using distance sensors when the robot approaches known walls or field elements.
 */
class OdometryReset : public Odometry
{
private:
    // Distance sensors for each wall direction
    std::unordered_map<WallType, DistanceSensorConfig> distanceSensors;
    
    // Field boundaries (set these based on your field dimensions)
    double fieldWidth;   // Total field width (X-axis)
    double fieldHeight;  // Total field height (Y-axis)
    
    // Reset thresholds and validation
    double resetThreshold;        // Distance threshold to trigger reset (inches)
    double resetTolerance;        // Tolerance for position correction (inches)
    bool autoResetEnabled;        // Enable automatic resets during update()
    
    // Helper methods
    double getDistanceToWall(WallType wall, const Pose& currentPose) const;
    bool isValidDistanceReading(WallType wall, double distance) const;
    Pose calculateCorrectedPose(WallType wall, double measuredDistance, const Pose& currentPose) const;
    double getExpectedWallDistance(WallType wall, const Pose& currentPose) const;

public:
    /**
     * @brief Construct Enhanced Odometry with distance sensor reset capability
     * 
     * @param left Left drive motors
     * @param right Right drive motors  
     * @param lateral Lateral tracking encoder
     * @param imuSensor IMU sensor
     * @param field_width Total field width in inches
     * @param field_height Total field height in inches
     * @param enable_heading_filter Enable heading filter
     * @param enable_velocity_filters Enable velocity filters
     */
    OdometryReset(pros::MotorGroup &left, pros::MotorGroup &right,
                    pros::Rotation &lateral, pros::Imu &imuSensor,
                    double field_width = 144.0, double field_height = 144.0,
                    bool enable_heading_filter = false,
                    bool enable_velocity_filters = false);

    /**
     * @brief Manually reset position based on distance sensor reading
     * 
     * @param wall Which wall to reset against
     * @param force_reset Force reset even if reading seems invalid
     * @return true if reset was successful, false otherwise
     */
    bool resetToWall(WallType wall, bool force_reset = false);

    /**
     * @brief Reset X-axis position based on left or right wall
     * 
     * @param wall Must be WallType::LEFT or WallType::RIGHT
     * @param force_reset Force reset even if reading seems invalid
     * @return true if reset was successful, false otherwise
     */
    bool resetXPosition(WallType wall, bool force_reset = false);

    /**
     * @brief Reset Y-axis position based on front or back wall
     * 
     * @param wall Must be WallType::FRONT or WallType::BACK  
     * @param force_reset Force reset even if reading seems invalid
     * @return true if reset was successful, false otherwise
     */
    bool resetYPosition(WallType wall, bool force_reset = false);

    /**
     * @brief Enable/disable automatic position correction during update()
     * 
     * @param enabled Whether to automatically correct position
     * @param threshold Distance threshold to trigger auto-correction (inches)
     */
    void setAutoReset(bool enabled, double threshold = 6.0);

    /**
     * @brief Set the reset tolerance for position corrections
     * 
     * @param tolerance Maximum allowed correction in inches
     */
    void setResetTolerance(double tolerance);

    /**
     * @brief Enable/disable a specific distance sensor
     * 
     * @param wall Which sensor to enable/disable
     * @param enabled New enabled state
     */
    void setSensorEnabled(WallType wall, bool enabled);

    /**
     * @brief Get current distance reading from specified sensor
     * 
     * @param wall Which sensor to read
     * @return Distance in inches, or -1 if sensor unavailable/invalid
     */
    double getDistanceReading(WallType wall) const;

    /**
     * @brief Override update to include automatic wall corrections
     */
    void update() override;

    /**
     * @brief Get debug information including distance sensor data
     */
    std::unordered_map<std::string, double> getDebugData() const override;

    /**
     * @brief Get extended debug info specific to enhanced odometry
     */
    struct EnhancedDebugInfo {
        double frontDistance;
        double backDistance; 
        double leftDistance;
        double rightDistance;
        bool autoResetActive;
        int recentResetCount;
        double lastResetTime;
    };
    
    EnhancedDebugInfo getEnhancedDebugInfo() const;
};

#endif // ODOMETRY_RESET_H