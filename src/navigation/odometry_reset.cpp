#include "odometry_reset.hpp"
#include "utilities/logger.hpp"
#include "utilities/math/angle.hpp"
#include "utilities/math/units.hpp"
#include <algorithm>
#include <cmath>

OdometryReset::OdometryReset(pros::MotorGroup &left, pros::MotorGroup &right,
                                 pros::Rotation &lateral, pros::Imu &imuSensor,
                                 double field_width, double field_height,
                                 bool enable_heading_filter, bool enable_velocity_filters)
    : Odometry(left, right, lateral, imuSensor, enable_heading_filter, enable_velocity_filters),
      fieldWidth(field_width),
      fieldHeight(field_height),
      resetThreshold(6.0),
      resetTolerance(2.0),
      autoResetEnabled(false)
{
    // Initialize distance sensor map with null sensors
    distanceSensors[WallType::FRONT] = DistanceSensorConfig();
    distanceSensors[WallType::BACK] = DistanceSensorConfig();
    distanceSensors[WallType::LEFT] = DistanceSensorConfig();
    distanceSensors[WallType::RIGHT] = DistanceSensorConfig();
}

double OdometryReset::getDistanceReading(WallType wall) const
{
    const auto& config = distanceSensors.at(wall);
    
    if (config.sensor == nullptr || !config.isActive) {
        return -1.0;
    }
    
    // Get distance in mm and convert to inches
    double distance_mm = config.sensor->get();
    if (distance_mm == PROS_ERR) {
        return -1.0;
    }
    
    double distance_inches = Units::metersToInches(distance_mm / 1000.0); // Convert mm to inches
    
    // Validate reading
    if (distance_inches > config.maxValidDistance || distance_inches <= 0) {
        return -1.0;
    }
    
    return distance_inches;
}

bool OdometryReset::isValidDistanceReading(WallType wall, double distance) const
{
    if (distance <= 0) return false;
    
    const auto& config = distanceSensors.at(wall);
    if (distance > config.maxValidDistance) return false;
    
    // Additional validation: check if reading is reasonable given current pose
    Pose currentPose = getPose();
    double expectedDistance = getExpectedWallDistance(wall, currentPose);
    
    // Allow some tolerance for expected vs measured distance
    double tolerance = std::max(3.0, expectedDistance * 0.2); // 20% or 3 inches minimum
    
    return std::abs(distance - expectedDistance) <= tolerance;
}

double OdometryReset::getExpectedWallDistance(WallType wall, const Pose& pose) const
{
    const auto& config = distanceSensors.at(wall);
    double heading = pose.theta;
    
    switch (wall) {
        case WallType::FRONT:
            // Distance to front wall accounting for robot heading and sensor offset
            return (fieldHeight - pose.y) / cos(heading) - config.offsetFromCenter;
            
        case WallType::BACK:
            // Distance to back wall
            return pose.y / cos(heading) - config.offsetFromCenter;
            
        case WallType::LEFT:
            // Distance to left wall  
            return pose.x / sin(heading) - config.offsetFromCenter;
            
        case WallType::RIGHT:
            // Distance to right wall
            return (fieldWidth - pose.x) / sin(heading) - config.offsetFromCenter;
    }
    
    return 0.0;
}

Pose OdometryReset::calculateCorrectedPose(WallType wall, double measuredDistance, 
                                            const Pose& currentPose) const
{
    const auto& config = distanceSensors.at(wall);
    Pose correctedPose = currentPose;
    double heading = currentPose.theta;
    
    switch (wall) {
        case WallType::FRONT:
            // Correct Y position based on front wall distance
            correctedPose.y = fieldHeight - (measuredDistance + config.offsetFromCenter) * cos(heading);
            break;
            
        case WallType::BACK:
            // Correct Y position based on back wall distance  
            correctedPose.y = (measuredDistance + config.offsetFromCenter) * cos(heading);
            break;
            
        case WallType::LEFT:
            // Correct X position based on left wall distance
            correctedPose.x = (measuredDistance + config.offsetFromCenter) * sin(heading);
            break;
            
        case WallType::RIGHT:
            // Correct X position based on right wall distance
            correctedPose.x = fieldWidth - (measuredDistance + config.offsetFromCenter) * sin(heading);
            break;
    }
    
    return correctedPose;
}

bool OdometryReset::resetToWall(WallType wall, bool force_reset)
{
    double distance = getDistanceReading(wall);
    
    if (distance < 0) {
        Logger::getInstance()->logWarning("Invalid distance reading for wall reset");
        return false;
    }
    
    if (!force_reset && !isValidDistanceReading(wall, distance)) {
        Logger::getInstance()->logWarning("Distance reading failed validation for wall reset");
        return false;
    }
    
    Pose currentPose = getPose();
    Pose correctedPose = calculateCorrectedPose(wall, distance, currentPose);
    
    // Check if correction is within tolerance
    double correction_magnitude = sqrt(pow(correctedPose.x - currentPose.x, 2) + 
                                     pow(correctedPose.y - currentPose.y, 2));
    
    if (!force_reset && correction_magnitude > resetTolerance) {
        Logger::getInstance()->logWarning("Position correction too large: %.2f inches", correction_magnitude);
        return false;
    }
    
    // Apply the correction
    setPose(correctedPose);
    
    Logger::getInstance()->log("Reset to wall %d: correction of %.2f inches", 
                static_cast<int>(wall), correction_magnitude);
    
    return true;
}

bool OdometryReset::resetXPosition(WallType wall, bool force_reset)
{
    if (wall != WallType::LEFT && wall != WallType::RIGHT) {
        Logger::getInstance()->logError("X position reset requires LEFT or RIGHT wall");
        return false;
    }
    
    return resetToWall(wall, force_reset);
}

bool OdometryReset::resetYPosition(WallType wall, bool force_reset)
{
    if (wall != WallType::FRONT && wall != WallType::BACK) {
        Logger::getInstance()->logError("Y position reset requires FRONT or BACK wall");
        return false;
    }
    
    return resetToWall(wall, force_reset);
}

void OdometryReset::setAutoReset(bool enabled, double threshold)
{
    autoResetEnabled = enabled;
    resetThreshold = threshold;
    
    Logger::getInstance()->log("Auto reset %s with threshold %.2f inches", 
                enabled ? "enabled" : "disabled", threshold);
}

void OdometryReset::setResetTolerance(double tolerance)
{
    resetTolerance = tolerance;
    Logger::getInstance()->log("Reset tolerance set to %.2f inches", tolerance);
}

void OdometryReset::setSensorEnabled(WallType wall, bool enabled)
{
    auto& config = distanceSensors[wall];
    config.isActive = enabled;
    
    Logger::getInstance()->log("Distance sensor for wall %d %s", 
                static_cast<int>(wall), enabled ? "enabled" : "disabled");
}

void OdometryReset::update()
{
    // Call parent update first
    Odometry::update();
    
    // Perform automatic wall corrections if enabled
    if (autoResetEnabled) {
        for (const auto& [wall, config] : distanceSensors) {
            if (!config.isActive || config.sensor == nullptr) continue;
            
            double distance = getDistanceReading(wall);
            if (distance > 0 && distance <= resetThreshold) {
                // Only reset if we're close to a wall and reading seems valid
                if (isValidDistanceReading(wall, distance)) {
                    resetToWall(wall, false);
                }
            }
        }
    }
}

std::unordered_map<std::string, double> OdometryReset::getDebugData() const
{
    // Get base debug data
    auto debugData = Odometry::getDebugData();
    
    // Add enhanced odometry debug data
    debugData["front_distance"] = getDistanceReading(WallType::FRONT);
    debugData["back_distance"] = getDistanceReading(WallType::BACK);
    debugData["left_distance"] = getDistanceReading(WallType::LEFT);
    debugData["right_distance"] = getDistanceReading(WallType::RIGHT);
    debugData["auto_reset_enabled"] = autoResetEnabled ? 1.0 : 0.0;
    debugData["reset_threshold"] = resetThreshold;
    debugData["reset_tolerance"] = resetTolerance;
    
    return debugData;
}

OdometryReset::EnhancedDebugInfo OdometryReset::getEnhancedDebugInfo() const
{
    EnhancedDebugInfo info;
    
    info.frontDistance = getDistanceReading(WallType::FRONT);
    info.backDistance = getDistanceReading(WallType::BACK);
    info.leftDistance = getDistanceReading(WallType::LEFT);
    info.rightDistance = getDistanceReading(WallType::RIGHT);
    info.autoResetActive = autoResetEnabled;
    
    // These would need additional tracking variables to implement properly
    info.recentResetCount = 0; // TODO: Implement reset counting
    info.lastResetTime = 0.0;  // TODO: Implement reset timing
    
    return info;
}