#ifndef I_LOCALIZATION_H
#define I_LOCALIZATION_H

#include <string>
#include <unordered_map>

/**
 * @brief Represents a 2D pose (position and orientation)
 */
struct Pose
{
    double x;     ///< X position in inches
    double y;     ///< Y position in inches
    double theta; ///< Heading in radians

    Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};

/**
 * @brief Particle structure for Monte Carlo Localization
 */
struct Particle
{
    Pose pose;
    double weight;
    
    Particle(double x = 0, double y = 0, double theta = 0, double w = 1.0)
        : pose(x, y, theta), weight(w) {}
        
    void predict(const Pose& motion, double motionNoise);
    void updateWeight(double likelihood);
};

/**
 * @brief Represents linear and angular velocity
 */
struct Velocity
{
    double linear;  ///< Linear velocity in inches/sec
    double angular; ///< Angular velocity in rad/sec

    Velocity(double linear = 0, double angular = 0) : linear(linear), angular(angular) {}
};

/**
 * @brief Localization type enumeration
 */
enum class LocalizationType 
{
    ODOMETRY,
    DISTANCE_RESET_ODOMETRY,
    MONTE_CARLO
};

/**
 * @brief Abstract base class for all localization implementations
 */
class ILocalization
{
public:
    virtual ~ILocalization() = default;
    
    // Core interface that all implementations MUST provide
    virtual void update() = 0;
    virtual void reset() = 0;
    virtual Pose getPose() const = 0;
    virtual void setPose(const Pose& newPose) = 0;
    virtual double getHeading() const = 0;
    virtual double getX() const = 0;
    virtual double getY() const = 0;
    virtual bool isReliable() const = 0;
    
    // Velocity interface (needed for trajectory following)
    virtual Velocity getLeftVelocity() const = 0;
    virtual Velocity getRightVelocity() const = 0;
    
    // Implementation identification
    virtual LocalizationType getType() const = 0;
    virtual std::string getTypeName() const = 0;
    
    // Debug interface - each implementation returns its own relevant data
    virtual std::unordered_map<std::string, double> getDebugData() const { return {}; }
};

#endif // I_LOCALIZATION_H