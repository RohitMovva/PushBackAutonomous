#ifndef LOCALIZATION_MANAGER_H
#define LOCALIZATION_MANAGER_H

#include "navigation/i_localization.hpp"
#include <memory>
#include <string>

// Forward declarations to avoid circular includes
class OdometryLocalization;
class MCLLocalization;

/**
 * @brief Localization manager that handles switching between implementations
 * 
 * Provides a unified interface for different localization methods while allowing
 * runtime switching with state preservation. Acts as a Strategy pattern coordinator.
 */
class LocalizationManager
{
private:
    std::unique_ptr<ILocalization> localization;
    
    // Factory methods for different implementations
    template<typename... Args>
    std::unique_ptr<ILocalization> createOdometry(Args&&... args);
    
    template<typename... Args>
    std::unique_ptr<ILocalization> createMCL(Args&&... args);
    
public:
    /**
     * @brief Default constructor - creates uninitialized manager
     */
    LocalizationManager() = default;
    
    /**
     * @brief Constructor that creates the specified localization type
     * @param type The type of localization to use
     * @param args Arguments specific to the localization implementation
     */
    template<typename... Args>
    LocalizationManager(LocalizationType type, Args&&... args)
    {
        switchTo(type, std::forward<Args>(args)...);
    }
    
    /**
     * @brief Switch localization implementation with state preservation
     * @param type New localization type
     * @param args Constructor arguments for the new implementation
     * @return true if switch was successful, false otherwise
     */
    template<typename... Args>
    bool switchTo(LocalizationType type, Args&&... args);
    
    /**
     * @brief Check if manager has been initialized
     */
    bool isInitialized() const { return localization != nullptr; }
    
    // Convenience methods that delegate to current implementation
    // These match exactly what your Robot class expects
    void update();
    void reset();
    Pose getPose() const;
    void setPose(const Pose& pose);
    double getHeading() const;
    double getX() const;
    double getY() const;
    double getTrackWidth() const;
    Velocity getLeftVelocity() const;
    Velocity getRightVelocity() const;
    bool isReliable() const;
    
    // Access to underlying implementation for specific features
    ILocalization* getImplementation() const { return localization.get(); }
    
    // Type checking
    LocalizationType getCurrentType() const;
    std::string getCurrentTypeName() const;
    
    // Debug data access
    std::unordered_map<std::string, double> getDebugData() const;
    
    /**
     * @brief Cast to specific implementation type for accessing specific methods
     * @tparam T The specific localization type to cast to
     * @return Pointer to the specific type, or nullptr if cast fails
     */
    template<typename T>
    T* getAs() const
    {
        return dynamic_cast<T*>(localization.get());
    }
};

// Template implementation (must be in header for template instantiation)
template<typename... Args>
bool LocalizationManager::switchTo(LocalizationType type, Args&&... args)
{
    // Preserve current pose if switching
    Pose currentPose;
    if (localization) {
        currentPose = localization->getPose();
    }
    
    // Create new implementation
    std::unique_ptr<ILocalization> newLocalization;
    
    switch (type) {
        case LocalizationType::ODOMETRY:
            newLocalization = createOdometry(std::forward<Args>(args)...);
            break;
        case LocalizationType::MONTE_CARLO:
            newLocalization = createMCL(std::forward<Args>(args)...);
            break;
        // Add more cases as needed
        default:
            return false;
    }
    
    if (newLocalization) {
        localization = std::move(newLocalization);
        localization->setPose(currentPose);
        return true;
    }
    return false;
}

template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::createOdometry(Args&&... args)
{
    return std::make_unique<OdometryLocalization>(std::forward<Args>(args)...);
}

template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::createMCL(Args&&... args)
{
    return std::make_unique<MCLLocalization>(std::forward<Args>(args)...);
}

#endif // LOCALIZATION_MANAGER_H