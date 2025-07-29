#ifndef LOCALIZATION_MANAGER_H
#define LOCALIZATION_MANAGER_H

#include "navigation/i_localization.hpp"
#include "utilities/logger.hpp"
#include <memory>
#include <string>

// Forward declarations to avoid circular includes
class Odometry;
class MCL;

/**
 * @brief Localization factory that creates localization implementations
 * 
 * Provides factory methods for creating different localization implementations.
 * Acts as a Factory pattern for localization objects.
 */
class LocalizationManager
{
private:
    // Factory methods for different implementations
    template<typename... Args>
    static std::unique_ptr<ILocalization> createOdometry(Args&&... args);
    
    template<typename... Args>
    static std::unique_ptr<ILocalization> createMCL(Args&&... args);
    
public:
    /**
     * @brief Create a localization implementation of the specified type
     * @param type The type of localization to create
     * @param args Arguments specific to the localization implementation
     * @return Unique pointer to the created localization object, or nullptr if creation failed
     */
    template<typename... Args>
    static std::unique_ptr<ILocalization> create(LocalizationType type, Args&&... args);
    
    /**
     * @brief Create a localization implementation and transfer state from existing one
     * @param type New localization type
     * @param existingLocalization Existing localization to preserve state from
     * @param args Constructor arguments for the new implementation
     * @return Unique pointer to the created localization object with preserved state
     */
    template<typename... Args>
    static std::unique_ptr<ILocalization> createWithStateTransfer(
        LocalizationType type, 
        const ILocalization* existingLocalization,
        Args&&... args);
    
    /**
     * @brief Get the type name as string for debugging
     * @param type The localization type
     * @return String representation of the type
     */
    static std::string getTypeName(LocalizationType type);
    
    /**
     * @brief Check if a type is supported
     * @param type The localization type to check
     * @return true if the type can be created, false otherwise
     */
    static bool isTypeSupported(LocalizationType type);
};

// Template implementation (must be in header for template instantiation)
template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::create(LocalizationType type, Args&&... args)
{
    switch (type) {
        case LocalizationType::ODOMETRY:
            return createOdometry(std::forward<Args>(args)...);
        case LocalizationType::MONTE_CARLO:
            return createMCL(std::forward<Args>(args)...);
        default:
            return nullptr;
    }
}

template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::createWithStateTransfer(
    LocalizationType type, 
    const ILocalization* existingLocalization,
    Args&&... args)
{
    // Create new implementation
    auto newLocalization = create(type, std::forward<Args>(args)...);
    
    // Transfer state if both objects exist
    if (newLocalization && existingLocalization) {
        newLocalization->setPose(existingLocalization->getPose());
    }
    
    return newLocalization;
}

template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::createOdometry(Args&&... args)
{   
    if constexpr (std::is_constructible_v<Odometry, Args...>) {
        return std::make_unique<Odometry>(std::forward<Args>(args)...);
    } else {
        return nullptr;
    }
}

template<typename... Args>
std::unique_ptr<ILocalization> LocalizationManager::createMCL(Args&&... args)
{
    if constexpr (std::is_constructible_v<MCL, Args...>) {
        return std::make_unique<MCL>(std::forward<Args>(args)...);
    } else {
        return nullptr;
    }
}

#endif // LOCALIZATION_MANAGER_H