#include "navigation/localization_manager.hpp"
#include "navigation/odometry.hpp"
#include "navigation/mcl.hpp"

// Convenience method implementations
void LocalizationManager::update()
{
    if (localization) localization->update();
}

void LocalizationManager::reset()
{
    if (localization) localization->reset();
}

Pose LocalizationManager::getPose() const
{
    return localization ? localization->getPose() : Pose();
}

void LocalizationManager::setPose(const Pose& pose)
{
    if (localization) localization->setPose(pose);
}

double LocalizationManager::getHeading() const
{
    return localization ? localization->getHeading() : 0.0;
}

double LocalizationManager::getX() const
{
    return localization ? localization->getX() : 0.0;
}

double LocalizationManager::getY() const
{
    return localization ? localization->getY() : 0.0;
}

double LocalizationManager::getTrackWidth() const
{
    return localization ? localization->getTrackWidth() : 0.0;
}

Velocity LocalizationManager::getLeftVelocity() const
{
    return localization ? localization->getLeftVelocity() : Velocity();
}

Velocity LocalizationManager::getRightVelocity() const
{
    return localization ? localization->getRightVelocity() : Velocity();
}

bool LocalizationManager::isReliable() const
{
    return localization ? localization->isReliable() : false;
}

LocalizationType LocalizationManager::getCurrentType() const
{
    return localization ? localization->getType() : LocalizationType::ODOMETRY;
}

std::string LocalizationManager::getCurrentTypeName() const
{
    return localization ? localization->getTypeName() : "None";
}

std::unordered_map<std::string, double> LocalizationManager::getDebugData() const
{
    return localization ? localization->getDebugData() : std::unordered_map<std::string, double>{};
}