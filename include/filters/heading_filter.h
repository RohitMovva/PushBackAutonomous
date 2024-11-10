#ifndef HEADING_FILTER_H
#define HEADING_FILTER_H

#include <cstdint>
#include <cmath>

/**
 * @brief Kalman filter for tracking heading and angular velocity
 * 
 * This class implements a specialized Kalman filter for heading tracking,
 * handling angle wraparound and optional angular velocity measurements.
 * The state vector contains [heading, angular_velocity].
 */
class HeadingFilter {
private:
    // State variables
    double heading;
    double angular_velocity;
    
    // Covariance matrix
    double P[2][2];
    
    // Noise parameters
    const double Q_heading;     // Process noise for heading
    const double Q_velocity;    // Process noise for angular velocity
    const double R_heading;     // Measurement noise for heading
    const double R_velocity;    // Measurement noise for angular velocity
    
    // Helper functions
    double normalizeAngle(double angle);
    double angleDifference(double a, double b);

public:
    /**
     * @brief Construct a new HeadingFilter
     * 
     * Initializes the filter state and covariance matrix with default values.
     */
    HeadingFilter();
    
    /**
     * @brief Predict the next state
     * 
     * @param dt Time step in seconds
     */
    void predict(double dt);
    
    /**
     * @brief Update the state based on measurements
     * 
     * @param measured_heading The measured heading in radians
     * @param measured_velocity Optional measured angular velocity in radians/sec
     * @param has_velocity Whether a velocity measurement was provided
     */
    void update(double measured_heading, double measured_velocity = 0.0, bool has_velocity = false);
    
    /**
     * @brief Get the current heading estimate
     * 
     * @return double Heading in radians [-π, π]
     */
    double getHeading() const;
    
    /**
     * @brief Get the current angular velocity estimate
     * 
     * @return double Angular velocity in radians/sec
     */
    double getAngularVelocity() const;
    
    /**
     * @brief Get the uncertainty in the heading estimate
     * 
     * @return double Standard deviation of heading estimate in radians
     */
    double getHeadingUncertainty() const;
    
    /**
     * @brief Reset the filter state
     * 
     * @param initial_heading Initial heading in radians (default: 0.0)
     */
    void reset(double initial_heading = 0.0);
};

#endif // HEADING_FILTER_H
