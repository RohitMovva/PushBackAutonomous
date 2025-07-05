#ifndef KALMAN_2D_H
#define KALMAN_2D_H

#include <vector>
#include <cmath>

/**
 * @brief 2D Kalman Filter for position tracking
 * 
 * This class implements a Kalman filter for tracking position and velocity
 * in 2D space. The state vector contains [x, y, vx, vy].
 */
class Kalman2D {
private:
    // State vector: [x, y, vx, vy]
    std::vector<double> x;
    
    // State covariance matrix
    std::vector<std::vector<double>> P;
    
    // Process noise
    std::vector<std::vector<double>> Q;
    
    // Measurement noise
    std::vector<std::vector<double>> R;

public:
    /**
     * @brief Construct a new Kalman2D filter
     * 
     * Initializes the state vector, covariance matrices, and noise parameters.
     */
    Kalman2D();

    /**
     * @brief Predict the next state
     * 
     * @param dt Time step in seconds
     */
    void predict(double dt);

    /**
     * @brief Update the state based on measurements
     * 
     * @param meas_x Measured x position
     * @param meas_y Measured y position
     */
    void update(double meas_x, double meas_y);

    /**
     * @brief Get the current state estimate
     * 
     * @return std::vector<double> State vector [x, y, vx, vy]
     */
    std::vector<double> getState() const;

    /**
     * @brief Get the current state covariance matrix
     * 
     * @return std::vector<std::vector<double>> Covariance matrix
     */
    std::vector<std::vector<double>> getCovarianceMatrix() const;
};

#endif // KALMAN_2D_H