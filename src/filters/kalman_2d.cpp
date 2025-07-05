#include "filters/kalman_2d.hpp"

Kalman2D::Kalman2D() 
    : x(std::vector<double>(4, 0.0))
    , P(std::vector<std::vector<double>>(4, std::vector<double>(4, 0.0)))
    , Q(std::vector<std::vector<double>>(4, std::vector<double>(4, 0.0)))
    , R(std::vector<std::vector<double>>(2, std::vector<double>(2, 0.0)))
{
    // Initialize covariance matrix
    for (int i = 0; i < 4; i++) {
        P[i][i] = 1.0;
    }
    
    // Initialize process noise
    Q[0][0] = Q[1][1] = 0.01;  // Position noise
    Q[2][2] = Q[3][3] = 0.1;   // Velocity noise
    
    // Initialize measurement noise
    R[0][0] = R[1][1] = 0.1;   // Position measurement noise
}

void Kalman2D::predict(double dt) {
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

void Kalman2D::update(double meas_x, double meas_y) {
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

std::vector<double> Kalman2D::getState() const {
    return x;
}

std::vector<std::vector<double>> Kalman2D::getCovarianceMatrix() const {
    return P;
}
