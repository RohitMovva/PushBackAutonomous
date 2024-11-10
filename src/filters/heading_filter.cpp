#include "filters/heading_filter.h"
#include <cstring>

// Helper function implementations
double HeadingFilter::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double HeadingFilter::angleDifference(double a, double b) {
    return normalizeAngle(a - b);
}

HeadingFilter::HeadingFilter()
    : heading(0)
    , angular_velocity(0)
    , Q_heading(0.01)
    , Q_velocity(0.1)
    , R_heading(0.1)
    , R_velocity(0.2)
{
    // Initialize covariance matrix
    P[0][0] = 1.0;  // Initial heading uncertainty
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 1.0;  // Initial angular velocity uncertainty
}

void HeadingFilter::predict(double dt) {
    // Predict state
    heading = normalizeAngle(heading + angular_velocity * dt);
    
    // Update covariance using Jacobian
    double F[2][2] = {
        {1.0, dt},
        {0.0, 1.0}
    };
    
    // Temporary matrices for computation
    double temp[2][2];
    double new_P[2][2];
    
    // P = F*P*F' + Q
    // First compute F*P
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            temp[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j];
        }
    }
    
    // Then multiply by F' and add Q
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            new_P[i][j] = temp[i][0] * F[j][0] + temp[i][1] * F[j][1];
        }
    }
    
    // Add process noise
    new_P[0][0] += Q_heading;
    new_P[1][1] += Q_velocity;
    
    // Update P
    memcpy(P, new_P, sizeof(P));
}

void HeadingFilter::update(double measured_heading, double measured_velocity, bool has_velocity) {
    // Innovation (angle difference accounting for wraparound)
    double y_heading = angleDifference(measured_heading, heading);
    
    // If we have velocity measurement
    if (has_velocity) {
        // Two-dimensional update
        double y_velocity = measured_velocity - angular_velocity;
        
        // Innovation covariance
        double S[2][2] = {
            {P[0][0] + R_heading, P[0][1]},
            {P[1][0], P[1][1] + R_velocity}
        };
        
        // Calculate Kalman gain
        // K = P * H' * inv(S)
        double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        double K[2][2] = {
            {(P[0][0] * S[1][1] - P[0][1] * S[1][0]) / det, (-P[0][0] * S[0][1] + P[0][1] * S[0][0]) / det},
            {(P[1][0] * S[1][1] - P[1][1] * S[1][0]) / det, (-P[1][0] * S[0][1] + P[1][1] * S[0][0]) / det}
        };
        
        // Update state
        heading = normalizeAngle(heading + K[0][0] * y_heading + K[0][1] * y_velocity);
        angular_velocity += K[1][0] * y_heading + K[1][1] * y_velocity;
        
        // Update covariance
        double new_P[2][2];
        new_P[0][0] = (1 - K[0][0]) * P[0][0] - K[0][1] * P[1][0];
        new_P[0][1] = (1 - K[0][0]) * P[0][1] - K[0][1] * P[1][1];
        new_P[1][0] = -K[1][0] * P[0][0] + (1 - K[1][1]) * P[1][0];
        new_P[1][1] = -K[1][0] * P[0][1] + (1 - K[1][1]) * P[1][1];
        
        memcpy(P, new_P, sizeof(P));
    } else {
        // Single dimensional update (heading only)
        double S = P[0][0] + R_heading;
        double K[2] = {P[0][0] / S, P[1][0] / S};
        
        // Update state
        heading = normalizeAngle(heading + K[0] * y_heading);
        angular_velocity += K[1] * y_heading;
        
        // Update covariance
        double new_P[2][2];
        new_P[0][0] = (1 - K[0]) * P[0][0];
        new_P[0][1] = (1 - K[0]) * P[0][1];
        new_P[1][0] = -K[1] * P[0][0] + P[1][0];
        new_P[1][1] = -K[1] * P[0][1] + P[1][1];
        
        memcpy(P, new_P, sizeof(P));
    }
}

double HeadingFilter::getHeading() const {
    return heading;
}

double HeadingFilter::getAngularVelocity() const {
    return angular_velocity;
}

double HeadingFilter::getHeadingUncertainty() const {
    return std::sqrt(P[0][0]);
}

void HeadingFilter::reset(double initial_heading) {
    heading = initial_heading;
    angular_velocity = 0.0;
    P[0][0] = 1.0;
    P[0][1] = 0.0;
    P[1][0] = 0.0;
    P[1][1] = 1.0;
}