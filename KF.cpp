#include <iostream>

using namespace std;
/*
k: The time step.
y: The initial measurement.
innovation: The difference between the measurement and the predicted measurement based on the current state estimate.
x_hat: The updated state estimate.
P: The updated error covariance.
K: The Kalman gain ratio.
P_k: The predicted error covariance.
*/
int main() {
    
    double x_true = 72; // true temperature
    double x_hat = 68; // initial estimate
    double P = 2; // initial error estimate
    double y = 75; // initial measurement
    double R = 4; // measurement error

    // Define system parameters
    double A = 1; // system model
    double B = 0; // input model
    double H = 1; // measurement mapping
    double Q = 0; // process noise

    // Implement Kalman filter algorithm for 5 time steps
    cout << "Time Step\tMeasurement\ty - Hx_hat\tState Estimate (x_hat)\t\tP\tK\t\t\tPk" << endl;
    for (int k = 0; k < 5; k++) {
        // Prediction step
        double x_hat_kminus1 = x_hat; 
        double P_kminus1 = P; 
        double u_kminus1 = 0; 
        double x_hat_k = A * x_hat_kminus1 + B * u_kminus1; 
        double P_k = A * P_kminus1 * A + Q;

        // Update step
        double y_k = y; 
        double innovation = y_k - H * x_hat_k; 
        double K = P_k * H / (H * P_k * H + R); 
        x_hat = x_hat_k + K * innovation; 
        P = (1 - K * H) * P_k; 

        cout << k << "\t\t" << y << "\t\t" << innovation << "\t\t\t" << x_hat << "\t\t\t" << P << "\t" << K << "\t\t" << P_k << endl;
    }

    return 0;
}