/*
1D Kalman Filter Algorithm
The one-dimensional Kalman filter can be described by the following equations:

1.	Predict step:
State prediction:
          x_hat = F * x
x_hat -the predicted state vector at the next time step
F - state transition matrix
x - current state vector.

Error covariance prediction:
               P_hat = F * P * F^T + Q
P_hat - predicted error covariance matrix at the next time step
P - current error covariance matrix
F^T - transpose of the state transition matrix
Q - process noise covariance matrix.

2.	Update step:
Innovation (measurement error):
               y = z - H * x_hat
y - innovation or measurement error
z - measurement vector
H - measurement matrix
x_hat - predicted state vector from the predict step.



Innovation covariance:
               S = H * P_hat * H^T + R
 S - innovation covariance matrix
 P_hat - predicted error covariance matrix from the predict step, H^T - transpose of the measurement matrix
R is the measurement noise covariance matrix.

Kalman gain:
               K = P_hat * H^T * S^-1
K - Kalman gain
P_hat - predicted error covariance matrix from the predict step
H^T - transpose of the measurement matrix
S^-1 - inverse of the innovation covariance matrix.

State update:
               x = x_hat + K * y
x - updated state vector
x_hat - predicted state vector from the predict step
K - Kalman gain
y - innovation or measurement error.

Error covariance update:
                    P = (I - K * H) * P_hat
P - updated error covariance matrix 
I - identity matrix
 K - Kalman gain
 H - measurement matrix
P_hat - predicted error covariance matrix from the predict step.

*/

#include <iostream>
#include <cmath>
#include <C:\Users\kirakum1\Desktop\Kalman\eigen\eigen-3.4.0\Eigen\Dense>
// #include <eigen\eigen-3.4.0\Eigen\Dense>
// eigen\eigen-3.4.0\Eigen\Dense


using namespace std;
using namespace Eigen;
int main()
{
     cout<<" ------------------"<<endl;
    // Definimg the state vector, measurement vector, and initial estimate
    VectorXd x(2); // state vector [position, velocity]
    x << 0, 0; // initial state

    cout<<"state vector [position, velocity] X="<<x<<endl;
    cout<<" ------------------"<<endl;


    double dt = 0.1; // time step
    VectorXd z(1); // measurement vector [position]
    z << 1;
    cout<<"measurement vector [position] Z="<<z<<endl;
    cout<<" ------------------"<<endl;


    // transition matrix
    MatrixXd F(2, 2);
    F << 1, dt,
         0, 1;

     cout<<"transition matrix F="<<F<<endl;
     cout<<" ------------------"<<endl;

     

    //measurement matrix
    MatrixXd H(1, 2);
    H << 1, 0;
    cout<<"measurement matrix H="<<H<<endl;
    cout<<" ------------------"<<endl;


    //  process noise covariance matrix
    MatrixXd Q(2, 2);
    Q << 1e-4, 0,
         0, 1e-4;
     cout<<"process noise covariance matrix  Q="<<Q<<endl;
     cout<<" ------------------"<<endl;


    //  measurement noise covariance matrix
    MatrixXd R(1, 1);
    R << 1;
    cout<<"measurement noise covariance matrix R="<<R<<endl;


    // initialize the error covariance matrix 
    MatrixXd P(2, 2);
    P << 1, 0,
         0, 1;
     cout<<"error covariance matrix P="<<P<<endl;
     

    //identity matrix
     MatrixXd I = MatrixXd::Identity(2, 2);
     cout<<"Identity Matrix I="<<I<<endl;

    // Running the kalman filter for each time step
    for (int i = 0; i < 5; i++) {
        // Predict the state estimate and error covariance
        x = F * x;
        P = F * P * F.transpose() + Q;

        // Updating the state estimate and error covariance based on the measurement step
        VectorXd y = z - H * x;
        MatrixXd S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();  //kalman gain
        x = x + K * y;
        P = (I - K * H) * P;

        // printing  the state estimate at each time step
        cout << "X = " << x.transpose() << endl;
    }

    return 0;
}