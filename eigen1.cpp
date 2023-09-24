#include <iostream>
#include <C:\Users\kirakum1\Desktop\Kalman\eigen\eigen-3.4.0\Eigen\Dense>

int main()
{
    
    Eigen::MatrixXd m(3,3);
    m << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;


    Eigen::Vector3d v(1, 2, 3);


    Eigen::Vector3d result = m * v;


    std::cout << "Result: " << result << std::endl;

    return 0;
}