#include <iostream>
#include <C:\Users\kirakum1\Desktop\Kalman\eigen\eigen-3.4.0\Eigen\Dense>

using namespace Eigen;
using namespace std;

int main()
{
    Matrix3d A; 
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    cout << "A = " << endl << A << endl;
    return 0;
}