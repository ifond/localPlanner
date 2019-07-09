#include <iostream>
#include <Eigen/Dense>
#include <array>
#include "curves/cubicPolynomial.h"

using Eigen::MatrixXd;


int main() {
    std::cout << "Hello, World!" << std::endl;
    MatrixXd m(2, 2);
    std::cout<<m<<std::endl;
    std::array<double, 3> start= {{0.0, 0.0, 0.0}};
    std::array<double, 3> end = {{20.0, 2.0, 0.0}};
    CubicPolynomial cubic(start, end);
    cubic.computeFrenetCoordinates();
    cout<<1.0/2.0<<endl;
    return 0;
}