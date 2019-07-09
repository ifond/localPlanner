//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_CUBICPOLYNOMIAL_H
#define LATTICEPLANNER_CUBICPOLYNOMIAL_H


#include <array>
#include <string>
#include <Eigen/Dense>
#include <math.h>
#include <vector>
#include "../FrenetMath/calHeading.h"

using namespace std;


class CubicPolynomial {
public:
    CubicPolynomial() = default;
    virtual ~CubicPolynomial() = default;
    CubicPolynomial(const std::array<double, 3> & start,
                    const std::array<double, 3> & end);


    /**
     * x0 is the value when f(x = 0);
     * dx0 is the value when f'(x = 0);
     * ddx0 is the value when f''(x = 0);
     * f(x = param) = x1
     */

    std::vector<std::vector<double >> computeFrenetCoordinates();


private:
    std::array<double, 4> coeffients_ = {{0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_ = {{0.0, 0.0, 0.0}};
    std::array<double, 3> end_condition_ = {{0.0, 0.0, 0.0}};
    std::vector<double> s;
    std::vector<double> rho;
    std::vector<double> theta;

    void computeCoefficients();

};



#endif //LATTICEPLANNER_CUBICPOLYNOMIAL_H
