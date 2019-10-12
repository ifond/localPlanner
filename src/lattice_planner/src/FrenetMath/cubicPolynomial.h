//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_CUBICPOLYNOMIAL_H
#define LATTICEPLANNER_CUBICPOLYNOMIAL_H


#include <array>
#include <vector>
#include "calHeading.h"
#include "selfType.h"


namespace lattice_planner{

/**
 * a struct:store cubic polynomial's coefficients
 */
struct cubic_coeffi
{
    /* data */
    double c0,c1,c2,c3;
};


class CubicPolynomial {
public:
    CubicPolynomial() = default;
    virtual ~CubicPolynomial() = default;
    CubicPolynomial(const lattice_planner::FrenetPose & start,
                    const lattice_planner::FrenetPose & end);


    /**
     * x0 is the value when f(x = 0);
     * dx0 is the value when f'(x = 0);
     * ddx0 is the value when f''(x = 0);
     * f(x = param) = x1
     */

    std::vector<lattice_planner::FrenetPose> computeFrenetCoordinates();

    void print_coefficients();

private:
    cubic_coeffi coeffients_;
    lattice_planner::FrenetPose start_condition_;
    lattice_planner::FrenetPose end_condition_;

    void computeCoefficients();

};

}

#endif //LATTICEPLANNER_CUBICPOLYNOMIAL_H
