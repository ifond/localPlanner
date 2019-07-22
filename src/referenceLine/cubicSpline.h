//
// Created by ustb on 19-7-15.
//

#ifndef LATTICEPLANNER_CUBICSPLINE_H
#define LATTICEPLANNER_CUBICSPLINE_H



#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <Eigen/Eigen>
#include <stdexcept>

using Vec_d=std::vector<double >;
using Poi_d=std::array<double, 2>;
using Vec_Poi=std::vector<Poi_d>;


namespace cubicSpline{

/**
 * calculate the first order difference
 * @param input
 * @return
 */
Vec_d vec_diff(Vec_d input);

/**
 * calculate the sum of a vector
 * @param input
 * @return
 */
Vec_d calculate_sum(const Vec_d &input);


class Spline{
public:
    Vec_d x;
    Vec_d y;
    int nx;
    Vec_d h;
    Vec_d a;
    Vec_d b;
    Vec_d c;
    //Eigen::VectorXf c;
    Vec_d d;

    Spline(){};
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(Vec_d x, Vec_d y);

    double calc(double t);

    double calc_d(double t);

    double calc_dd(double t);

private:
    Eigen::MatrixXd calc_A();

    Eigen::VectorXd calc_B();

    int binarySearch(double t, int start, int end);
};

class Spline2D{
public:
    Spline sx;
    Spline sy;
    Vec_d s;

    Spline2D(Vec_d x, Vec_d y);
    Poi_d calculatePosition(double s_t);

    double calculateCurvature(double s_t);

    double calculateHeading(double s_t);

private:
    Vec_d calc_s(Vec_d x, Vec_d y);
};
}

#endif //LATTICEPLANNER_CUBICSPLINE_H
