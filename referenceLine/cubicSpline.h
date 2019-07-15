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

using Vec_f=std::vector<float>;
using Poi_f=std::array<float, 2>;
using Vec_Poi=std::vector<Poi_f>;


namespace cubicSpline{

Vec_f vec_diff(Vec_f input);
Vec_f cum_sum(const Vec_f & input);

class Spline{
public:
    Vec_f x;
    Vec_f y;
    int nx;
    Vec_f h;
    Vec_f a;
    Vec_f b;
    Vec_f c;
    //Eigen::VectorXf c;
    Vec_f d;

    Spline(){};
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(Vec_f x, Vec_f y);

    float calc(float t);

    float calc_d(float t);

    float calc_dd(float t);

private:
    Eigen::MatrixXf calc_A();

    Eigen::VectorXf calc_B();

    int bisect(float t, int start, int end);
};

class Spline2D{
public:
    Spline sx;
    Spline sy;
    Vec_f s;

    Spline2D(Vec_f x, Vec_f y);
    Poi_f calc_postion(float s_t);
    float calc_curvature(float s_t);
    float calc_yaw(float s_t);

private:
    Vec_f calc_s(Vec_f x, Vec_f y);
};
}

#endif //LATTICEPLANNER_CUBICSPLINE_H
