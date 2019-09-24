//
// Created by ustb on 19-7-12.
//
//  input:way-points
//  output:coefficients of the arc-length parameterized reference line
//  x=a[0]+a[1]*s + a[2]*s^2 + a[3]*s^3;
//  y=b[0]+b[1]*s + b[2]*s^2 + b[3]*s^3;
//  d_x = a_vec[1] + 2 * a_vec[2] * s + 3 * a_vec[3] * s ** 2
//  d_y = b_vec[1] + 2 * b_vec[2] * s + 3 * b_vec[3] * s ** 2

#ifndef LATTICEPLANNER_PARAMETERS_H
#define LATTICEPLANNER_PARAMETERS_H

#include <vector>
#include <array>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


namespace lattice_planner{

struct pose_frenet{
    double s;
    double rho;
    double heading=0.0;
};

// struct PoseCartesian{
//     double x;
//     double y;
//     double yaw;
// };

struct arc_length_parameter{
    double s;
    double a0, a1, a2, a3;
    double b0, b1, b2, b3;
};

// template <typename T>
// void operator>>(const YAML::Node& node, T& i); 

struct parameters
{
    /* data */
    double r_circle = 1.0;
    double d_circle = 2.0;
    double obstacle_inflation = 1.5;
    double alpha1 = 100;
    double alpha2 = 1;
    double alpha3 = 10;
    double alpha4 = 0.0;
    int longitudinal_num = 5;
    int lateral_num = 9;  // 横向采样个数
    double longitudinal_step = 20.0;
    double lateral_step = 0.5;
    double lane_width = 3.75;
    double s0 = 0.0;

};


int setStart();



} //namespace latticeParameter

#endif //LATTICEPLANNER_PARAMETERS_H
