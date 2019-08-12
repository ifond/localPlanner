//
// Created by ustb on 19-7-12.
//

#include "frenetToCartesian.h"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

namespace lattice_planner{

std::vector<double> frenet_to_cartesian(double s, 
                                        double rho, 
                                        double thetaRho, 
                                        std::vector<std::vector<double > > efficients){
    std::array<double,3 > pose = poses_of_reference_line(s, efficients);
    double x_r=pose[0];
    double y_r=pose[1];
    double theta_r=pose[2];
    double x = x_r + rho * cos(theta_r + M_PI / 2.0);
    double y = y_r + rho * sin(theta_r + M_PI / 2.0);
    double theta = theta_r + thetaRho;
    std::vector<double> cartesian_pose = {{x, y, theta}};
    return cartesian_pose;
}

}