//
// Created by ustb on 19-7-8.
//
#include <vector>
#include <math.h>
#include "calHeading.h"

double cal_FrenetHeading(std::array<double, 2> pose0, std::array<double, 2> pose1){
    double s0 = *(pose0.cbegin());
    double rho0 = *(pose0.cbegin()+1);
    double s1 = *(pose1.cbegin());
    double rho1 = *(pose1.cbegin()+1);

    double theta = atan2((rho1-rho0),(s1-s0));
    return theta;

}
