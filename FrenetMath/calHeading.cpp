//
// Created by ustb on 19-7-8.
//

#include "calHeading.h"

double calFrenetHeading(std::array<double, 2> state0, std::array<double, 2> state1){
    double s0 = *(state0.cbegin());
    double rho0 = *(state0.cbegin()+1);
    double s1 = *(state1.cbegin());
    double rho1 = *(state1.cbegin()+1);

    double theta = atan2((rho1-rho0),(s1-s0));
    return theta;

}
