//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_FRENETTOCARTESIAN_H
#define LATTICEPLANNER_FRENETTOCARTESIAN_H


#include "../referenceLine/referenceLine.h"

std::vector<double> frenet_to_cartesian(double s, double rho, double thetaRho, std::vector<std::vector<std::vector<double > > > efficients);


#endif //LATTICEPLANNER_FRENETTOCARTESIAN_H
