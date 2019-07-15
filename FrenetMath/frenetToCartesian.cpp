//
// Created by ustb on 19-7-12.
//

#include "frenetToCartesian.h"
#include <iostream>
#include <vector>

typedef std::vector<std::vector<std::vector<double > > > coefficients_type;

def frenet_to_cartesian(double s, double rho, double thetaRho, coefficients_type efficients){
    xr, yr, thetar = findEfficients(s, efficients)
    x = xr + rho * math.cos(thetar + math.pi / 2.0)
    y = yr + rho * math.sin(thetar + math.pi / 2.0)
    theta = thetar + thetaRho
    return x, y, theta
}
