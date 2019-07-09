//
// Created by ustb on 19-7-9.
//

#ifndef LATTICEPLANNER_CALKAPPA_H
#define LATTICEPLANNER_CALKAPPA_H


#include <array>
#include "../pathPlanner/latticePlanner.h"
#include "../curves/cubicPolynomial.h"


/**
 * calculate the average curvature of a cubic polynomial path
 * @param start_node
 * @param next_node
 * @return
 */
double trajectory_kappa(Node start_node, Node next_node);


#endif //LATTICEPLANNER_CALKAPPA_H
