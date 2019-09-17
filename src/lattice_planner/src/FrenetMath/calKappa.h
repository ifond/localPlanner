//
// Created by ustb on 19-7-9.
//

#ifndef LATTICEPLANNER_CALKAPPA_H
#define LATTICEPLANNER_CALKAPPA_H

#include "../pathPlanner/latticePlanner.h"
#include "../curves/cubicPolynomial.h"
#include "../FrenetMath/frenetToCartesian.h"


namespace lattice_planner{
class Node;
/**
 * calculate the average curvature of a cubic polynomial path
 * @param node
 * @param next_node
 * @return
 */
double trajectory_kappa(const Node node, 
                        const Node next_node, 
                        std::vector<arc_length_parameter> & coefficients);

}

#endif //LATTICEPLANNER_CALKAPPA_H
