//
// Created by ustb on 19-7-9.
//
#pragma once

#ifndef LATTICEPLANNER_CALKAPPA_H
#define LATTICEPLANNER_CALKAPPA_H

#include "../pathPlanner/latticePlanner.h"
#include "../curves/cubicPolynomial.h"

class Node;
/**
 * calculate the average curvature of a cubic polynomial path
 * @param node
 * @param next_node
 * @return
 */
double trajectory_kappa(const Node node, const Node next_node);


#endif //LATTICEPLANNER_CALKAPPA_H
