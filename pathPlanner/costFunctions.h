//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_COSTFUNCTIONS_H
#define LATTICEPLANNER_COSTFUNCTIONS_H

#include "../FrenetMath/calKappa.h"



class Node;
/**
 * calculate the average curvature of a motion primitive
 * @param node
 * @param next_node
 * @return
 */
double kappa_cost(Node & node, Node & next_node);


/**
 * calculate the reference line cost introduced by the lateral offset from the reference line
 * @param start_node
 * @param next_node
 * @param refline
 * @return
 */
double reference_line_cost(Node start_node, Node next_node, double & refline);


/**
 * calculate the collision risk,
 * @param start_node
 * @param next_node
 * @param obstacle
 * @return
 */
double collision_risk(Node start_node, Node next_node, const std::vector<std::vector<double>>& obstacle);


double total_cost(Node node, Node next_node, double & refline, const std::vector<std::vector<double>>& obstacle);


#endif //LATTICEPLANNER_COSTFUNCTIONS_H
