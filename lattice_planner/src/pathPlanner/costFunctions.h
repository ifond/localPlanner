//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_COSTFUNCTIONS_H
#define LATTICEPLANNER_COSTFUNCTIONS_H

#include "../FrenetMath/calKappa.h"


namespace lattice_planner{
    
class Node;
/**
 * calculate the average curvature of a motion primitive
 * @param node
 * @param next_node
 * @return
 */
double kappa_cost(const Node node, 
                const Node & next_node, 
                std::vector<std::vector<double > > & coefficients);


/**
 * calculate the reference line cost introduced by the lateral offset from the reference line
 * @param start_node
 * @param next_node
 * @param refline
 * @return
 */
double reference_line_cost(const Node start_node, const Node next_node, double & refline);


/**
 * calculate the collision risk,
 * @param start_node
 * @param next_node
 * @param obstacle
 * @return
 */
double collision_risk(const Node start_node, 
                    const Node next_node, 
                    const std::vector<std::vector<double>> & obstacle);


double total_cost(const Node node, 
                const Node next_node, 
                double & refline, 
                const std::vector<std::vector<double>> & obstacle, 
                std::vector<std::vector<double > > &coefficients);

}

#endif //LATTICEPLANNER_COSTFUNCTIONS_H
