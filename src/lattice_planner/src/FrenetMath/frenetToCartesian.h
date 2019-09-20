//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_FRENETTOCARTESIAN_H
#define LATTICEPLANNER_FRENETTOCARTESIAN_H


#include <vector>
#include <array>
#include "../config/parameters.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>


namespace lattice_planner{

geometry_msgs::PoseStamped frenet_to_cartesian(double s, 
                                        double rho, 
                                        double thetaRho, 
                                        std::vector<arc_length_parameter> &coefficients);

/**
 * input:s,output:x,y,theta
 * calculate the poses of points in the reference line
 * @param s
 * @param coefficients
 * @return
 */
geometry_msgs::PoseStamped poses_of_reference_line(double s, std::vector<arc_length_parameter>  & coefficients);

/**
 * algorithm:binary search
 * @param coefficients
 * @param s
 * @return
 */
int binary_search(std::vector<arc_length_parameter>  & coefficients, double s);

}

#endif //LATTICEPLANNER_FRENETTOCARTESIAN_H
