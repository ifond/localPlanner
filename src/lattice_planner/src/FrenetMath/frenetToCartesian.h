//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_FRENETTOCARTESIAN_H
#define LATTICEPLANNER_FRENETTOCARTESIAN_H


#include <vector>
#include <array>
#include "selfType.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>


namespace lattice_planner{

/**
 * the frenet pose is transformed into the cartesian pose
 */
geometry_msgs::PoseStamped frenetToCartesian(FrenetPose frtPose,
                                            std::vector<CubicCoefficients> &coefficients);

/**
 * input:s,output:x,y,theta
 * calculate the poses of points in the reference line
 * @param s
 * @param coefficients
 * @return
 */
geometry_msgs::PoseStamped poses_of_reference_line(double s, std::vector<CubicCoefficients>  & coefficients);

/**
 * algorithm:binary search
 * @param coefficients
 * @param s
 * @return
 */
int binary_search(std::vector<CubicCoefficients> & coefficients, double s);

}

#endif //LATTICEPLANNER_FRENETTOCARTESIAN_H
