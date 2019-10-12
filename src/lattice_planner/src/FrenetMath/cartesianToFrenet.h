//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_CARTESIANTOFRENET_H
#define LATTICEPLANNER_CARTESIANTOFRENET_H

#include <geometry_msgs/PoseWithCovariance.h>
#include "../pathPlanner/latticePlanner.h"
#include "selfType.h"
#include <nav_msgs/Path.h>
// #include <algorithm>
// #include <functional>
// #include <memory>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "kdTree.h"


namespace lattice_planner{

/**
 * the pose of cartesian is transformed into the pose of frenet
 */
FrenetPose CartesianToFrenet(const geometry_msgs::PoseWithCovarianceStamped &pose_, 
                              nav_msgs::Path &refline_, 
                              std::vector<CubicCoefficients> &coefficients_);


/**
 * judge a cartesian pose is the left side or right side of a reference line
 */
int IsRightOrLeft(point_t p, point_t mappingp_p, point_t mapping_pNeighbour);
    
}

#endif //LATTICEPLANNER_CARTESIANTOFRENET_H
