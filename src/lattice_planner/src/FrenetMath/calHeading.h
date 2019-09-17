//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_CALHEADING_H
#define LATTICEPLANNER_CALHEADING_H

#include <array>
#include "../config/parameters.h"

namespace lattice_planner{

double cal_FrenetHeading(lattice_planner::pose_frenet pose0, lattice_planner::pose_frenet pose1);


}


#endif //LATTICEPLANNER_CALHEADING_H
