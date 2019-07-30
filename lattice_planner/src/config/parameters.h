//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_PARAMETERS_H
#define LATTICEPLANNER_PARAMETERS_H

#include <vector>
#include <array>
#include <cmath>


namespace lattice_planner{


    struct parameters
    {
        /* data */   
    const double r_circle = 1.0;
    const double d_circle = 2.0;
    const double obstacle_inflation = 1.5;
    const double alpha1 = 100;
    const double alpha2 = 1;
    const double alpha3 = 10;
    const double alpha4 = 0.0;

    const int longitudinal_num = 5;
    const int lateral_num = 9;  // 横向采样个数
    const double longitudinal_step = 20.0;
    const double lateral_step = 0.5;
    const double lane_width = 3.75;
    const int SampleNumberOnOneSide = lateral_num / 2;    // sampling number on one side of the reference line
    const double s0 = 0.0;
    const double s_max = longitudinal_num*longitudinal_step;
    const double s_end = s0 + s_max;
    double refLineRho = lane_width * 0.5;
    std::array<double, 3> start_SRho = {{s0, refLineRho, 0.0 * M_PI / 180.0}};

    // the frenet coordinates of obstacles
    std::vector<std::vector<double> > obs = {{20, refLineRho - 1},
                                                {40, refLineRho + 1},
                                                {70, refLineRho - 1}};
    double obstacleHeading = 0.0 * M_PI / 180.0;

    std::vector<int> last_column_id = {lateral_num * (longitudinal_num - 1) + 1, lateral_num * longitudinal_num};  // 最后一列的编号

    std::vector<double > waypoints_x={{0.0, 20.0, 50, 100.0, 150.0, 220.0, 300.0, 350.0, 400.0, 430.0, 370.0, 300, 200.0}};
    std::vector<double > waypoints_y={{0.0, 70.0, 100, 120.0, 100.0, 150.0, 180.0, 150.0, 110.0, 20.0, -80.0, -80.0, -80.0}};

    double vehicle_body_fast_check_circle = 3.0;  // body collision fast check circle, the radius is 3.0 m
    double vehicle_body_envelope_circle = 1.0;    // little circles, the radius is 1.0 m  


    bool show_lattice_in_rviz=true;

    };

} //namespace latticeParameter

#endif //LATTICEPLANNER_PARAMETERS_H
