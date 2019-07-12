//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_PARAMETERS_H
#define LATTICEPLANNER_PARAMETERS_H

#include <iostream>
#include <vector>
#include <array>

#define PI 3.1415926


struct costParameter {
    double r_circle = 1.0;
    double d_circle = 2.0;
    double obstacle_inflation = 1.5;
    double alpha1 = 100;
    double alpha2 = 1;
    double alpha3 = 10;
    double alpha4 = 0.0;
};


struct plannerParameter {
    double s_max = 100.0;
    int longi_num = 5;
    int lateral_num = 9;  // 横向采样个数
    double longi_step = 20.0;
    double latera_step = 0.5;
    double lane_width = 3.75;
    int LeftSampleNum = lateral_num / 2;    // sampling number on one side of the reference line
    double s0 = 0.0;
    double s_end = s0 + s_max;
    double refLineRho = lane_width * 0.5;
    std::array<double, 4> start_SRho = {{s0, refLineRho, 0.0 * PI / 180.0}};

    // the frenet coordinates of obstacles
    std::vector<std::vector<double> > obs = {{20, refLineRho - 1},
                                             {40, refLineRho + 2},
                                             {70, refLineRho + 2}};
    double obstacleHeading = 0.0 * PI / 180.0;

    std::vector<int> last_column_id = {lateral_num * (longi_num - 1) + 1, lateral_num * longi_num};  // 最后一列的编号
};


#endif //LATTICEPLANNER_PARAMETERS_H
