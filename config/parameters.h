//
// Created by ustb on 19-7-12.
//

#ifndef LATTICEPLANNER_PARAMETERS_H
#define LATTICEPLANNER_PARAMETERS_H

#include <vector>
#include <array>


namespace latticeParameter{
    #define PI 3.1415926

    struct costParameter {
        const double r_circle = 1.0;
        const double d_circle = 2.0;
        const double obstacle_inflation = 1.5;
        const double alpha1 = 100;
        const double alpha2 = 1;
        const double alpha3 = 10;
        const double alpha4 = 0.0;
    };


    struct plannerParameter {

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
        std::array<double, 4> start_SRho = {{s0, refLineRho, 0.0 * PI / 180.0}};

        // the frenet coordinates of obstacles
        std::vector<std::vector<double> > obs = {{20, refLineRho - 1},
                                                 {40, refLineRho + 2},
                                                 {70, refLineRho + 2}};
        double obstacleHeading = 0.0 * PI / 180.0;

        std::vector<int> last_column_id = {lateral_num * (longitudinal_num - 1) + 1, lateral_num * longitudinal_num};  // 最后一列的编号
    };

} //namespace latticeParameter

#endif //LATTICEPLANNER_PARAMETERS_H
