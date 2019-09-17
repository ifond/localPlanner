//
// Created by ustb on 19-7-12.
//

#include "parameters.h"
// #include <yaml-cpp/yaml.h>

namespace lattice_planner{
    

// int setStart(){

//         int lateral_num, SampleNumberOnOneSide, longitudinal_num;
//     SampleNumberOnOneSide = lateral_num / 2;
//     double longitudinal_step, lateral_step, lane_width, s0;
//     ros::param::get("/lattice_planner/lateral_num", lateral_num);
//     ros::param::get("/lattice_planner/longitudinal_step", longitudinal_step);
//     ros::param::get("/lattice_planner/lateral_step", lateral_step);
//     ros::param::get("/lattice_planner/lane_width", lane_width);
//     ros::param::get("/lattice_planner/longitudinal_num", longitudinal_num);
//     ros::param::get("/lattice_planner/s0", s0);
//     double refLineRho=lane_width * 0.5;

//     std::array<double, 3> start_SRho = {{s0, refLineRho, 0.0 * M_PI / 180.0}};
//     geometry_msgs::PoseStamped goal;
        
//     geometry_msgs::PoseStamped start_frenet;
//     start_frenet.pose.position.x=s0;
//     start_frenet.pose.position.y =refLineRho;
//     // 只通过y即绕z的旋转角度计算四元数，用于平面小车。返回四元数
//     start_frenet.pose.orientation=  tf::createQuaternionMsgFromYaw(0.0 * M_PI / 180.0); 
//     start_frenet.header.frame_id="map";
//     start_frenet.header.stamp = ros::Time::now();
//     return 0;
// }

// template <typename T>
// void operator>>(const YAML::Node& node, T& i) {
//     i = node.as<T>();
// }

}