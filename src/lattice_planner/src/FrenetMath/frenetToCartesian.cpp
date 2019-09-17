//
// Created by ustb on 19-7-12.
//

#include "frenetToCartesian.h"
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

namespace lattice_planner{

geometry_msgs::PoseStamped frenet_to_cartesian(double s, 
                                                double rho, 
                                                double thetaRho, 
                                                std::vector<arc_length_parameter> coefficients){
    geometry_msgs::PoseStamped Refline_pose = poses_of_reference_line(s, coefficients);
    double x_r=Refline_pose.pose.position.x;
    double y_r=Refline_pose.pose.position.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(Refline_pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double theta_r=yaw;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = x_r + rho * cos(theta_r + M_PI / 2.0);
    pose.pose.position.y = y_r + rho * sin(theta_r + M_PI / 2.0);
    double theta = theta_r + thetaRho;
    geometry_msgs::Quaternion geo_q = tf::createQuaternionMsgFromYaw(theta);
    pose.pose.orientation = geo_q;
    // pose.yaw = theta_r + thetaRho;
    return pose;
}


geometry_msgs::PoseStamped poses_of_reference_line(double s, std::vector<arc_length_parameter> & coefficients){

    int s_id = 0;
    s_id = binary_search(coefficients, s);
    arc_length_parameter param = coefficients[s_id];
    double s_start = param.s;
    // std::vector<double> a = {{coefficients[s_id][1], coefficients[s_id][2], coefficients[s_id][3], coefficients[s_id][4]}};
    // std::vector<double > b = {{coefficients[s_id][5], coefficients[s_id][6], coefficients[s_id][7], coefficients[s_id][8]}};

    double x = param.a0 + param.a1 * s + param.a2 * s * s + param.a3 * s * s * s;
    double d_x = param.a1 + 2 * param.a2 * s + 3 * param.a3 * s * s;
    double y = param.b0 + param.b1 * s + param.b2 * s * s + param.b3 * s * s * s;
    double d_y = param.b1 + 2 * param.b2 * s + 3 * param.b3 * s * s;

    double theta = std::atan2(d_y, d_x);

    geometry_msgs::PoseStamped refLine_pose;
    refLine_pose.pose.position.x = x;
    refLine_pose.pose.position.y = y;
    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
    refLine_pose.pose.orientation = q;
    return refLine_pose;
}

int binary_search(std::vector<arc_length_parameter> &coefficients, double s) {

    int head = 0;
    int tail = coefficients.size() - 1;
    int mid = 0;

    while (head < tail){
        mid = (head + tail) / 2;
        if (coefficients[mid].s < s) head = mid+1;
        else if (s<coefficients[mid].s) tail = mid -1;
        else return mid;
    }

    if (s < coefficients[head].s)
        return head -1;
    else
        return head;
}


}