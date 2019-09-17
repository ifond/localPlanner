// this node is to parameter the reference line using arc-length parameter s.

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include "referenceLine.h"



int main(int argc, char **argv){

    ros::init(argc, argv, "reference_arclength_node");

    ros::NodeHandle nh;

    ros::Publisher referenceLine_pub = nh.advertise<std_msgs::String>("arc_length_parameter_table", 1000);

    std::vector<std::vector<double > > coefficients = reference_line::refLine_coefficients();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        /* code */
        std_msgs::String msg;
        std::stringstream ss;
        ss<<"---------------calculate the arc-length parameter table-------------";
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());
        referenceLine_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}