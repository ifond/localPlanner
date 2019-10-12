#include <iostream>
#include <array>
#include <stack>
#include <vector>
#include <fstream>
#include "pathPlanner/latticePlanner.h"
// #include "curves/cubicPolynomial.h"
#include "FrenetMath/selfType.h"
// #include "FrenetMath/frenetToCartesian.h"
// #include "../include/lattice_planner/lattice_planner.h"
#include <ros/ros.h>
// #include <nav_msgs/Path.h>
// #include <geometry_msgs/PoseWithCovariance.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/PoseStamped.h>

// #include <nav_msgs/Path.h>
// #include "FrenetMath/cartesianToFrenet.h"


std::vector<lattice_planner::CubicCoefficients> loadCoefficients(){
	std::ifstream readFile("/home/ustb/coefficients_test.csv");
    std::string lineStr;

	std::vector<lattice_planner::CubicCoefficients> coefficients;
	// std::vector<std::vector<double>> coefficients;
	while (getline(readFile, lineStr))
	{

		// 存成二维表结构
		std::stringstream ss(lineStr);
		std::string str_data;
        double d_data;
		std::vector<double> lineArray;
		// 按照逗号分隔
		while (getline(ss, str_data, ',')){
            double d_data=std::atof(str_data.c_str());
			lineArray.push_back(d_data);
        }

        lattice_planner::CubicCoefficients p;
        p.s = lineArray[0];
        p.a0 = lineArray[1];
        p.a1 = lineArray[2];
        p.a2 = lineArray[3];
        p.a3 = lineArray[4];
        p.b0 = lineArray[5];
        p.b1 = lineArray[6];
        p.b2 = lineArray[7];
        p.b3 = lineArray[8];     
		coefficients.push_back(p);
	}
	return coefficients;
}


int main(int argc, char **argv) {

    ROS_INFO("lattice planner is running...");
    ros::init(argc, argv, "lattice_planner");
	std::vector<lattice_planner::CubicCoefficients> coefficients;
	// call the lattice planner
	coefficients = loadCoefficients();

	// ros::Rate loop_rate(0.1);
	// while (ros::ok()){

	lattice_planner::Dijkstra planner(coefficients);
	
	// planner.generatePath();
	// planner.ShowRefLineInRviz();
	// planner.generateLattice();
	// planner.ShowObstaclesInRviz();
	ros::spin();               // check for incoming messages
		// loop_rate.sleep();
	// }

	return 0;

}
