#include <iostream>
#include <array>
#include <stack>
#include <vector>
#include "pathPlanner/latticePlanner.h"
#include "curves/cubicPolynomial.h"
#include "config/parameters.h"
#include "referenceLine/referenceLine.h"
#include "referenceLine/referenceLine.h"
#include "FrenetMath/frenetToCartesian.h"

#include "../include/lattice_planner/lattice_planner.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
// #include <visualization_msgs/Marker.h>


int main(int argc, char **argv) {
	lattice_planner::parameters param;

    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "--------------------- lattice planner ---------------------" << std::endl;
    std::cout << "----------------------search algorithm: Dijkstra-----------" << std::endl;

    
    ros::init (argc, argv, "lattice_planner");

	ros::NodeHandle ph;
	//	publish path
	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("optimal_path",1, true);
	// publish lattice
	// ros::Publisher lattice_pub = ph.advertise<nav_msgs::Path>("lattice",1, true);

	ros::Publisher pcl_pub = ph.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_msg;
	// ros::Publisher cloud_pub = ph.advertise<snensor_msgs::PointCloud2>("cloud",1, true);

	std::vector<double > x_vec, y_vec, x_lattice, y_lattice;

	coefficients_type coefficients = lattice_planner::pathPlanner(x_vec, y_vec);	//	call the lattice planner
	if(param.show_lattice_in_rviz){ 
		lattice_planner::generate_lattice(x_lattice, y_lattice, coefficients);
	}

	// x_lattice = set[0];
	// y_lattice = set[1];
	// std::cout<<"----x_lattice size----"<<std::endl;
	// std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;

	// 节点休眠时间1Hz;
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		nav_msgs::Path path;
		//nav_msgs::Path path;
		path.header.stamp=ros::Time::now();
		// path.header.frame_id="/my_frame";
		path.header.frame_id="/map";
		for (uint32_t i = 0; i < x_vec.size(); ++i){			
			geometry_msgs::PoseStamped this_pose_stamped;
			this_pose_stamped.pose.position.x = x_vec[i];
			this_pose_stamped.pose.position.y = y_vec[i];

			//geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1);
			this_pose_stamped.pose.orientation.x = 0;
			this_pose_stamped.pose.orientation.y = 0;
			this_pose_stamped.pose.orientation.z = 0;
			this_pose_stamped.pose.orientation.w = 1;

			this_pose_stamped.header.stamp=ros::Time::now();
			// this_pose_stamped.header.frame_id="/my_frame";
			this_pose_stamped.header.frame_id="/map";

			path.poses.push_back(this_pose_stamped);

		}
		path_pub.publish(path);

		if(param.show_lattice_in_rviz){
			// nav_msgs::Path lattice_path;
			// lattice_path.header.frame_id = "/map";
			// lattice_path.header.stamp = ros::Time::now();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < x_lattice.size(); i++){
					pcl::PointXYZ point;			
					point.x = x_lattice[i];
					point.y = y_lattice[i];
					point.z = 0;
					cloud->points.push_back(point);
					//geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1);

					// geometry_msgs::PoseStamped this_pose_stamped2;
					// this_pose_stamped2.pose.position.x = x_lattice[i][j];
					// this_pose_stamped2.pose.position.y = y_lattice[i][j];
					// this_pose_stamped2.pose.orientation.x = 0;
					// this_pose_stamped2.pose.orientation.y = 0;
					// this_pose_stamped2.pose.orientation.z = 0;
					// this_pose_stamped2.pose.orientation.w = 1;

					// this_pose_stamped2.header.stamp=ros::Time::now();
					// this_pose_stamped2.header.frame_id="/map";

					// lattice_path.poses.push_back(this_pose_stamped2);
				
			}
			// lattice_pub.publish(lattice_path);

			pcl::toROSMsg(*cloud, cloud_msg);
			// output.header.frame_id = "odom";

			// sensor_msgs::PointCloud2 cloud_msg;
			// pcl::toROSMsg(*cloud,cloud_msg);
			cloud_msg.header.stamp = ros::Time::now();
			cloud_msg.header.frame_id="/map";
			pcl_pub.publish(cloud_msg);
		}

		ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
		// ++count;
	}

	return 0;

}
