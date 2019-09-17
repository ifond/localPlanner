#include <iostream>
#include <array>
#include <stack>
#include <vector>
#include "pathPlanner/latticePlanner.h"
#include "curves/cubicPolynomial.h"
#include "config/parameters.h"
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
#include <visualization_msgs/MarkerArray.h>


std::vector<lattice_planner::arc_length_parameter> loadCoefficients(){
	std::ifstream readFile("/home/ustb/coefficients.csv");
    std::string lineStr;

	std::vector<lattice_planner::arc_length_parameter> coefficients;
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

        lattice_planner::arc_length_parameter p;
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

    ROS_INFO("--------------- lattice planner -------------");
    ROS_INFO("------------search algorithm: Dijkstra--------");

    ros::init (argc, argv, "lattice_planner");

	ros::NodeHandle ph;

	// int lateral_num, SampleNumberOnOneSide, longitudinal_num;
    // SampleNumberOnOneSide = lateral_num / 2;
    // double longitudinal_step, lateral_step, lane_width, s0;
    // ros::param::get("/lattice_planner/lateral_num", lateral_num);
	// std::cout<<"main lateral number:"<<lateral_num<<std::endl;
    // ros::param::get("/lattice_planner/longitudinal_step", longitudinal_step);
    // ros::param::get("/lattice_planner/lateral_step", lateral_step);
	double lane_width=3.75;
    // ph.getParam("lane_width", lane_width);
	ROS_INFO("LANE_WIDTH = %f", lane_width);
    // ros::param::get("/lattice_planner/longitudinal_num", longitudinal_num);
    // ros::param::get("/lattice_planner/s0", s0);

    double refLineRho=lane_width * 0.5;
	//	publish path
	ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("optimal_path",1, true);
	// publish lattice
	// ros::Publisher lattice_pub = ph.advertise<nav_msgs::Path>("lattice",1, true);

	ros::Publisher pcl_pub = ph.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);


	// std::vector<ros::Publisher> obs_pubs; 
	ros::Publisher obss_pub = ph.advertise<visualization_msgs::MarkerArray>("visualization_markerarray",100);
	// 设置初始形状为立方体
  	uint32_t shape = visualization_msgs::Marker::CUBE;


    // pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_msg;
	// ros::Publisher cloud_pub = ph.advertise<snensor_msgs::PointCloud2>("cloud",1, true);

	// std::vector<geometry_msgs::PoseStamped> optimal_path, path_lattice; 

	double begin_time = ros::Time::now().toSec();
	std::vector<lattice_planner::arc_length_parameter> coefficients;
	// call the lattice planner
	coefficients = loadCoefficients();
	// double lane_width=3.75;
	// double refLineRho = lane_width*0.5;
	double s0=0.0;
	bool show_lattice_in_rviz=true;


	lattice_planner::pose_frenet start_;
	start_.s=s0;
	start_.rho = refLineRho;
	start_.heading = 0;
    lattice_planner::Node initState(start_.s, start_.rho, start_.heading, 0, -1);

	// coefficients_type coefficients = refLine_coefficients();
    lattice_planner::Dijkstra planner(coefficients, initState);
    std::vector<geometry_msgs::PoseStamped> optimal_path = planner.generatePath();
    // std::cout<<"----------path ----------------"<<std::endl;
    // std::cout<<optimal_path.size()<<"size:"<<std::endl;
	std::vector<geometry_msgs::PoseStamped> path_lattice;
	if(show_lattice_in_rviz){ 
		path_lattice = planner.generateLattice();
	}
	// std::vector<lattice_planner::arc_length_parameter>  coefficients = startPlan(optimal_path, path_lattice);	


	// std::cout<<"-----optimal_path size---------------"<<std::endl;
	// std::cout<<optimal_path.size()<<std::endl;
	double running_time = (ros::Time::now().toSec() - begin_time);
	ROS_INFO ("%f secs for local_planner", running_time);

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
		for (uint32_t i = 0; i < optimal_path.size(); ++i){			
			geometry_msgs::PoseStamped this_pose_stamped;
			this_pose_stamped.pose.position = optimal_path[i].pose.position;
			// this_pose_stamped.pose.position.y = y_vec[i];

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

		if(1){
			// nav_msgs::Path lattice_path;
			// lattice_path.header.frame_id = "/map";
			// lattice_path.header.stamp = ros::Time::now();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			for (int i = 0; i < path_lattice.size(); i++){
					pcl::PointXYZ point;			
					point.x = path_lattice[i].pose.position.x;
					point.y = path_lattice[i].pose.position.y;
					point.z = path_lattice[i].pose.position.z;
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

		/**
		 * pub obstacles
		 */
		visualization_msgs::MarkerArray obss;

		std::vector<std::vector<double>> obstacles = {{20, refLineRho - 1},{40, refLineRho + 1},{70, refLineRho - 1}};

		for (int i=0; i != obstacles.size(); i++)
		{
			// Create lines and points
			visualization_msgs::Marker obs;
			obs.header.frame_id =  "/map";
			obs.header.stamp = ros::Time::now();
			obs.ns = "obstacles";
			obs.action = visualization_msgs::Marker::ADD;
			//obs.pose.orientation.w = 1.0;
			obs.id = i;
			obs.type = shape;

			double obs_s = obstacles[i][0];
			double obs_rho = obstacles[i][1];
			double heading = 0;

			geometry_msgs::PoseStamped poseCartesian = lattice_planner::frenet_to_cartesian(obs_s, obs_rho, heading, coefficients);
			obs.pose = poseCartesian.pose;
			// obs.pose.position.x = poseCartesian.x;
			// obs.pose.position.y = poseCartesian.y;
			// obs.pose.position.z = 0;
			// obs.pose.orientation.x = 0.0;
			// obs.pose.orientation.y = 0.0;
			// obs.pose.orientation.z = 0.0;
			// obs.pose.orientation.w = 1.0;

			// 设置标记的比例，所有方向上尺度1表示1米
			obs.scale.x = 1.0;
			obs.scale.y = 1.0;
			obs.scale.z = 1.0;

			//设置标记颜色，确保alpha（不透明度）值不为0
			obs.color.r = 0.0f;
			obs.color.g = 1.0f;
			obs.color.b = 0.0f;
			obs.color.a = 1.0;
	
			obss.markers.push_back(obs);
     
			}
 
        obss_pub.publish(obss);

		ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
		// ++count;
	}

	return 0;

}
