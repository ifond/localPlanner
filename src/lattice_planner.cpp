#include <iostream>
#include <array>
#include <stack>
#include "curves/cubicPolynomial.h"
#include "pathPlanner/latticePlanner.h"
#include "config/parameters.h"
#include "referenceLine/referenceLine.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include "referenceLine/referenceLine.h"
#include "frenetToCartesian.h"


// #include <visualization_msgs/Marker.h>


void planner(std::vector< std::vector< double>> & x_vec, std::vector< std::vector< double>> & y_vec,
			std::vector< std::vector< double>> & x_lattice, std::vector< std::vector< double>> & y_lattice){
	
	latticeParameter::plannerParameter pp;
    Node initState(pp.start_SRho[0], pp.start_SRho[1], pp.start_SRho[2], 0, -1);
	coefficients_type coefficients = referenceLine::refLine_coefficients();
    Dijkstra planner(coefficients, initState);
    std::vector<Node> closed = planner.dijkstra();
//    cout<<"---------------closed_size---------------"<<endl;
//    cout<<closed.size()<<endl;
    Node goal= planner.determineGoal();
    std::cout<<"==============goal================="<<std::endl;
    std::cout<<"["<<goal.x_<<","<<goal.y_<<"]"<<std::endl;
	std::stack<Node> pathNode;
    planner.pathTrace(goal, pathNode);

	std::vector<double > cartesian_pose;

	while (!pathNode.empty()){
		
		std::array<double, 3> start= {{ pathNode.top().x_, pathNode.top().y_, 0.0 * M_PI / 180.0}};
		pathNode.pop();
		if (pathNode.empty()) break;
   		std::array<double, 3> end = {{ pathNode.top().x_, pathNode.top().y_, 0.0 * M_PI / 180.0}};
				
		// std::cout<<"-----x,y-------------"<<std::endl;
		// std::cout<<start[0]<<","<<start[1]<<std::endl;
   		CubicPolynomial cubic(start, end);
   		std::vector<std::vector<double> > set=cubic.computeFrenetCoordinates();
		
		std::vector<double> s_vec=*(set.begin());
		std::vector<double> rho_vec=*(set.begin()+1);
		std::vector<double> theta = *(set.begin()+2);

		std::vector<double> tmp_x_vec, tmp_y_vec;
		for(std::size_t i=0; i!= s_vec.size(); i++){
			cartesian_pose=frenet_to_cartesian(s_vec[i], rho_vec[i], theta[i], coefficients);
        	tmp_x_vec.push_back(cartesian_pose[0]);
        	tmp_y_vec.push_back(cartesian_pose[1]);

		}
		x_vec.push_back(tmp_x_vec);
		y_vec.push_back(tmp_y_vec);
		
		// std::cout<<"----------path s,rho----------------"<<std::endl;
		// std::cout<<set.size()<<"rho size:"<<set[1].size()<<"theta size:"<<set[2].size()<<std::endl;


	}
	
	// std::cout<<"------------s size--------------"<<std::endl;
	// std::cout<<"s.size() "<<x_vec.size()<<"s[0].size(): "<<x_vec[0].size()<<std::endl;
    // std::vector<std::vector<Vec_d > > coefficients = referenceLine::refLine_coefficients();
    // cout<<"coefficients:"<<coefficients[10][1][0]<<endl;

	// generate the path lattice graph
	generate_lattice(x_lattice, y_lattice, coefficients);	
}


int main(int argc, char **argv) {

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


	std::vector< std::vector< double>> x_vec, y_vec, x_lattice, y_lattice;

	planner(x_vec, y_vec, x_lattice, y_lattice);	//	call the lattice planner

	// x_lattice = set[0];
	// y_lattice = set[1];
	// std::cout<<"----x_lattice size----"<<std::endl;
	// std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;

	// int count=0;
	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		nav_msgs::Path path;
		//nav_msgs::Path path;
		path.header.stamp=ros::Time::now();
		// path.header.frame_id="/my_frame";
		path.header.frame_id="/map";
		for (uint32_t i = 0; i < x_vec.size(); ++i){
			for (size_t j = 0; j < x_vec[i].size(); j++){
				
				geometry_msgs::PoseStamped this_pose_stamped;
				this_pose_stamped.pose.position.x = x_vec[i][j];
				this_pose_stamped.pose.position.y = y_vec[i][j];

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
		}
		path_pub.publish(path);

		// nav_msgs::Path lattice_path;
		// lattice_path.header.frame_id = "/map";
		// lattice_path.header.stamp = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		for (int i = 0; i < x_lattice.size(); i++){
			for (int j = 0; j < x_lattice[i].size(); j++){
				pcl::PointXYZ point;			

                point.x = x_lattice[i][j];
				point.y = y_lattice[i][j];
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
			
		}
		// lattice_pub.publish(lattice_path);

		pcl::toROSMsg(*cloud, cloud_msg);
    	// output.header.frame_id = "odom";

		// sensor_msgs::PointCloud2 cloud_msg;
		// pcl::toROSMsg(*cloud,cloud_msg);
		cloud_msg.header.stamp = ros::Time::now();
		cloud_msg.header.frame_id="/map";
		pcl_pub.publish(cloud_msg);

		ros::spinOnce();               // check for incoming messages
		loop_rate.sleep();
		// ++count;
	}

	return 0;

}