//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_LATTICEPLANNER_H
#define LATTICEPLANNER_LATTICEPLANNER_H


#include <ros/ros.h>
#include "../config/parameters.h"
#include "costFunctions.h"
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "../FrenetMath/cartesianToFrenet.h"
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl/point_types.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <ctime>
#include <stack>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>


namespace lattice_planner{


/**
* @brief Node class
* @param x_: X value, when we make planning in the frenet coordinate system,
*            x_ represents the longitudinal station
* @param y_ Y value, when we make planning in the frenet coordinate system,
*           y_ represents the lateral offset
* @param cost_ Cost to get to this node
* @param id_ Node's id
* @param pid_ Node's parent's id
*/
class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
public:
    /** \brief x coordinate */
    double x_;
    /** \brief y coordinate */
    double y_;
    /** \brief cost to reach this node */
    double cost_;
    /** \brief Node id */
    int id_;
    /** \brief Node's parent's id */
    int pid_;


    /**
    * @brief Constructor for Node class
    * @param x X value
    * @param y Y value
    * @param cost Cost to get to this node
    * @param id Node's id
    * @param pid Node's parent's id
    */
    Node(double x = 0.0, double y = 0.0, double cost = 0.0, int id = 0, int pid = 0);

    /**
    * @brief Prints the values of the variables in the node
    * @return void
    */
    void PrintStatus();

    /**
    * @brief Overloading operator + for Node class
    * @param p node
    * @return Node with current node's and input node p's values added
    */
    Node operator+(Node p);

    /**
    * @brief Overloading operator - for Node class
    * @param p node
    * @return Node with current node's and input node p's values subtracted
    */
    Node operator-(Node p);

    /**
    * @brief Overloading operator = for Node class
    * @param p node
    * @return void
    */
    void operator=(Node p);

    /**
    * @brief Overloading operator == for searching the id of the Node class
    * @param p node
    * @return bool whether current node equals input node
    */
    bool operator==(Node p);


    /**
    * @brief Overloading operator != for Node class
    * @param p node
    * @return bool whether current node is not equal to input node
    */
    bool operator!=(Node p);
};

/**
* @brief Struct created to encapsulate function compare cost between 2 nodes. 升序
*/
struct compare_cost{
    bool operator()(Node& p1, Node& p2);
};


/**
 * dijkstra planning in the path lattice
 */
class Dijkstra{
public:

    Dijkstra() = default;
    Dijkstra(std::vector<arc_length_parameter> &coefficients);
    
    /**
     * @brief Main algorithm of Dijkstra.
     */
    void makePlan();

    /**
    * calculate id of the vertices in the path lattice
    * @param p
    * @return
    */
    int calIndex(Node p);
    bool nodeIsInClosed(Node &p);
    Node minCostInOpen();
    bool NodeInOpen(Node &p, std::vector<Node>::iterator &it);
    void DeleteOpenNode(Node p);
    void determineGoal();
    void pathTrace(std::stack<Node> &path);
    
    /**
    * @brief Get permissible motion primatives for the bot
    * @return vector of permissible motions
    */
    std::vector<Node> GetNextMotion();
    
private:
    ros::NodeHandle nh;
    std::vector<Node> open_list_;
    std::vector<Node> closed_list_;
    Node nodeStart_, goal_;
    std::vector<arc_length_parameter> coefficients_;
    // std::vector<geometry_msgs::PoseStamped> optimal_path, path_lattice; 
    // sampling poses from the Frenet coordinate system
    std::vector<pose_frenet> samplingPoses;
    
    std::vector<geometry_msgs::PoseStamped> path_lattice;
    sensor_msgs::PointCloud2 cloud_msg;
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    visualization_msgs::MarkerArray obstaclesInRviz_;
    nav_msgs::Path optimal_path_;
    nav_msgs::Path ref_line_;

    geometry_msgs::PoseWithCovarianceStamped cartesianStart_;
    pose_frenet FrenetStart_;
    ros::Publisher path_pub;
	ros::Publisher refLine_pub;
	// publish lattice
	// ros::Publisher lattice_pub = nh.advertise<nav_msgs::Path>("lattice",1, true);

	ros::Publisher lattice_pub_;
	// std::vector<ros::Publisher> obs_pubs; 
	ros::Publisher obss_pub;
    ros::Subscriber SubStart_;
    ros::Publisher pub_startInRviz_;
    double begin_time_;
    double running_time_;
	

public:
    void samplingNodes();
    void generatePath();
    void generateLattice();
    void generateRefLine();
    
    void ShowRefLineInRviz();
    void ShowObstaclesInRviz();
    void setStart_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial);


private:
   
    double r_circle = 1.0;
    double d_circle = 2.0;
    double obstacle_inflation = 1.5;
    int longitudinal_num = 5;
    int lateral_num = 9;  // 横向采样个数
    double longitudinal_step = 20.0;
    double lateral_step = 0.5;
    double lane_width = 3.75;
    int SampleNumberOnOneSide;    // sampling number on one side of the reference line
    double s0;
    double s_max;
    double s_end;
    double refLineRho_;
    
    // the frenet coordinates of obstacles
    pose_frenet obs1,obs2,obs3;
    std::vector<pose_frenet> obstacles_;

    double obstacleHeading = 0.0 * M_PI / 180.0;

    // 最后一列的编号
    std::vector<int> last_column_id; 
    double vehicle_body_fast_check_circle = 3.0;  // body collision fast check circle, the radius is 3.0 m
    double vehicle_body_envelope_circle = 1.0;    // little circles, the radius is 1.0 m  

    bool show_lattice_in_rviz=true; 


};

}   // namespace lattice_planner


#endif //LATTICEPLANNER_LATTICEPLANNER_H
