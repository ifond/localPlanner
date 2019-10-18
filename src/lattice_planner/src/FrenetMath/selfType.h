//
// Created by ustb on 19-7-12.
//
//  input:way-points
//  output:coefficients of the arc-length parameterized reference line
// Paper:
// Fast Collision Checking for Intelligent Vehicle Motion Planning


#ifndef LATTICEPLANNER_SELFTYPE_H
#define LATTICEPLANNER_SELFTYPE_H

#include <vector>
#include <array>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


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
    double x_=0.0;
    /** \brief y coordinate */
    double y_=0.0;
    /** \brief cost to reach this node */
    double cost_=0.0;
    /** \brief Node id */
    int id_=0;
    /** \brief Node's parent's id */
    int pid_=0;

    // Node();

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
 * self defined data structure
 * 
 * 
 */
struct FrenetPose{
    double s;
    double rho;
    double heading=0.0;
};


struct CartesianPose{
    double x;
    double y;
    double yaw;
};


struct CartesianPoint{
    double x,y;
};


struct CartesianPath{
    std::vector<CartesianPose> cartesianPath;
};


struct FrenetPath{
    std::vector<FrenetPose> frenetPath;
};


/**
 * strcut arc_length_parameter
 * record the coefficients of a cubic arc-length curve
 * x=a0 + a1*s + a2*s^2 + a3*s^3;
 * y=b0 + b1*s + b2*s^2 + b3*s^3;
 * d_x = a1 + 2 * a2 * s + 3 * a3 * s ** 2
 * d_y = b1 + 2 * b2 * s + 3 * b3 * s ** 2
 */
struct CubicCoefficients{
    double s;
    double a0, a1, a2, a3;
    double b0, b1, b2, b3;
};


struct PointsObstacle
{
    /* data */
    std::vector<CartesianPoint> points;
};


class VehicleBox{
private:
    CartesianPoint p0,p1,p2,p3;

    /**
     * 外接圆半径
     */
    double CircumscribedCircleRadius_;

    /**
     * 内接圆半径
     */
    double InscribedCircleRadius_;

    /**
     * 后轴中点
     */
    CartesianPose RearAxleMidPoint_;

    double VehicleWidth_;
    double VehicleLength_;

    int NumCircles_;

    double BodyDiskRadius_;
    double DistanceBetweenDisks_;

public:
    VehicleBox();


};


struct VehicleBoxes{
    /* data */
    std::vector<VehicleBox> boxes;
};






} //namespace latticeParameter

#endif //LATTICEPLANNER_PARAMETERS_H
