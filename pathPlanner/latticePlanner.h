//
// Created by ustb on 19-7-8.
//

#ifndef LATTICEPLANNER_LATTICEPLANNER_H
#define LATTICEPLANNER_LATTICEPLANNER_H

#include <vector>
#include <queue>
#include <iostream>
#include <iomanip>
#include <array>

#define PI 3.1415926

double s_max = 100.0;
int longi_num = 5;
int lateral_num = 9;  // 横向采样个数
double longi_step = 20.0;
double latera_step = 0.5;
double lane_width = 3.75;
int LeftSampleNum = lateral_num/2;    // sampling number on one side of the reference line
double s0 = 0.0;
double s_end=s0+s_max;
double refLineRho = lane_width*0.5;
std::array<double, 4> start_SRho = {{s0, refLineRho, 0.0 * PI / 180.0}};

// 障碍物的frenet坐标,
std::vector<std::vector<double >>  obs= {{20, refLineRho - 1},
                                         {40, refLineRho + 2},
                                         {70, refLineRho + 2}};
double obstacleHeading=0.0 * PI / 180.0;

//last_column_id = [lateral_num * longi_num, lateral_num * (longi_num-1) + 1];  // 最后一列的编号

/**
* @brief Node class
* @param x_ X value
* @param y_ Y value
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
    Node(double x = 0, double y = 0, double cost = 0, int id = 0, int pid = 0);

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
    * @brief Overloading operator == for Node class
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
* @brief Struct created to encapsulate function compare cost between 2 nodes. Used in with multiple algorithms and classes
*/
struct compare_cost{

    /**
    * @brief Compare cost between 2 nodes
    * @param p1 Node 1
    * @param p2 Node 2
    * @return Returns whether cost to get to node 1 is greater than the cost to get to node 2
    */
    bool operator()(Node& p1, Node& p2);
};

/**
* @brief Get permissible motion primatives for the bot
* @return vector of permissible motions
*/
std::vector<std::array<double, 2>> GetMotion();

/**
* @brief Prints the grid passed
* @param grid Modify this grid
* @return void
*/

void PrintPath(std::vector<Node> path_vector, Node start_, Node goal_, std::vector<std::vector<int>> &grid);

/**
* @brief Prints out the cost for reaching points on the grid in the grid shape
* @param grid Grid on which algorithm is running
* @param point_list Vector of all points that have been considered. Nodes in vector contain cost.
* @return void
*/
void PrintCost(std::vector<std::vector<int>> &grid, std::vector<Node> point_list);


/**
 * dijkstra planning in the path lattice
 */
class Dijkstra{
public:

    /**
     * @brief Main algorithm of Dijstra.
     */
    std::vector<Node> dijkstra(Node start_);

    /**
    * calculate id of the vertices in the path lattice
    * @param p
    * @return
    */
    int calIndex(Node p);

private:
    std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
    std::vector<Node> closed_list_;
    Node start_, goal_;
    int n;
};


#endif //LATTICEPLANNER_LATTICEPLANNER_H
