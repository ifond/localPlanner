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
#include <algorithm>
#include "costFunctions.h"
#include <typeinfo>

#include "../config/parameters.h"



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
* @brief Get permissible motion primatives for the bot
* @return vector of permissible motions
*/
std::vector<Node> GetMotion();


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
    bool nodeIsInClosed(Node &p);
    Node minCostInOpen();
    bool NodeInOpen(Node &p, std::vector<Node>::iterator &it);
    void DeleteOpenNode(Node p);
    Node determineGoal();
    std::vector<Node> pathTrace(Node & p);


        private:
    std::vector<Node> open_list_;
    std::vector<Node> closed_list_;
    Node start_, goal_;

};



#endif //LATTICEPLANNER_LATTICEPLANNER_H
