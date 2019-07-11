#include <iostream>
#include <Eigen/Dense>
#include <array>
#include "curves/cubicPolynomial.h"
#include "pathPlanner/latticePlanner.h"


using Eigen::MatrixXd;


int main() {

//    std::array<double, 3> start= {{0.0, 0.0, 0.0}};
//    std::array<double, 3> end = {{20.0, 2.0, 0.0}};
//    CubicPolynomial cubic(start, end);
//    cubic.computeFrenetCoordinates();


    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "--------------------- lattice planner ---------------------" << std::endl;
    std::cout << "----------------------search algorithm: Dijkstra-----------" << std::endl;

    Node initState(0.0, 0.0, 0.0, 0, -1);
    Dijkstra planner;
    std::vector<Node> closed = planner.dijkstra(initState);
    Node goal= planner.determineGoal();
    cout<<"==============goal================="<<endl;
    cout<<goal.x_<<"-"<<goal.y_<<endl;
    std::vector<Node> pathNode = planner.pathTrace(goal);
    for (auto i:pathNode){
        std::cout << "node:" << i.x_ << "-" << i.y_ << std::endl;
    }



//    std::vector<int> a={0,1,2};
//    cout<<a.size()<<endl;
//    a.erase(a.begin());
//    cout<<a.size()<<endl;

    return 0;
}