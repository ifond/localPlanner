#include <iostream>
#include <array>
#include "curves/cubicPolynomial.h"
#include "pathPlanner/latticePlanner.h"
#include "config/parameters.h"
#include "referenceLine/referenceLine.h"


int main() {

//    std::array<double, 3> start= {{0.0, 0.0, 0.0}};
//    std::array<double, 3> end = {{20.0, 2.0, 0.0}};
//    CubicPolynomial cubic(start, end);
//    cubic.computeFrenetCoordinates();


    std::cout << "-----------------------------------------------------------" << std::endl;
    std::cout << "--------------------- lattice planner ---------------------" << std::endl;
    std::cout << "----------------------search algorithm: Dijkstra-----------" << std::endl;

    latticeParameter::plannerParameter pp;
    Node initState(pp.start_SRho[0], pp.start_SRho[1], pp.start_SRho[2], 0, -1);
    Dijkstra planner;
    std::vector<Node> closed = planner.dijkstra(initState);
//    cout<<"---------------closed_size---------------"<<endl;
//    cout<<closed.size()<<endl;
    Node goal= planner.determineGoal();
    cout<<"==============goal================="<<endl;
    cout<<"["<<goal.x_<<","<<goal.y_<<"]"<<endl;
    std::vector<Node> pathNode = planner.pathTrace(goal);

    cout<<"------------path node-----------------"<<endl;
    for (auto i:pathNode){
        std::cout << "[" << i.x_ << "," << i.y_ <<"]"<< std::endl;
    }

    std::vector<std::vector<Vec_d > > coefficients = referenceLine::refLine_coefficients();
    cout<<"coefficients:"<<coefficients[10][1][0]<<endl;

    return 0;
}