//
// Created by ustb on 19-7-8.
//

#include "costFunctions.h"
#include <cmath>

latticeParameter::costParameter cp;


double kappa_cost(const Node node, const Node & next_node, std::vector<std::vector<std::vector<double > > > & coefficients) {
    double mean_kappa = trajectory_kappa(node, next_node, coefficients);

    return mean_kappa;
}


double reference_line_cost(const Node start_node, const Node next_node, double & refline){

    double dis1 = fabs(start_node.y_ - refline);
    double dis2 = fabs(next_node.y_ - refline);

    double cost = (dis1 + dis2)/2.0;
    return cost;
}


double collision_risk(const Node start_node, const Node next_node, const std::vector<std::vector<double>>& obstacle) {

    std::array<double, 3> start= {{start_node.x_, start_node.y_, 0.0 * M_PI / 180.0}};
    std::array<double, 3> end = {{next_node.x_, next_node.y_, 0.0 * M_PI / 180.0}};
    CubicPolynomial cubic(start, end);
    std::vector<std::vector<double >> set = cubic.computeFrenetCoordinates();
    std::vector<double> s = *(set.cbegin());
    std::vector<double> rho = *(set.cbegin()+1);
    std::vector<double> theta = *(set.cbegin()+2);

    double dis = 100.0;
    for(size_t i=0; i < (s.size()-1); i=i+5){
        for (size_t j=0; j!=obstacle.size(); j++ ){
            double TmpDis =sqrt(pow((s[i]-obstacle[j][0]), 2) + pow((rho[i] - obstacle[j][1]), 2));
            if (TmpDis < (cp.r_circle + cp.obstacle_inflation)){
                if (TmpDis<dis) dis = TmpDis;
            }
        }
    }
    double cost = 1 / (dis + 0.001);
    return cost;
}


double total_cost(const Node node, const Node next_node, double & refline, const std::vector<std::vector<double>> & obstacle, 
                    std::vector<std::vector<std::vector<double > > > & coefficients) {

    double cost1 = kappa_cost(node, next_node, coefficients);
    double cost2 = reference_line_cost(node, next_node, refline);
    double cost3 = collision_risk(node, next_node, obstacle);

//    cout<<"cost1:"<< cost1<<"cost2:"<<cost2<<"cost3:"<<cost3<<endl;
    double cost = cp.alpha1 * cost1 + cp.alpha2 * cost2 + cp.alpha3 * cost3;
    return cost;
}


