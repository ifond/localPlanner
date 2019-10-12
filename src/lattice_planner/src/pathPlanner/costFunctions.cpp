//
// Created by ustb on 19-7-8.
//

#include "costFunctions.h"
#include <cmath>
// #include <yaml-cpp/yaml.h>


namespace lattice_planner{


double kappa_cost(const Node node, 
                const Node & next_node, 
                std::vector<CubicCoefficients> & coefficients) {
    double mean_kappa = trajectory_kappa(node, next_node, coefficients);
    // ROS_INFO("trajectory_kappa() is completed...");
    return mean_kappa;
}


double reference_line_cost(const Node start_node, const Node next_node, double & refline){

    double dis1 = fabs(start_node.y_ - refline);
    double dis2 = fabs(next_node.y_ - refline);

    double cost = (dis1 + dis2)/2.0;
    return cost;
}


double collision_risk(const Node start_node, const Node next_node, const std::vector<FrenetPose>& obstacle) {

    FrenetPose start;
    start.s=start_node.x_;
    start.rho = start_node.y_;
    start.heading = 0.0 * M_PI / 180.0;

    FrenetPose end;
    end.s = next_node.x_;
    end.rho = next_node.y_;
    end.heading = 0.0 * M_PI / 180.0;

    CubicPolynomial cubic(start, end);
    std::vector<FrenetPose> frenet_path = cubic.computeFrenetCoordinates();

    double r_circle=1.0;
    double obstacle_inflation=1.5;
    // ros::param::get("/lattice_planner/r_circle", r_circle);
    // std::cout<<"r_circle:"<<r_circle<<std::endl;
    // ros::param::get("/lattice_planner/obstacle_inflation", obstacle_inflation);
    double dis = 100.0;
    for(size_t i=0; i < (frenet_path.size()-1); i=i+5){
        for (size_t j=0; j!=obstacle.size(); j++ ){
            double TmpDis =sqrt(pow((frenet_path[i].s-obstacle[j].s), 2) + pow((frenet_path[i].rho - obstacle[j].rho), 2));
            if (TmpDis < (r_circle + obstacle_inflation)){
                if (TmpDis<dis) dis = TmpDis;
            }
        }
    }
    double cost = 1 / (dis + 0.001);
    return cost;
}


double total_cost(const Node node, 
                const Node next_node, 
                double & refline, 
                const std::vector<FrenetPose> & obstacle, 
                std::vector<CubicCoefficients> & coefficients) {

    double cost1 = kappa_cost(node, next_node, coefficients);
    double cost2 = reference_line_cost(node, next_node, refline);
    double cost3 = collision_risk(node, next_node, obstacle);

    // ROS_INFO("costFunction method is complted...");
    // cout<<"cost1:"<< cost1<<"cost2:"<<cost2<<"cost3:"<<cost3<<endl;

    // double alpha1,alpha2,alpha3;
    double alpha1 = 100;
    double alpha2 = 1;
    double alpha3 = 10;
    double alpha4 = 0.0;
    // ros::param::get("/lattice_planner/alpha1", alpha1);
    // ros::param::get("/lattice_planner/alpha2", alpha2);
    // ros::param::get("/lattice_planner/alpha3", alpha3);
    double cost = alpha1 * cost1 + alpha2 * cost2 + alpha3 * cost3;
    return cost;
}

}   // namespace lattice_planner

