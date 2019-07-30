//
// Created by ustb on 19-7-9.
//

#include "calKappa.h"
#include <array>
#include <cmath>


namespace lattice_planner{

double trajectory_kappa(const Node node, 
                        const Node next_node, 
                        std::vector<std::vector<double > > & coefficients){

    std::array<double, 3> start= {{node.x_, node.y_, 0.0 * M_PI / 180.0}};
    std::array<double, 3> end = {{next_node.x_, next_node.y_, 0.0 * M_PI / 180.0}};
    CubicPolynomial cubic(start, end);
    std::vector<std::vector<double >> set = cubic.computeFrenetCoordinates();
    std::vector<double> s = *(set.begin());
    std::vector<double> rho = *(set.begin()+1);
    std::vector<double> theta = *(set.begin()+2);

//    cout<<"-------------------s,rho,theta-size()-------------"<<endl;
//    cout<<s.size()<<"-"<<rho.size()<<"-"<<theta.size()<<endl;

    std::vector<double> kappa_set;
    for (int i=0; i < (s.size()-2); i=i+5){
        double x0, x1, x2, y0, y1, y2, k1, k2, k3, s0, rho0, s1, rho1, s2, rho2, theta_rho0, theta_rho1, theta_rho2;
        std::vector<double> cartesian_pose;
        s0 = s[i];
        rho0 = rho[i];
        theta_rho0 = theta[i];
        s1=s[i + 1];
        rho1=rho[i + 1];
        theta_rho1 = theta[i+1];
        s2=s[i + 2];
        rho2=rho[i + 2];
        theta_rho2 = theta[i+2];

        cartesian_pose=frenet_to_cartesian(s0, rho0, theta_rho0, coefficients);
        x0 = cartesian_pose[0];
        y0 = cartesian_pose[1];
        cartesian_pose=frenet_to_cartesian(s1, rho1, theta_rho1, coefficients);
        x1 = cartesian_pose[0];
        y1 = cartesian_pose[1];
        cartesian_pose=frenet_to_cartesian(s2, rho2, theta_rho2, coefficients);
        x2 = cartesian_pose[0];
        y2 = cartesian_pose[1];

        k1 = (x1 - x0) * (y2 - 2 * y1 + y0);
        k2 = (y1 - y0) * (x2 - 2 * x1 + x0);
        k3 = pow((pow((x1-x0),2)+pow((y1-y0), 2)), 1.5);

        if (k3 == 0.0) kappa_set.push_back(0.0);

        else kappa_set.push_back((k1 - k2) / k3);
    }

//    cout<<"--------------------kappa_size-----------"<<endl;
//    cout<<kappa_set.size()<<endl;

    double sum_kappa = 0.0;
    for (auto i:kappa_set){
        sum_kappa = sum_kappa + pow(i,2);
    }
    double mean_kappa = sum_kappa/kappa_set.size();

//    cout<<"---------------mean kappa-----------"<<endl;
//    cout<<mean_kappa<<endl;

    return mean_kappa;

}

}