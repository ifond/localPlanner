//
// Created by ustb on 19-7-8.
//

#include <iostream>
#include "cubicPolynomial.h"
#include <string>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

namespace lattice_planner{


CubicPolynomial::CubicPolynomial(const lattice_planner::pose_frenet & start, 
                                const lattice_planner::pose_frenet & end) {
    start_condition_ = start;
    end_condition_ = end;
    computeCoefficients();
}


void CubicPolynomial::computeCoefficients() {
    double s0 = start_condition_.s;
    double rho0 = start_condition_.rho;
    double theta0 = start_condition_.heading;

    double sg = end_condition_.s;
    double rhog = end_condition_.rho;
    double thetag = end_condition_.heading;

    Matrix4d A;
    Vector4d B(rho0, rhog, tan(theta0), tan(thetag));

    A << 1.0, s0, s0*s0, s0*s0*s0,
            1.0, sg, sg*sg, sg*sg*sg,
            0.0, 1.0, 2.0 * s0, 3 * s0 * s0,
            0.0, 1.0, 2.0 * sg, 3 * sg * sg;

    Vector4d TmpCoefficients = A.colPivHouseholderQr().solve(B);
    // std::cout<<"print refLine_coefficients:"<<TmpCoefficients<<std::endl;

    coeffients_.c0 = TmpCoefficients[0];
    coeffients_.c1 = TmpCoefficients[1];
    coeffients_.c2 = TmpCoefficients[2];
    coeffients_.c3 = TmpCoefficients[3];

}

std::vector<lattice_planner::pose_frenet> CubicPolynomial::computeFrenetCoordinates() {

    double step = 0.1;
    double s0 = start_condition_.s;
    double sg = end_condition_.s;
    int length_s = int((sg-s0)/step);
    // cout<<"------------------cubic sampling points-------------"<<endl;
    // cout<<length_s<<endl;

    double coeff0 = coeffients_.c0;
    double coeff1 = coeffients_.c1;
    double coeff2 = coeffients_.c2;
    double coeff3 = coeffients_.c3;
    double tmp_s = s0;
    std::vector<lattice_planner::pose_frenet> frenet_path;
    for (int i=0; i<length_s; i++){
        tmp_s = tmp_s+step;
        double tmp_rho = coeff0 + coeff1 * tmp_s + coeff2 * pow(tmp_s, 2) + coeff3 * pow(tmp_s, 3);
        lattice_planner::pose_frenet state0;
        state0.s = tmp_s;
        state0.rho = tmp_rho;

        double next_s = tmp_s + step;
        double next_rho = coeff0 + coeff1 * next_s + coeff2 * pow(next_s, 2) + coeff3 * pow(next_s, 3);
        lattice_planner::pose_frenet state1;
        state1.s = next_s;
        state1.rho = next_rho;

        double tmp_theta = cal_FrenetHeading(state0, state1);

        state0.heading = tmp_theta;
        frenet_path.push_back(state0);
    }

    return frenet_path;

}

void CubicPolynomial::print_coefficients(){

    std::cout<<"===============print cubic polynomial's coefficients======="<<std::endl;
    std::cout<<coeffients_.c0<<"-"<<coeffients_.c1<<"-"<<coeffients_.c2<<"-"<<coeffients_.c3<<std::endl;

}

}