//
// Created by ustb on 19-7-8.
//

#include <iostream>
#include "cubicPolynomial.h"
#include <string>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;


CubicPolynomial::CubicPolynomial(
        const std::array<double, 3> & start, const std::array<double, 3> & end) {
    start_condition_ = start;
    end_condition_ = end;
    computeCoefficients();
}


void CubicPolynomial::computeCoefficients() {
    double s0 = *(start_condition_.begin());
    double rho0 = *(start_condition_.begin()+1);
    double theta0 = *(start_condition_.begin() + 2);

    double sg = *(end_condition_.begin());
    double rhog = *(end_condition_.begin() + 1);
    double thetag = *(end_condition_.begin() + 2);

    Matrix4d A;
    Vector4d B(rho0, rhog, tan(theta0), tan(thetag));

    A << 1.0, s0, s0*s0, s0*s0*s0,
            1.0, sg, sg*sg, sg*sg*sg,
            0.0, 1.0, 2.0 * s0, 3 * s0 * s0,
            0.0, 1.0, 2.0 * sg, 3 * sg * sg;

    Vector4d TmpCoefficients = A.colPivHouseholderQr().solve(B);
//    std::cout<<"print coefficients:"<<TmpCoefficients<<std::endl;

    for (int i=0; i<TmpCoefficients.size();i++){
        *(coeffients_.begin()+i) = TmpCoefficients[i];
//        std::cout<<* (coeffients_.begin()+i)<<endl;
    }

}

std::vector<std::vector<double >> CubicPolynomial::computeFrenetCoordinates() {

    double step = 0.1;
    double s0 = *(start_condition_.begin());
    double sg = *(end_condition_.begin());
    int length_s = int((sg-s0)/step);
//    cout<<"------------------cubic sampling points-------------"<<endl;
//    cout<<length_s<<endl;

    double coeff0 = *(coeffients_.begin());
    double coeff1 = *(coeffients_.begin()+1);
    double coeff2 = *(coeffients_.begin()+2);
    double coeff3 = *(coeffients_.begin()+3);
    double tmp_s = s0;
    for (int i=0; i<length_s; i++){
        tmp_s = tmp_s+step;
        double tmp_rho = coeff0 + coeff1 * tmp_s + coeff2 * pow(tmp_s, 2) + coeff3 * pow(tmp_s, 3);
        std::array<double, 2> state0 = {{tmp_s, tmp_rho}};
        double next_s = tmp_s + step;
        double next_rho = coeff0 + coeff1 * next_s + coeff2 * pow(next_s, 2) + coeff3 * pow(next_s, 3);
        std::array<double, 2> state1 = {{next_s, next_rho}};
        double tmp_theta = calFrenetHeading(state0, state1);

        s.push_back(tmp_s);
        rho.push_back(tmp_rho);
        theta.push_back(tmp_theta);
    }
//    cout<<s.size()<<"rho size:"<<rho.size()<<"theta size:"<<theta.size()<<endl;
//    cout<<*(s.cend()-1)<<"rho size:"<<*(rho.cend()-1)<<"theta size:"<<*(theta.cend()-1)<<endl;

    std::vector<std::vector<double >> set;
    set.push_back(s);
    set.push_back(rho);
    set.push_back(theta);
    return set;

}