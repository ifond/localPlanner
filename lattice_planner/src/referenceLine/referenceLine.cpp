//
// Created by ustb on 19-7-12.
//

#include <Eigen/Dense>
#include "referenceLine.h"
#include "../config/parameters.h"


namespace lattice_planner{

parameters param_r;

coefficients_type refLine_coefficients(){
    
    std::vector<double > x=param_r.waypoints_x;
    std::vector<double > y=param_r.waypoints_y;
    std::vector<double > r_x, r_y, r_heading, r_curvature, r_s;

    cubicSpline::Spline2D cubic_spline(x, y);
    for(double i=0; i<cubic_spline.s.back(); i=i+0.1){
        std::array<double , 2> point_ = cubic_spline.calculatePosition(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        r_heading.push_back(cubic_spline.calculateHeading(i));
        r_curvature.push_back(cubic_spline.calculateCurvature(i));
        r_s.push_back(i);
    }
//    std::cout<<"----------------cubic spline--------------"<<std::endl;
//    std::cout<<r_x.size()<<" "<<r_y.size()<<" "<<r_heading.size()<<" "
//    <<r_curvature.size()<<" "<< r_s.size()<<std::endl;

    std::vector<std::vector<double > > coefficients = SparseWayPoints(r_x, r_y, r_heading, r_s);
    return coefficients;

}

std::vector<double> arcLengthRefLine(std::vector<double> pose, 
                                                    std::vector<double> nextPose,
                                                    double s0, 
                                                    double sf){
    double x0 = pose[0];
    double y0 = pose[1];
    double theta0 = pose[2];
    double xf = nextPose[0];
    double yf = nextPose[1];
    double thetaf = nextPose[2];

    Eigen::Matrix4d A;
    A << 1.0, s0, s0*s0, s0*s0*s0,
            1.0, sf, sf*sf, sf*sf*sf,
            0.0, 1.0, 2.0 * s0, 3 * s0 * s0,
            0.0, 1.0, 2.0 * sf, 3 * sf * sf;
    Eigen::Vector4d B(x0, xf, cos(theta0), cos(thetaf));
    Eigen::Vector4d a_vec = A.colPivHouseholderQr().solve(B);

    Eigen::Vector4d C(y0, yf, sin(theta0), sin(thetaf));
    Eigen::Vector4d b_vec = A.colPivHouseholderQr().solve(C);

    std::vector<double> a={{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    for (int i=0; i<a_vec.size();i++){
        a[i] = a_vec[i];
        a[i+4] = b_vec[i];

    }
    std::vector<double > Tmp_coefficients = {{s0}};
    Tmp_coefficients.insert(Tmp_coefficients.end(), a.begin(), a.end());
//    std::cout<<"=============tmp coefficients==========="<<std::endl;
//    std::cout<<s_d.size()<<a.size()<<b.size()<<std::endl;
    return Tmp_coefficients;

}

std::vector<std::vector<double > > SparseWayPoints(std::vector<double > r_x, 
                                                    std::vector<double > r_y, 
                                                    std::vector<double > r_heading, 
                                                    std::vector<double > r_s){
    std::vector<std::vector<double > > coefficients;

    int step=20;
    for (auto i=0; i < r_x.size(); i=i+step){
        // append the last point to the coefficient_list
        if ( (i+step) > (r_x.size() - 1)) {
            std::vector<double > pose = {{r_x[i], r_y[i], r_heading[i]}};
            std::vector<double > next_pose = {{r_x.back(), r_y.back(), r_heading.back()}};
            double s0 = r_s[i];
            double sf = r_s.back();

            std::vector<double > TmpCoefficients = arcLengthRefLine(pose, next_pose, s0, sf);
            coefficients.push_back(TmpCoefficients);
        }

        else{
            std::vector<double > pose = {{r_x[i], r_y[i], r_heading[i]}};
            std::vector<double > next_pose = {{r_x[i+step], r_y[i+step], r_heading[i+step]}};
            double s0 = r_s[i];
            double sf = r_s[i+step];

            std::vector<double > TmpCoefficients = arcLengthRefLine(pose, next_pose, s0, sf);
            coefficients.push_back(TmpCoefficients);
        }
        // std::cout<<"---------------tmp coefficients------"<<std::endl;
        // std::cout<<TmpCoefficients.size()<<std::endl;

    }

    return coefficients;

}

std::array<double, 3> poses_of_reference_line(double s, coefficients_type & coefficients){

    int s_id = 0;
    s_id = binary_search(coefficients, s);
    double s_start = coefficients[s_id][0];
    std::vector<double> a = {{coefficients[s_id][1], coefficients[s_id][2], coefficients[s_id][3], coefficients[s_id][4]}};
    std::vector<double > b = {{coefficients[s_id][5], coefficients[s_id][6], coefficients[s_id][7], coefficients[s_id][8]}};

    double x = a[0] + a[1] * s + a[2] * s * s + a[3] * s * s * s;
    double d_x = a[1] + 2 * a[2] * s + 3 * a[3] * s * s;
    double y = b[0] + b[1] * s + b[2] * s * s + b[3] * s * s * s;
    double d_y = b[1] + 2 * b[2] * s + 3 * b[3] * s * s;

    double theta = std::atan2(d_y, d_x);

    std::array<double, 3> pose = {{x, y, theta}};
    return pose;
}

int binary_search(coefficients_type & coefficients, double s) {

    int head = 0;
    int tail = coefficients.size() - 1;
    int mid = 0;

    while (head < tail){
        mid = (head + tail) / 2;
        if (coefficients[mid][0] < s) head = mid+1;
        else if (s<coefficients[mid][0]) tail = mid -1;
        else return mid;
    }

    if (s < coefficients[head][0])
        return head -1;
    else
        return head;
}

} //namespace referenceLine
