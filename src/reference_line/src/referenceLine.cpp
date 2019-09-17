//
// Created by ustb on 19-7-12.
//

#include <Eigen/Dense>
#include "referenceLine.h"
#include <fstream>


namespace reference_line{

// std::vector<double > waypoints_x={{0.0, 20.0, 50, 100.0, 150.0, 220.0, 300.0, 350.0, 400.0, 430.0, 370.0, 300, 200.0}};
// std::vector<double > waypoints_y={{0.0, 70.0, 100, 120.0, 100.0, 150.0, 180.0, 150.0, 110.0, 20.0, -80.0, -80.0, -80.0}};

std::vector<double > waypoints_x={{0.0, 20.0, 50, 100.0, 150.0, 220.0, 300.0}};
std::vector<double > waypoints_y={{0.0, 70.0, 100, 120.0, 100.0, 150.0, 180.0}};

coefficients_type refLine_coefficients(){
    // std::ofstream writeFile;    
    // writeFile.open("coefficients.csv", std::ios::out); // 打开模式可省略 
    std::ofstream writeFile;    
    writeFile.open("/home/ustb/coefficients_test.csv", std::ios::ate); // 打开模式可省略 
    writeFile.close();
    std::vector<double > x=waypoints_x;
    std::vector<double > y=waypoints_y;
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
    // writeFile.close();
    return coefficients;

}

std::vector<double> arcLengthRefLine(std::vector<double> pose, 
                                                    std::vector<double> nextPose,
                                                    double s0, 
                                                    double sf){

    std::ofstream writeFile;    
    writeFile.open("/home/ustb/coefficients_test.csv", std::ios::app); // 打开模式可省略 
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

    writeFile << s0 << ',' 
    << a[0] << ',' 
    << a[1] << ','
    << a[2] << ','
    << a[3] << ','
    << a[4] << ','
    << a[5] << ','
    << a[6] << ','
    << a[7] << std::endl;
    writeFile.close();
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

} //namespace referenceLine
