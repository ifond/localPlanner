//
// Created by ustb on 19-7-12.
//

#include <Eigen/Dense>
#include "referenceLine.h"
#include "../config/parameters.h"

namespace referenceLine{

std::vector<std::vector<Vec_d > > refLine_coefficients(){
    latticeParameter::referenceLineWayPoints way_points;
    Vec_d x=way_points.x;
    Vec_d y=way_points.y;
    Vec_d r_x, r_y, r_heading, r_curvature, r_s;

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

    std::vector<std::vector<Vec_d > > coefficients = SparseWayPoints(r_x, r_y, r_heading, r_s);
    return coefficients;

}

std::vector<std::vector<double> > arcLengthRefLine(std::vector<double> pose, std::vector<double> nextPose,
                                                   double s0, double sf){
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

    std::vector<double> a={{0.0, 0.0, 0.0, 0.0}}, b={{0.0, 0.0, 0.0, 0.0}};
    for (int i=0; i<a_vec.size();i++){
        a[i] = a_vec[i];
        b[i] = b_vec[i];

    }
    Vec_d s_d = {s0};
    std::vector<std::vector<double> > Tmp_coefficients = {{s_d, a, b}};
//    std::cout<<"=============tmp coefficients==========="<<std::endl;
//    std::cout<<s_d.size()<<a.size()<<b.size()<<std::endl;
    return Tmp_coefficients;

}

std::vector<std::vector<Vec_d > > SparseWayPoints(Vec_d r_x, Vec_d r_y, Vec_d r_heading, Vec_d r_s){
    std::vector<std::vector<Vec_d > > coefficients;

    for (auto i=0; i < r_x.size(); i=i+20){
        if ( (i+20) > (r_x.size() - 1)) {
            Vec_d pose = {{r_x[i], r_y[i], r_heading[i]}};
            Vec_d next_pose = {{r_x.back(), r_y.back(), r_heading.back()}};
            double s0 = r_s[i];
            double sf = r_s.back();

            std::vector<Vec_d > TmpCoefficients = arcLengthRefLine(pose, next_pose, s0, sf);
            coefficients.push_back(TmpCoefficients);
        }

        else{
            Vec_d pose = {{r_x[i], r_y[i], r_heading[i]}};
            Vec_d next_pose = {{r_x[i+20], r_y[i+20], r_heading[i+20]}};
            double s0 = r_s[i];
            double sf = r_s[i+20];

            std::vector<Vec_d > TmpCoefficients = arcLengthRefLine(pose, next_pose, s0, sf);
            coefficients.push_back(TmpCoefficients);
        }
        //        std::cout<<"---------------tmp coefficients------"<<std::endl;
//        std::cout<<TmpCoefficients.size()<<std::endl;

    }

    return coefficients;

}


def search_in_coefficients(double s, std::vector<std::vector<std::vector<double > > > efficients){
# find the id of s in the list coefficients
        s_id = 0
        for i in range(len(efficients)):
        if (s > efficients[i][0]):
        s_id = i
        continue
        else:
        break

# s_id = binary_search(efficients, s)
# print("s:", s)
        s_start, b_vector, c_vector = efficients[s_id]
# print('output_s:', efficients[s_id])

        x = b_vector[0] + b_vector[1] * s + b_vector[2] * s ** 2 + b_vector[3] * s ** 3
        d_x = b_vector[1] + 2 * b_vector[2] * s + 3 * b_vector[3] * s ** 2

        y = c_vector[0] + c_vector[1] * s + c_vector[2] * s ** 2 + c_vector[3] * s ** 3
        d_y = c_vector[1] + 2 * c_vector[2] * s + 3 * c_vector[3] * s ** 2

        theta = math.atan2(d_y, d_x)
        return x, y, theta
}



def binary_search(list, element) {
    """
    :param
    list:
    :param
    element:
    :return:
    """
    low = 0
    high = len(list) - 1
# print(__file__)
# print("coefficients length:", high)
# id = 0
    while low <= high:
    mid = (low + high) // 2
    if list[mid][0] < element:
    low = mid + 1
    if low > high: return mid
    elif
    list[mid][0] > element:

    high = mid - 1
    if low > high: return low
    else:
    return mid

# id = low
# return id
}

} //namespace referenceLine
