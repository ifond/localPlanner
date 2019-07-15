//
// Created by ustb on 19-7-15.
//

#include "cubicSpline.h"


namespace cubicSpline{


    Vec_f vec_diff(Vec_f input){
        Vec_f output;
        for(unsigned int i=1; i<input.size(); i++){
            output.push_back(input[i] - input[i-1]);
        }
        return output;
    }

    Vec_f cum_sum(const Vec_f & input){
        Vec_f output;
        float temp = 0;
        for(auto i:input){
            temp += i;
            output.push_back(temp);
        }
        return output;
    }


    Spline::Spline(Vec_f x, Vec_f y):x(x), y(y), nx(x.size()), h(vec_diff(x)), a(y){
        Eigen::MatrixXf A = calc_A();
        Eigen::VectorXf B = calc_B();
        Eigen::VectorXf c_eigen = A.colPivHouseholderQr().solve(B);
        float * c_pointer = c_eigen.data();
        //Eigen::Map<Eigen::VectorXf>(c, c_eigen.rows(), 1) = c_eigen;
        c.assign(c_pointer, c_pointer+c_eigen.rows());

        for(int i=0; i<nx-1; i++){
            d.push_back((c[i+1]-c[i])/(3.0*h[i]));
            b.push_back((a[i+1] - a[i])/h[i] - h[i] * (c[i+1] + 2*c[i])/3.0);
        }
    }

    float Spline::calc(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx);
        float dx = t - x[seg_id];
        return a[seg_id] + b[seg_id] * dx + c[seg_id] * dx * dx + d[seg_id] * dx * dx * dx;
    }

    float Spline::calc_d(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx-1);
        float dx = t - x[seg_id];
        return b[seg_id]  + 2 * c[seg_id] * dx + 3 * d[seg_id] * dx * dx;
    }

    float Spline::calc_dd(float t){
        if(t<x.front() || t>x.back()){
            throw std::invalid_argument( "received value out of the pre-defined range" );
        }
        int seg_id = bisect(t, 0, nx);
        float dx = t - x[seg_id];
        return 2 * c[seg_id] + 6 * d[seg_id] * dx;
    }

    Eigen::MatrixXf Spline::calc_A(){
        Eigen::MatrixXf A = Eigen::MatrixXf::Zero(nx, nx);
        A(0, 0) = 1;
        for(int i=0; i<nx-1; i++){
            if (i != nx-2){
                A(i+1, i+1) = 2 * (h[i] + h[i+1]);
            }
            A(i+1, i) = h[i];
            A(i, i+1) = h[i];
        }
        A(0, 1) = 0.0;
        A(nx-1, nx-2) = 0.0;
        A(nx-1, nx-1) = 1.0;
        return A;
    };
    Eigen::VectorXf Spline::calc_B(){
        Eigen::VectorXf B = Eigen::VectorXf::Zero(nx);
        for(int i=0; i<nx-2; i++){
            B(i+1) = 3.0*(a[i+2]-a[i+1])/h[i+1] - 3.0*(a[i+1]-a[i])/h[i];
        }
        return B;
    };

    int Spline::bisect(float t, int start, int end){
        int mid = (start+end)/2;
        if (t==x[mid] || end-start<=1){
            return mid;
        }else if (t>x[mid]){
            return bisect(t, mid, end);
        }else{
            return bisect(t, start, mid);
        }
    }


    Spline2D::Spline2D(Vec_f x, Vec_f y){
    s = calc_s(x, y);
    sx = Spline(s, x);
    sy = Spline(s, y);
    }

    Poi_f Spline2D::calc_postion(float s_t){
        float x = sx.calc(s_t);
        float y = sy.calc(s_t);
        return {{x, y}};
    }

    float Spline2D::calc_curvature(float s_t){
        float dx = sx.calc_d(s_t);
        float ddx = sx.calc_dd(s_t);
        float dy = sy.calc_d(s_t);
        float ddy = sy.calc_dd(s_t);
        return (ddy * dx - ddx * dy)/(dx * dx + dy * dy);
    }

    float Spline2D::calc_yaw(float s_t){
        float dx = sx.calc_d(s_t);
        float dy = sy.calc_d(s_t);
        return std::atan2(dy, dx);
    }

    Vec_f Spline2D::calc_s(Vec_f x, Vec_f y) {
        Vec_f ds;
        Vec_f out_s{0};
        Vec_f dx = vec_diff(x);
        Vec_f dy = vec_diff(y);

        for (unsigned int i = 0; i < dx.size(); i++) {
            ds.push_back(std::sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
        }

        Vec_f cum_ds = cum_sum(ds);
        out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
        return out_s;
    }

};

#ifdef BUILD_INDIVIDUAL
int main(){
    Vec_f x{-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    Vec_f y{0.7, -6,   5,   6.5, 0.0, 5.0, -2.0};
    Vec_f r_x;
    Vec_f r_y;
    Vec_f ryaw;
    Vec_f rcurvature;
    Vec_f rs;

    Spline2D csp_obj(x, y);
    for(float i=0; i<csp_obj.s.back(); i+=0.1){
        std::array<float, 2> point_ = csp_obj.calc_postion(i);
        r_x.push_back(point_[0]);
        r_y.push_back(point_[1]);
        ryaw.push_back(csp_obj.calc_yaw(i));
        rcurvature.push_back(csp_obj.calc_curvature(i));
        rs.push_back(i);
    }


}
#endif