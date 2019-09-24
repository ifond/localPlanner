//
// Created by ustb on 19-7-12.
//

#include "cartesianToFrenet.h"

#include <iostream>
#include <string>
#include <cmath>
#include <iterator>
#include <limits>
#include <tf/transform_datatypes.h>


namespace lattice_planner{

KDNode::KDNode() = default;

KDNode::KDNode(const point_t &pt, const size_t &idx_, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    x = pt;
    index = idx_;
    left = left_;
    right = right_;
}

KDNode::KDNode(const pointIndex &pi, const KDNodePtr &left_,
               const KDNodePtr &right_) {
    x = pi.first;
    index = pi.second;
    left = left_;
    right = right_;
}

KDNode::~KDNode() = default;

double KDNode::coord(const size_t &idx) { return x.at(idx); }
KDNode::operator bool() { return (!x.empty()); }
KDNode::operator point_t() { return x; }
KDNode::operator size_t() { return index; }
KDNode::operator pointIndex() { return pointIndex(x, index); }

KDNodePtr NewKDNodePtr() {
    KDNodePtr mynode = std::make_shared< KDNode >();
    return mynode;
}

inline double dist2(const point_t &a, const point_t &b) {
    double distc = 0;
    for (size_t i = 0; i < a.size(); i++) {
        double di = a.at(i) - b.at(i);
        distc += di * di;
    }
    return distc;
}

inline double dist2(const KDNodePtr &a, const KDNodePtr &b) {
    return dist2(a->x, b->x);
}

comparer::comparer(size_t idx_) : idx{idx_} {};

inline bool comparer::compare_idx(const pointIndex &a,  //
                                  const pointIndex &b   //
) {
    return (a.first.at(idx) < b.first.at(idx));  //
}

inline void sort_on_idx(const pointIndexArr::iterator &begin,  //
                        const pointIndexArr::iterator &end,    //
                        size_t idx) {
    comparer comp(idx);
    comp.idx = idx;

    using std::placeholders::_1;
    using std::placeholders::_2;

    std::sort(begin, end, std::bind(&comparer::compare_idx, comp, _1, _2));
}

using pointVec = std::vector< point_t >;

KDNodePtr KDTree::make_tree(const pointIndexArr::iterator &begin,  //
                            const pointIndexArr::iterator &end,    //
                            const size_t &length,                  //
                            const size_t &level                    //
) {
    if (begin == end) {
        return NewKDNodePtr();  // empty tree
    }

    size_t dim = begin->first.size();

    if (length > 1) {
        sort_on_idx(begin, end, level);
    }

    auto middle = begin + (length / 2);

    auto l_begin = begin;
    auto l_end = middle;
    auto r_begin = middle + 1;
    auto r_end = end;

    size_t l_len = length / 2;
    size_t r_len = length - l_len - 1;

    KDNodePtr left;
    if (l_len > 0 && dim > 0) {
        left = make_tree(l_begin, l_end, l_len, (level + 1) % dim);
    } else {
        left = leaf;
    }
    KDNodePtr right;
    if (r_len > 0 && dim > 0) {
        right = make_tree(r_begin, r_end, r_len, (level + 1) % dim);
    } else {
        right = leaf;
    }

    // KDNode result = KDNode();
    return std::make_shared< KDNode >(*middle, left, right);
}

KDTree::KDTree(pointVec point_array) {
    leaf = std::make_shared< KDNode >();
    // iterators
    pointIndexArr arr;
    for (size_t i = 0; i < point_array.size(); i++) {
        arr.push_back(pointIndex(point_array.at(i), i));
    }

    auto begin = arr.begin();
    auto end = arr.end();

    size_t length = arr.size();
    size_t level = 0;  // starting

    root = KDTree::make_tree(begin, end, length, level);
}

KDNodePtr KDTree::nearest_(const KDNodePtr &branch,  //
                            const point_t &pt,        //
                            const size_t &level,      //
                            const KDNodePtr &best,    //
                            const double &best_dist){
    double d, dx, dx2;

    if (!bool(*branch)) {
        return NewKDNodePtr();  // basically, null
    }

    point_t branch_pt(*branch);
    size_t dim = branch_pt.size();

    d = dist2(branch_pt, pt);
    dx = branch_pt.at(level) - pt.at(level);
    dx2 = dx * dx;

    KDNodePtr best_l = best;
    double best_dist_l = best_dist;

    if (d < best_dist) {
        best_dist_l = d;
        best_l = branch;
    }

    size_t next_lv = (level + 1) % dim;
    KDNodePtr section;
    KDNodePtr other;

    // select which branch makes sense to check
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    // keep nearest neighbor from further down the tree
    KDNodePtr further = nearest_(section, pt, next_lv, best_l, best_dist_l);
    if (!further->x.empty()) {
        double dl = dist2(further->x, pt);
        if (dl < best_dist_l) {
            best_dist_l = dl;
            best_l = further;
        }
    }
    // only check the other branch if it makes sense to do so
    if (dx2 < best_dist_l) {
        further = nearest_(other, pt, next_lv, best_l, best_dist_l);
        if (!further->x.empty()) {
            double dl = dist2(further->x, pt);
            if (dl < best_dist_l) {
                best_dist_l = dl;
                best_l = further;
            }
        }
    }

    return best_l;
};

// default caller
KDNodePtr KDTree::nearest_(const point_t &pt) {
    size_t level = 0;
    // KDNodePtr best = branch;
    double branch_dist = dist2(point_t(*root), pt);
    return nearest_(root,          // beginning of tree
                    pt,            // point we are querying
                    level,         // start from level 0
                    root,          // best is the root
                    branch_dist);  // best_dist = branch_dist
};

point_t KDTree::nearest_point(const point_t &pt) {
    return point_t(*nearest_(pt));
};
size_t KDTree::nearest_index(const point_t &pt) {
    return size_t(*nearest_(pt));
};

pointIndex KDTree::nearest_pointIndex(const point_t &pt) {
    KDNodePtr Nearest = nearest_(pt);
    return pointIndex(point_t(*Nearest), size_t(*Nearest));
}

pointIndexArr KDTree::neighborhood_(const KDNodePtr &branch,          //
                                    const point_t &pt,                //
                                    const double &rad,                //
                                    const size_t &level) {
    double d, dx, dx2;

    if (!bool(*branch)) {
        // branch has no point, means it is a leaf,
        // no points to add
        return pointIndexArr();
    }

    size_t dim = pt.size();

    double r2 = rad * rad;

    d = dist2(point_t(*branch), pt);
    dx = point_t(*branch).at(level) - pt.at(level);
    dx2 = dx * dx;

    pointIndexArr nbh, nbh_s, nbh_o;
    if (d <= r2) {
        nbh.push_back(pointIndex(*branch));
    }

    //
    KDNodePtr section;
    KDNodePtr other;
    if (dx > 0) {
        section = branch->left;
        other = branch->right;
    } else {
        section = branch->right;
        other = branch->left;
    }

    nbh_s = neighborhood_(section, pt, rad, (level + 1) % dim);
    nbh.insert(nbh.end(), nbh_s.begin(), nbh_s.end());
    if (dx2 < r2) {
        nbh_o = neighborhood_(other, pt, rad, (level + 1) % dim);
        nbh.insert(nbh.end(), nbh_o.begin(), nbh_o.end());
    }

    return nbh;
};

pointIndexArr KDTree::neighborhood(const point_t &pt,               //
                                    const double &rad) {
    size_t level = 0;
    return neighborhood_(root, pt, rad, level);
}

pointVec KDTree::neighborhood_points(const point_t &pt,                 //
                                    const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    pointVec nbhp;
    nbhp.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhp.begin(),
                   [](pointIndex x) { return x.first; });
    return nbhp;
}

indexArr KDTree::neighborhood_indices(const point_t &pt,                  //
                                        const double &rad) {
    size_t level = 0;
    pointIndexArr nbh = neighborhood_(root, pt, rad, level);
    indexArr nbhi;
    nbhi.resize(nbh.size());
    std::transform(nbh.begin(), nbh.end(), nbhi.begin(),
                   [](pointIndex x) { return x.second; });
    return nbhi;
}


pose_frenet CartesianToFrenet(const geometry_msgs::PoseWithCovarianceStamped &pose_, 
                                nav_msgs::Path &refline_, 
                                std::vector<arc_length_parameter> &coefficients_){

    ROS_INFO("cartesian to frenet is begining...");

    pose_frenet pose_frenet_;

    tf::Quaternion q;
    tf::quaternionMsgToTF(pose_.pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double cartesian_yaw=yaw;
    // double cartesian_yaw=M_PI/2;

    pointVec points;
    point_t pt;

    for(auto i:refline_.poses){
        pt = {i.pose.position.x, i.pose.position.y};
        points.push_back(pt);

    }
    ROS_INFO("refPath length: %d", refline_.poses.size());
    KDTree tree(points);

    point_t point_;
    pointIndex pointAndId_;  // record the mapping point in the reference line

    // std::cout << "nearest test\n";

    point_ = {pose_.pose.pose.position.x, pose_.pose.pose.position.y};
    // point_ = {10, 50};


    // lattice_planner::point_t nearst_p = tree.nearest_point(point_);
    // ROS_INFO("nearst position: %f, %f", nearst_p[0], nearst_p[1]);

    // record the nearst point in the reference path waypoints
    pointIndex pointAndId = tree.nearest_pointIndex(point_);
    point_t nearst_p2 = pointAndId.first;
    size_t nearstPointID = pointAndId.second;

    // ROS_INFO("nearst position: %f, %f, id: %d", nearst_p2[0], nearst_p2[1], nearstPointID);

    // To calculate the mapping point in the reference line, there are two methods:
    // 1. kd-tree, search the nearst point in the reference line
    // 2. nonlinear optimization, the solvers can be ceres or ipopt

    if (nearstPointID >0 and nearstPointID <(refline_.poses.size()-1)){
        arc_length_parameter currentPointPara = coefficients_[nearstPointID];
        arc_length_parameter lastPointPara = coefficients_[nearstPointID-1];
        arc_length_parameter nextPointPara = coefficients_[nearstPointID+1];

        double step = 0.02; // search step is 0.01[m]
        int n = int((currentPointPara.s - lastPointPara.s) / step);

        pointVec last_points;

        for (int i_n=0;i_n!=n;i_n++){
            double s = lastPointPara.s+i_n*step;
            double x = lastPointPara.a0 + lastPointPara.a1 * s + lastPointPara.a2 * s * s + lastPointPara.a3 * s * s * s;
            double y = lastPointPara.b0 + lastPointPara.b1 * s + lastPointPara.b2 * s * s + lastPointPara.b3 * s * s * s;
            point_t last_pt = {x, y};
            last_points.push_back(last_pt);
        }

        for (int i_n=0;i_n!=n;i_n++){
            double s = currentPointPara.s+i_n*step;
            double x = currentPointPara.a0 + currentPointPara.a1 * s + currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
            double y = currentPointPara.b0 + currentPointPara.b1 * s + currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
            point_t last_pt = {x, y};
            last_points.push_back(last_pt);
        }

        KDTree tmp_tree(last_points);

        // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

        // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
        pointAndId_ = tmp_tree.nearest_pointIndex(point_);
        point_t mapping_p2 = pointAndId_.first;
        size_t nearstPointID_ = pointAndId_.second;
        point_t mapping_pNeighbour = last_points[nearstPointID_+1];
        int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

        double mapping_distance = sqrt(pow(point_[0]-mapping_p2[0], 2)+pow(point_[1]-mapping_p2[1], 2));
        double mapping_s = lastPointPara.s+nearstPointID_*step;
        double mapping_heading;
        double d_x,d_y;
        if(mapping_s < currentPointPara.s){
            d_x = lastPointPara.a1 + 2 * lastPointPara.a2 * mapping_s + 3 * lastPointPara.a3 * mapping_s * mapping_s;
            d_y = lastPointPara.b1 + 2 * lastPointPara.b2 * mapping_s + 3 * lastPointPara.b3 * mapping_s * mapping_s;

        }

        else{
            d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * mapping_s + 3 * currentPointPara.a3 * mapping_s * mapping_s;
            d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * mapping_s + 3 * currentPointPara.b3 * mapping_s * mapping_s;
        }

        double theta_refline = std::atan2(d_y, d_x);
        mapping_heading = cartesian_yaw-theta_refline; 

        pose_frenet_.s = mapping_s;
        pose_frenet_.rho = mapping_distance*flag;
        pose_frenet_.heading = mapping_heading;


    }
    else if (nearstPointID<1)
    {
        /* code */
        arc_length_parameter currentPointPara = coefficients_[nearstPointID];
        arc_length_parameter nextPointPara = coefficients_[nearstPointID+1];

        double step = 0.02; // search step is 0.01[m]
        int n = int((nextPointPara.s - currentPointPara.s) / step);

        pointVec last_points;
        for (int i_n=0;i_n!=n;i_n++){
            double s = currentPointPara.s+i_n*step;
            double x = currentPointPara.a0 + currentPointPara.a1 * s + currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
            double y = currentPointPara.b0 + currentPointPara.b1 * s + currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
            point_t last_pt = {x, y};
            last_points.push_back(last_pt);
        }

        KDTree tmp_tree(last_points);

        // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

        // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
        pointAndId_ = tmp_tree.nearest_pointIndex(point_);
        point_t mapping_p2 = pointAndId_.first;
        size_t nearstPointID_ = pointAndId_.second;
        point_t mapping_pNeighbour = last_points[nearstPointID_+1];
        int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

        double mapping_distance = sqrt(pow(point_[0]-mapping_p2[0], 2)+pow(point_[1]-mapping_p2[1], 2));
        double mapping_s = currentPointPara.s+nearstPointID_*step;
        double mapping_heading;
        double d_x,d_y;
        d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * mapping_s + 3 * currentPointPara.a3 * mapping_s * mapping_s;
        d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * mapping_s + 3 * currentPointPara.b3 * mapping_s * mapping_s;
        double theta_refline = std::atan2(d_y, d_x);
        mapping_heading = cartesian_yaw-theta_refline; 

        pose_frenet_.s = mapping_s;
        pose_frenet_.rho = mapping_distance*flag;
        pose_frenet_.heading = mapping_heading;
    }
    else{
        arc_length_parameter currentPointPara = coefficients_[nearstPointID];
        arc_length_parameter lastPointPara = coefficients_[nearstPointID-1];
        double step = 0.02; // search step is 0.01[m]
        int n = int((currentPointPara.s - lastPointPara.s) / step);

        pointVec last_points;

        for (int i_n=0;i_n!=n;i_n++){
            double s = lastPointPara.s+i_n*step;
            double x = lastPointPara.a0 + lastPointPara.a1 * s + lastPointPara.a2 * s * s + lastPointPara.a3 * s * s * s;
            double y = lastPointPara.b0 + lastPointPara.b1 * s + lastPointPara.b2 * s * s + lastPointPara.b3 * s * s * s;
            point_t last_pt = {x, y};
            last_points.push_back(last_pt);
        }
        KDTree tmp_tree(last_points);

        // lattice_planner::point_t mapping_p = tmp_tree.nearest_point(point_);

        // ROS_INFO("nearst position: %f, %f", mapping_p[0], mapping_p[1]);
        pointAndId_ = tmp_tree.nearest_pointIndex(point_);
        point_t mapping_p2 = pointAndId_.first;
        size_t nearstPointID_ = pointAndId_.second;

        point_t mapping_pNeighbour = last_points[nearstPointID_+1];
        int flag = IsRightOrLeft(point_, mapping_p2, mapping_pNeighbour);

        double mapping_distance = sqrt(pow(point_[0]-mapping_p2[0], 2)+pow(point_[1]-mapping_p2[1], 2));
        double mapping_s = lastPointPara.s+nearstPointID_*step;
        double mapping_heading;
        double d_x,d_y;
        d_x = lastPointPara.a1 + 2 * lastPointPara.a2 * mapping_s + 3 * lastPointPara.a3 * mapping_s * mapping_s;
        d_y = lastPointPara.b1 + 2 * lastPointPara.b2 * mapping_s + 3 * lastPointPara.b3 * mapping_s * mapping_s;
        double theta_refline = std::atan2(d_y, d_x);
        mapping_heading = cartesian_yaw-theta_refline; 

        pose_frenet_.s = mapping_s;
        pose_frenet_.rho = mapping_distance * flag;
        pose_frenet_.heading = mapping_heading;
    } 

    // if (nearstPointID >0 ){
    //     arc_length_parameter currentPointPara = coefficients_[nearstPointID];
    //     arc_length_parameter lastPointPara = coefficients_[nearstPointID-1];

    //     double x = currentPointPara.a0 + currentPointPara.a1 * s + currentPointPara.a2 * s * s + currentPointPara.a3 * s * s * s;
    //     double d_x = currentPointPara.a1 + 2 * currentPointPara.a2 * s + 3 * currentPointPara.a3 * s * s;
    //     double y = currentPointPara.b0 + currentPointPara.b1 * s + currentPointPara.b2 * s * s + currentPointPara.b3 * s * s * s;
    //     double d_y = currentPointPara.b1 + 2 * currentPointPara.b2 * s + 3 * currentPointPara.b3 * s * s;

    //     double theta = std::atan2(d_y, d_x);

    //     geometry_msgs::PoseStamped refLine_pose;
    //     refLine_pose.pose.position.x = x;
    //     refLine_pose.pose.position.y = y;
    //     geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(theta);
    //     refLine_pose.pose.orientation = q;
    //     refLine_pose.header.stamp=ros::Time::now();
    //     // this_pose_stamped.header.frame_id="/my_frame";
    //     refLine_pose.header.frame_id="/map";


    // }
    // else{
    //     arc_length_parameter currentPointPara = coefficients_[nearstPointID];
    // } 
    ROS_INFO("start cartesian pose.x:%f,y:%f,yaw:%f", 
            pose_.pose.pose.position.x, pose_.pose.pose.position.y, cartesian_yaw);
    ROS_INFO("the mapping point in the reference line.x:%f,y:%f", 
            pointAndId_.first[0], pointAndId_.first[1]);
    ROS_INFO("the start Frenet pose.s:%f, rho:%f, heading: %f", 
            pose_frenet_.s, pose_frenet_.rho, pose_frenet_.heading);
    return pose_frenet_;
}


int IsRightOrLeft(point_t p, point_t mappingp_p, point_t mapping_pNeighbour){

    point_t a={mapping_pNeighbour[0]-mappingp_p[0], mapping_pNeighbour[1]-mappingp_p[1]};

    point_t b = {p[0]-mappingp_p[0], p[1]-mappingp_p[1]};

    double flag = a[0]*b[1]-a[1]*b[0];
    if(flag>0.0) return 1;
    else if(flag<0.0) return -1;
    else return 0;
    
}
   
}

#ifdef BUILD_INDIVIDUAL
int main()
{   
    lattice_planner::pointVec points;
    lattice_planner::point_t pt;

    pt = {0.0, 0.0};
    points.push_back(pt);
    pt = {1.0, 0.0};
    points.push_back(pt);
    pt = {0.0, 1.0};
    points.push_back(pt);
    pt = {1.0, 1.0};
    points.push_back(pt);
    pt = {0.5, 0.5};
    points.push_back(pt);

    lattice_planner::KDTree tree(points);

    std::cout << "nearest test\n";
    pt = {0.8, 0.2};
    auto res = tree.nearest_point(pt);
    for (double b : res) {
        std::cout << b << " ";
    }
    std::cout << '\n';

    return 0;
}
#endif