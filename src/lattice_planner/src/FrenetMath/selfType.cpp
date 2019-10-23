
#include "selfType.h"
#include "cartesianToFrenet.h"




namespace lattice_planner{

Node::Node(double x, double y, double cost, int id, int pid){
    this->x_ = x;
    this->y_ = y;
    this->cost_ = cost;
    this->id_ = id;
    this->pid_ = pid;
}

void Node::PrintStatus(){
    std::cout << "--------------"              << std::endl
              << "Node          :"             << std::endl
              << "x             : " << x_      << std::endl
              << "y             : " << y_      << std::endl
              << "Cost          : " << cost_   << std::endl
              << "Id            : " << id_     << std::endl
              << "Pid           : " << pid_    << std::endl
              << "--------------"              << std::endl;
}

Node Node::operator+(Node p){
    Node tmp;
    tmp.x_ = this->x_ + p.x_;
    tmp.y_ = p.y_;
    tmp.cost_ = this->cost_ + p.cost_;
    return tmp;
}

Node Node::operator-(Node p){
    Node tmp;
    tmp.x_ = this->x_ - p.x_;
    tmp.y_ = this->y_ - p.y_;
    return tmp;
}

void Node::operator=(Node p){
    this->x_ = p.x_;
    this->y_ = p.y_;
    this->cost_ = p.cost_;
    this->id_ = p.id_;
    this->pid_ = p.pid_;
}

bool Node::operator==(Node p) {
    if (this->x_ == p.x_ && this->y_ == p.y_) return true;
    return false;
}

bool Node::operator!=(Node p){
    if (this->x_ != p.x_ || this->y_ != p.y_) return true;
    return false;
}


bool compare_cost::operator()(Node & p1, Node & p2){
    if (p1.cost_ >= p2.cost_) return true;
    return false;
};


VehicleDisks::VehicleDisks(CartesianPose &rearAxleMidPoint,
                            double vehicleWidth,
                            double vehicleLength,
                            double H){

    RearAxleMidPoint_ = rearAxleMidPoint;

    VehicleWidth_=vehicleWidth;
    VehicleLength_=vehicleLength;
    H_ = H;
    Distance_ = VehicleLength_/2.0 - H_;

    disk1.Catpoint.x = RearAxleMidPoint_.x;
    disk1.Catpoint.y = RearAxleMidPoint_.y;
    disk1.radius= sqrt(H_*H_ + VehicleWidth_*VehicleWidth_/4.0);

    disk2.Catpoint.x = RearAxleMidPoint_.x + Distance_ * cos(RearAxleMidPoint_.yaw);
    disk2.Catpoint.y = RearAxleMidPoint_.y + Distance_ * sin(RearAxleMidPoint_.yaw);
    disk2.radius = disk1.radius;

    disk3.Catpoint.x = RearAxleMidPoint_.x + 2 * Distance_ * cos(RearAxleMidPoint_.yaw);
    disk3.Catpoint.y = RearAxleMidPoint_.y + 2 * Distance_ * sin(RearAxleMidPoint_.yaw);
    disk3.radius = disk1.radius;

    BigDisk.Catpoint.x = disk2.Catpoint.x;
    BigDisk.Catpoint.y = disk2.Catpoint.y;
    BigDisk.radius = sqrt(VehicleLength_ * VehicleLength_ / 4.0 + VehicleWidth_*VehicleWidth_/4.0);

}


VehicleDisks::VehicleDisks(CartesianPose &rearAxleMidPoint,
                           std::vector<CubicCoefficients> *coefficients,
                           nav_msgs::Path &refline){

    RearAxleMidPoint_ = rearAxleMidPoint;

    coefficients_ = coefficients;

    VehicleWidth_=1.6;
    VehicleLength_=4.0;
    H_ = 1.0;
    Distance_ = VehicleLength_/2.0 - H_;

    disk1.Catpoint.x = RearAxleMidPoint_.x;
    disk1.Catpoint.y = RearAxleMidPoint_.y;
    disk1.radius= sqrt(H_*H_ + VehicleWidth_*VehicleWidth_/4.0);

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(RearAxleMidPoint_.yaw);
    geometry_msgs::PoseWithCovarianceStamped diskStamped1;
    diskStamped1.pose.pose.position.x = disk1.Catpoint.x; 
    diskStamped1.pose.pose.position.y = disk1.Catpoint.y;
    diskStamped1.pose.pose.orientation = q;
    FrenetPose frtdisk1 = CartesianToFrenet(diskStamped1, refline, *coefficients_);
    disk1.FrtPoint.s = frtdisk1.s;
    disk1.FrtPoint.rho = frtdisk1.rho;

    disk2.Catpoint.x = RearAxleMidPoint_.x + Distance_ * cos(RearAxleMidPoint_.yaw);
    disk2.Catpoint.y = RearAxleMidPoint_.y + Distance_ * sin(RearAxleMidPoint_.yaw);
    disk2.radius = disk1.radius;

    geometry_msgs::PoseWithCovarianceStamped diskStamped2;
    diskStamped2.pose.pose.position.x = disk2.Catpoint.x; 
    diskStamped2.pose.pose.position.y = disk2.Catpoint.y;
    diskStamped2.pose.pose.orientation = q;
    FrenetPose frtdisk2 = CartesianToFrenet(diskStamped2, refline, *coefficients_);
    disk2.FrtPoint.s = frtdisk2.s;
    disk2.FrtPoint.rho = frtdisk2.rho;

    disk3.Catpoint.x = RearAxleMidPoint_.x + 2 * Distance_ * cos(RearAxleMidPoint_.yaw);
    disk3.Catpoint.y = RearAxleMidPoint_.y + 2 * Distance_ * sin(RearAxleMidPoint_.yaw);
    disk3.radius = disk1.radius;

    geometry_msgs::PoseWithCovarianceStamped diskStamped3;
    diskStamped3.pose.pose.position.x = disk3.Catpoint.x; 
    diskStamped3.pose.pose.position.y = disk3.Catpoint.y;
    diskStamped3.pose.pose.orientation = q;
    FrenetPose frtdisk3 = CartesianToFrenet(diskStamped3, refline, *coefficients_);
    disk3.FrtPoint.s = frtdisk3.s;
    disk3.FrtPoint.rho = frtdisk3.rho;

    BigDisk.Catpoint.x = disk2.Catpoint.x;
    BigDisk.Catpoint.y = disk2.Catpoint.y;
    BigDisk.radius = sqrt(VehicleLength_ * VehicleLength_ / 4.0 + VehicleWidth_*VehicleWidth_/4.0);
    BigDisk.FrtPoint.s = disk2.FrtPoint.s;
    BigDisk.FrtPoint.rho = disk2.FrtPoint.rho;

}
    

VehicleDisk VehicleDisks::getDisk1() const {

    return disk1;
}


VehicleDisk VehicleDisks::getDisk2() const{
    return disk2;
}


VehicleDisk VehicleDisks::getDisk3() const{
    return disk3;
}

VehicleDisk VehicleDisks::getBigDisk() const{
    return BigDisk;
}


} // namespace name
