
#include "selfType.h"



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



VehicleBox::VehicleBox(){


    RearAxleMidPoint_.x=0.0;
    RearAxleMidPoint_.y=0.0;
    RearAxleMidPoint_.yaw=0.0;

    VehicleWidth_=1.8;
    VehicleLength_=4.0;

    CircumscribedCircleRadius_=sqrt(pow(VehicleWidth_/2.0,2) + pow(VehicleLength_/2.0,2));

    InscribedCircleRadius_ = VehicleWidth_/2.0;

    NumCircles_=3;
    BodyDiskRadius_ = sqrt(pow(VehicleLength_,2)/(NumCircles_*NumCircles_) + pow(VehicleWidth_,2)/4);

    DistanceBetweenDisks_ = 2*sqrt(pow(BodyDiskRadius_,2)-pow(VehicleWidth_,2)/4);


    
}
    
} // namespace name
