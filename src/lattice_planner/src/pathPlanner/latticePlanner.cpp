//
// Created by ustb on 19-7-8.
//

#include "latticePlanner.h"
#include <queue>
#include <iostream>
#include <iomanip>
#include <array>
#include <algorithm>
#include <typeinfo>
#include <sstream>
#include <fstream>
#include <stdlib.h>


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


Dijkstra::Dijkstra(std::vector<arc_length_parameter> & coefficients, Node & start)
{
    std::cout<<"=======lattice planner is initialing==========="<<std::endl;
    // ros::param::get("/lattice_planner/lateral_num", lateral_num);
    // ros::param::get("/lattice_planner/longitudinal_step", longitudinal_step);
    // ros::param::get("/lattice_planner/lateral_step", lateral_step);
    // ros::param::get("/lattice_planner/lane_width", lane_width);
    // ros::param::get("/lattice_planner/longitudinal_num", longitudinal_num);
    // ros::param::get("/lattice_planner/s0", s0);
    // ros::param::get("/lattice_planner/lane_width", lane_width);
    // ros::param::get("/lattice_planner/obstacleHeading", obstacleHeading);

    r_circle = 1.0;
    d_circle = 2.0;
    obstacle_inflation = 1.5;
    alpha1 = 100;
    alpha2 = 1;
    alpha3 = 10;
    alpha4 = 0.0;

    longitudinal_num = 5;
    lateral_num = 9;  // 横向采样个数
    longitudinal_step = 20.0;
    lateral_step = 0.5;
    lane_width = 3.75;
    SampleNumberOnOneSide = lateral_num / 2;    // sampling number on one side of the reference line
    s0 = 0.0;
    s_max = longitudinal_num*longitudinal_step;
    s_end = s0 + s_max;
    refLineRho = lane_width * 0.5;
    obstacleHeading=0;


    // the frenet coordinates of obstacles
    
    obs1.s = 20.0;
    obs1.rho = refLineRho - 1;
    obs1.heading = obstacleHeading;

    obs2.s = 40.0;
    obs2.rho = refLineRho + 1;
    obs2.heading = obstacleHeading;
    obs3.s = 70.0;
    obs3.rho = refLineRho - 1;
    obs3.heading = obstacleHeading;
    obstacles.push_back(obs1);
    obstacles.push_back(obs2);
    obstacles.push_back(obs3);

    // 最后一列的编号
    last_column_id = {lateral_num * (longitudinal_num - 1) + 1, lateral_num * longitudinal_num};  

    coefficients_=coefficients;
    start_=start;

    start_SRho.s = start.x_;
    start_SRho.rho = start.y_;
    start_SRho.heading = obstacleHeading;

    std::cout<<"----lattice planner has been initialized---"<<std::endl;

}


std::vector<Node> Dijkstra::GetNextMotion(){

    std::vector<Node> motion;
    for(int i=0; i<lateral_num; i++){
        Node tmp_motion(longitudinal_step, (i - SampleNumberOnOneSide) * lateral_step+ refLineRho, 0.0, 0, 0);
        motion.push_back(tmp_motion);
    }

//    cout<<"-------output motion-------------"<<endl;
//    for(auto m:motion){
//        cout<<m.x_<<"-"<<m.y_<<endl;
//    }
    return motion;
}


int Dijkstra::calIndex(Node p) {

    double id = (p.y_ - (refLineRho- SampleNumberOnOneSide * lateral_step)) / lateral_step + ((p.x_ - start_.x_) /longitudinal_step - 1.0) *lateral_num + 1.0;
    return int(id);
}


bool Dijkstra::nodeIsInClosed(Node &p){
    for(auto k:closed_list_){
        if (k.id_== p.id_)  return true;

    }
    return false;
}

Node Dijkstra::minCostInOpen() {
    auto p = *open_list_.begin();
    for (auto o:open_list_){
        if(p.cost_ > o.cost_)
            p = o;
    }
    return p;
}

bool Dijkstra::NodeInOpen(Node &p, std::vector<Node>::iterator &it) {

    for (auto o=open_list_.begin();o!=open_list_.end();o++){
        if(o->id_ == p.id_){
            it = o;
            return true;
        }
    }
    return false;
}

void Dijkstra::DeleteOpenNode(Node p) {
    for (auto o=open_list_.begin();o!=open_list_.end();o++){
        if(o->id_ == p.id_){
            open_list_.erase(o);
            break;
        }
    }
}

void Dijkstra::makePlan(){

    std::vector<Node> motion = GetNextMotion();
    // std::cout<<"motion_size:"<<motion.size()<<std::endl;
    open_list_.push_back(start_);
    // cout<<open_list_.size()<<endl;

    // Main loop
    while(!open_list_.empty()){
        Node current = minCostInOpen();
        // std::cout<<"-------------current id-------------"<<std::endl;
        // std::cout<<current.id_<<std::endl;
        // std::cout<<"open_size:"<<open_list_.size()<<std::endl;
        closed_list_.push_back(current);
        for(auto i:motion){
            Node new_point;
            //            cout<<"current "<<current.x_<<endl;
            //            cout<<"current address "<<&(current)<<endl;
            new_point.x_ = current.x_ + i.x_;
            new_point.y_ = i.y_;
            new_point.id_ = calIndex(new_point);
            new_point.pid_ = current.id_;
            new_point.cost_ = total_cost(current, new_point, refLineRho, obstacles, coefficients_);

            //            cout<<"------------------newpoint.x--------"<<endl;
            //            cout<<new_point.x_<<endl;
            if (new_point.x_ > s_end) break;

            auto it = open_list_.begin();
            bool flag=nodeIsInClosed(new_point);
            if (flag) continue;
            else if (NodeInOpen(new_point, it)){
                if (it->cost_ > new_point.cost_) {
                    it->cost_ = new_point.cost_;
                    it->pid_ = current.id_;
                }
            }
            else open_list_.push_back(new_point);
        }
        DeleteOpenNode(current);
    }

}

void Dijkstra::determineGoal() {
    std::vector<Node> tmpVector;
 
    for (int i=*(last_column_id.cbegin()); i!=*(last_column_id.cbegin()+1) + 1; i++) {
        for (auto j:closed_list_){
            if (i == j.id_) {
                tmpVector.push_back(j);
                break;
            }
        }
    }

    goal_ = *tmpVector.begin();
    for (auto t:tmpVector){
        if(goal_.cost_ > t.cost_)
            goal_ = t;
    }

}


void Dijkstra::pathTrace(std::stack<Node> &path) {
    
    path.push(goal_);

    int ptr = goal_.pid_;
    while (ptr != -1) {
        for (auto c:closed_list_){
            if(c.id_ == ptr){
                path.push(c);
                ptr = c.pid_;
                break;
            }
        }
    }

}

std::vector<geometry_msgs::PoseStamped> Dijkstra::generatePath(){
    std::vector<geometry_msgs::PoseStamped> optimal_path;

    makePlan();

    determineGoal();
    std::cout<<"==============goal================="<<std::endl;
    std::cout<<"["<<goal_.x_<<","<<goal_.y_<<"]"<<std::endl;
	std::stack<lattice_planner::Node> pathNode;
    pathTrace(pathNode);

	// PoseCartesian cartesian_pose;
    geometry_msgs::PoseStamped cartesian_pose;

	while (!pathNode.empty()){
		
		lattice_planner::pose_frenet start;
        start.s = pathNode.top().x_;
        start.rho = pathNode.top().y_;
        start.heading = 0.0 * M_PI / 180.0;
        
		pathNode.pop();
		if (pathNode.empty()) break;
   		lattice_planner::pose_frenet end;
        end.s = pathNode.top().x_;
        end.rho = pathNode.top().y_;
        end.heading = 0.0 * M_PI / 180.0;
           				
		// std::cout<<"-----x,y-------------"<<std::endl;
		// std::cout<<start[0]<<","<<start[1]<<std::endl;
   		lattice_planner::CubicPolynomial cubic(start, end);
   		std::vector<lattice_planner::pose_frenet> frenet_path=cubic.computeFrenetCoordinates();
		
		// std::vector<double> s_vec=*(set.begin());
		// std::vector<double> rho_vec=*(set.begin()+1);
		// std::vector<double> theta = *(set.begin()+2);

		for(std::size_t i=0; i< frenet_path.size(); i=i+4){
			cartesian_pose=frenet_to_cartesian(frenet_path[i].s, frenet_path[i].rho, frenet_path[i].heading, coefficients_);
        	        
            optimal_path.push_back(cartesian_pose);	
		}
	}

    return optimal_path;
}

void Dijkstra::samplingNodes(std::vector<pose_frenet> & poses_frenet){
    pose_frenet tmp_pose;

	for (int i=0; i<longitudinal_num; i++){
		double x_i = (i + 1) * longitudinal_step + start_.x_;

        // std::vector<std::array<double, 3> > lateral_nodes;
		for (int j=0; j< lateral_num; j++){
			double y_i = (j - SampleNumberOnOneSide) * lateral_step+ refLineRho;
			// std::array<double, 3> tmp_node = {{x_i, y_i, 0.0 * M_PI / 180.0}};
            tmp_pose.s = x_i;
            tmp_pose.rho = y_i;
            tmp_pose.heading = 0.0 * M_PI / 180.0;
            poses_frenet.push_back(tmp_pose);
        }
        // Nodes.push_back(lateral_nodes);  
    }

}

std::vector<geometry_msgs::PoseStamped> Dijkstra::generateLattice(){
    std::vector<geometry_msgs::PoseStamped> path_lattice;

    std::vector<pose_frenet> poses_frenet;
	samplingNodes(poses_frenet);
	geometry_msgs::PoseStamped cartesian_pose;

	// 生成车辆起点到第一列采样点的图
	for( int i=0; i<lateral_num; i++){
        pose_frenet start;
        start.s = start_SRho.s;
        start.rho = start_SRho.rho;
        start.heading = start_SRho.heading;
   		pose_frenet end;
        end.s = poses_frenet[i].s;
        end.rho = poses_frenet[i].rho;
        end.heading = poses_frenet[i].heading;

   		CubicPolynomial cubic(start, end);
   		std::vector<pose_frenet> frenet_path=cubic.computeFrenetCoordinates();

		for(std::size_t i=0; i!= frenet_path.size(); i++){
			cartesian_pose = frenet_to_cartesian(frenet_path[i].s, frenet_path[i].rho, frenet_path[i].heading, coefficients_);
        	path_lattice.push_back(cartesian_pose);
		}
    } 

	// 采样点之间的图
	for (int i=0; i<longitudinal_num-1; i++ ){
		for (int j=0;j<lateral_num; j++){
            for(int q=0; q<lateral_num; q++){
                pose_frenet start;
                start.s = poses_frenet[i*lateral_num+j].s;
                start.rho = poses_frenet[i*lateral_num+j].rho;
                start.heading = poses_frenet[i*lateral_num+j].heading;                
                
   		        pose_frenet end;
                end.s = poses_frenet[(i+1)*lateral_num + q].s;
                end.rho = poses_frenet[(i+1)*lateral_num + q].rho;
                end.heading = poses_frenet[(i+1)*lateral_num + q].heading;

                // std::cout<<"---------start end----------"<<std::endl;
                // std::cout<<"start x:"<<start[0]<<"end x: "<<end[0]<<std::endl;

   		        CubicPolynomial cubic(start, end);
   		        std::vector<pose_frenet> frenet_path=cubic.computeFrenetCoordinates();

                for(std::size_t t=0; t!= frenet_path.size(); t++){
                    cartesian_pose = frenet_to_cartesian(frenet_path[t].s, frenet_path[t].rho, frenet_path[t].heading, coefficients_);
                    path_lattice.push_back(cartesian_pose);

                }           
                // std::cout<<"----s_vlatticesize----"<<std::endl;
                // std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;
            }
        }
    }
    return path_lattice;
}

}   // namespace lattice_planner

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){

    Node initState(0.0, 0.0, 0.0, 0, -1);

    //Make sure start and goal are not obstacles and their ids are correctly assigned.

    Dijkstra new_dijkstra;
    std::vector<Node> closed = new_dijkstra.dijkstra(start);
    Node goal= planning.determineGoal();
    std::vector<Node> pathNode = planning.pathTrace(goal);
    for (auto i:pathNode){
        std::cout << "node:" << i.x_ << "-" << i.y_ << std::endl;
    }

    return 0;
}
#endif