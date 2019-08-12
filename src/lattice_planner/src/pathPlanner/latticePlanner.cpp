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


namespace lattice_planner{

parameters param;

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


std::vector<Node> GetMotion(){

    std::vector<Node> motion;
    for(int i=0; i<param.lateral_num; i++){
        Node tmp_motion(param.longitudinal_step, (i - param.SampleNumberOnOneSide) * param.lateral_step+ param.refLineRho, 0.0, 0, 0);
        motion.push_back(tmp_motion);
    }

//    cout<<"-------output motion-------------"<<endl;
//    for(auto m:motion){
//        cout<<m.x_<<"-"<<m.y_<<endl;
//    }
    return motion;
}


int Dijkstra::calIndex(Node p) {

    double id = (p.y_ - (param.refLineRho- param.SampleNumberOnOneSide * param.lateral_step)) / param.lateral_step + ((p.x_ - start_.x_) /param.longitudinal_step - 1.0) *param.lateral_num + 1.0;
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

std::vector<Node> Dijkstra::dijkstra(){
    // start_ = start_in;
//    cout<<typeid(start_.x_).name()<<endl;
//    cout<<"start:"<<start_.x_ << " " << start_.y_<<endl;
    // Get possible motions, child nodes
    std::vector<Node> motion = GetMotion();
//    cout<<"motion_size:"<<motion.size()<<endl;
    open_list_.push_back(start_);
//    cout<<open_list_.size()<<endl;

    // Main loop
    while(!open_list_.empty()){
        Node current = minCostInOpen();
        // std::cout<<"-------------current id-------------"<<std::endl;
        // std::cout<<current.id_<<std::endl;
        // std::cout<<"open_size:"<<open_list_.size()<<endl;
        closed_list_.push_back(current);
        for(auto i:motion){
            Node new_point;
//            cout<<"current "<<current.x_<<endl;
//            cout<<"current address "<<&(current)<<endl;
            new_point.x_ = current.x_ + i.x_;
            new_point.y_ = i.y_;
            new_point.id_ = calIndex(new_point);
            new_point.pid_ = current.id_;
            new_point.cost_ = total_cost(current, new_point, param.refLineRho, param.obs, coefficients_);

//            cout<<"------------------newpoint.x--------"<<endl;
//            cout<<new_point.x_<<endl;
            if (new_point.x_ > param.s_end) break;

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
    return closed_list_;
}


Node Dijkstra::determineGoal() {
    std::vector<Node> tmpVector;

    for (int i=*(param.last_column_id.cbegin()); i!=*(param.last_column_id.cbegin()+1) + 1; i++) {
        for (auto j:closed_list_){
            if (i == j.id_) {
                tmpVector.push_back(j);
                break;
            }
        }
    }

    Node goal = *tmpVector.begin();
    for (auto t:tmpVector){
        if(goal.cost_ > t.cost_)
            goal = t;
    }
    return goal;
}


void Dijkstra::pathTrace(Node & p, std::stack<Node> &path) {
    
    path.push(p);

    int ptr = p.pid_;
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


void samplingNodes(std::vector<std::vector<std::array<double,3> > > & Nodes){
    
	for (int i=0; i<param.longitudinal_num; i++){
		double x_i = (i + 1) * param.longitudinal_step + param.start_SRho[0];

        std::vector<std::array<double, 3> > lateral_nodes;
		for (int j=0; j< param.lateral_num; j++){
			double y_i = (j - param.SampleNumberOnOneSide) * param.lateral_step+ param.refLineRho;
			std::array<double, 3> tmp_node = {{x_i, y_i, 0.0 * M_PI / 180.0}};
            lateral_nodes.push_back(tmp_node);

            // std::cout<<"lateral size()"<< lateral_nodes.size()<<std::endl;

        }
        Nodes.push_back(lateral_nodes);
        
    }
    // std::cout<<"-------------sampling nodes--------"<<std::endl;
    // std::cout<<Nodes.size()<<" "<<Nodes[0].size()<<std::endl;
    // for(auto i:Nodes){
        // for(auto j:i){
            // std::cout<<"["<<j[0]<<","<<j[1]<<"]"<<std::endl;
        // }
    // }
}

void generate_lattice(std::vector<double> &x_lattice, 
						std::vector<double> &y_lattice, 
						coefficients_type &coefficients){

    std::vector<std::vector<std::array<double, 3> > > Nodes;
	samplingNodes(Nodes);
	std::vector<double > cartesian_pose;
	// 生成车辆起点到第一列采样点的图
	for( int i=0; i<param.lateral_num; i++){
        std::array<double, 3> start= param.start_SRho;
   		std::array<double, 3> end = Nodes[0][i];
   		CubicPolynomial cubic(start, end);
   		std::vector<std::vector<double> > set=cubic.computeFrenetCoordinates();
           		
		std::vector<double> s_vec=*(set.begin());
		std::vector<double> rho_vec=*(set.begin()+1);
		std::vector<double> theta = *(set.begin()+2);

		for(std::size_t i=0; i!= s_vec.size(); i++){
			cartesian_pose=frenet_to_cartesian(s_vec[i], rho_vec[i], theta[i], coefficients);
        	x_lattice.push_back(cartesian_pose[0]);
        	y_lattice.push_back(cartesian_pose[1]);

		}
        // std::cout<<"----s_vlatticesize----"<<std::endl;
        // std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;

    } 
	// 采样点之间的图
	for (int i=0; i<param.longitudinal_num-1; i++ ){
		for (int j=0;j<param.lateral_num; j++){
            for(int q=0; q<param.lateral_num; q++){
                std::array<double, 3> start= Nodes[i][j];
   		        std::array<double, 3> end = Nodes[i+1][q];
                // std::cout<<"---------start end----------"<<std::endl;
                // std::cout<<"start x:"<<start[0]<<"end x: "<<end[0]<<std::endl;

   		        CubicPolynomial cubic(start, end);
   		        std::vector<std::vector<double> > set=cubic.computeFrenetCoordinates();
                std::vector<double> s_vec=*(set.begin());
                std::vector<double> rho_vec=*(set.begin()+1);
                std::vector<double> theta = *(set.begin()+2);

                for(std::size_t i=0; i!= s_vec.size(); i++){
                    cartesian_pose=frenet_to_cartesian(s_vec[i], rho_vec[i], theta[i], coefficients);
                    x_lattice.push_back(cartesian_pose[0]);
                    y_lattice.push_back(cartesian_pose[1]);

                }           
                // std::cout<<"----s_vlatticesize----"<<std::endl;
                // std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;
            }
        }
    }

    // std::cout<<"----s_vlatticesize----"<<std::endl;
    // std::cout<<x_lattice.size()<<" "<<y_lattice.size()<<std::endl;
}

coefficients_type pathPlanner(std::vector<double> &x_vec, 
						  std::vector<double> &y_vec){
	
    Node initState(param.start_SRho[0], param.start_SRho[1], param.start_SRho[2], 0, -1);
	coefficients_type coefficients = refLine_coefficients();
    Dijkstra planner(coefficients, initState);
    std::vector<Node> closed = planner.dijkstra();

    Node goal= planner.determineGoal();
    std::cout<<"==============goal================="<<std::endl;
    std::cout<<"["<<goal.x_<<","<<goal.y_<<"]"<<std::endl;
	std::stack<Node> pathNode;
    planner.pathTrace(goal, pathNode);

	std::vector<double > cartesian_pose;

	while (!pathNode.empty()){
		
		std::array<double, 3> start= {{ pathNode.top().x_, pathNode.top().y_, 0.0 * M_PI / 180.0}};
		pathNode.pop();
		if (pathNode.empty()) break;
   		std::array<double, 3> end = {{ pathNode.top().x_, pathNode.top().y_, 0.0 * M_PI / 180.0}};
				
		// std::cout<<"-----x,y-------------"<<std::endl;
		// std::cout<<start[0]<<","<<start[1]<<std::endl;
   		CubicPolynomial cubic(start, end);
   		std::vector<std::vector<double> > set=cubic.computeFrenetCoordinates();
		
		std::vector<double> s_vec=*(set.begin());
		std::vector<double> rho_vec=*(set.begin()+1);
		std::vector<double> theta = *(set.begin()+2);

		for(std::size_t i=0; i< s_vec.size(); i=i+4){
			cartesian_pose=frenet_to_cartesian(s_vec[i], rho_vec[i], theta[i], coefficients);
        	x_vec.push_back(cartesian_pose[0]);
        	y_vec.push_back(cartesian_pose[1]);

		}
		
		// std::cout<<"----------path s,rho----------------"<<std::endl;
		// std::cout<<set.size()<<"rho size:"<<set[1].size()<<"theta size:"<<set[2].size()<<std::endl;

	}
	
	// std::cout<<"------------s size--------------"<<std::endl;
	// std::cout<<"s.size() "<<x_vec.size()<<"s[0].size(): "<<x_vec[0].size()<<std::endl;
    // std::vector<std::vector<Vec_d > > coefficients = referenceLine::refLine_coefficients();
    // cout<<"coefficients:"<<coefficients[10][1][0]<<endl;

	// generate the path lattice graph
	// generate_latttice(x_lattice, y_lattice, coefficients);
	return coefficients;	
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