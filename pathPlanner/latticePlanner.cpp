//
// Created by ustb on 19-7-8.
//

#include "latticePlanner.h"

plannerParameter parameter;


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
    for(int i=0; i<parameter.lateral_num; i++){
        Node tmp_motion(parameter.longitudinal_step, (i - parameter.SampleNumberOnOneSide) * parameter.lateral_step+ parameter.refLineRho, 0.0, 0, 0);
        motion.push_back(tmp_motion);
    }

//    cout<<"-------output motion-------------"<<endl;
//    for(auto m:motion){
//        cout<<m.x_<<"-"<<m.y_<<endl;
//    }
    return motion;
}


int Dijkstra::calIndex(Node p) {

    double id = (p.y_ - (parameter.refLineRho- parameter.SampleNumberOnOneSide * parameter.lateral_step)) / parameter.lateral_step + ((p.x_ - start_.x_) /parameter.longitudinal_step - 1.0) *parameter.lateral_num + 1.0;
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

std::vector<Node> Dijkstra::dijkstra(Node start_in){
    start_ = start_in;
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
        cout<<"-------------current id-------------"<<endl;
        cout<<current.id_<<endl;
//        cout<<"open_size:"<<open_list_.size()<<endl;
        closed_list_.push_back(current);
        for(auto i:motion){
            Node new_point;
//            cout<<"current "<<current.x_<<endl;
//            cout<<"current address "<<&(current)<<endl;
            new_point.x_ = current.x_ + i.x_;
            new_point.y_ = i.y_;
            new_point.id_ = calIndex(new_point);
            new_point.pid_ = current.id_;
            new_point.cost_ = total_cost(current, new_point, parameter.refLineRho, parameter.obs);

//            cout<<"------------------newpoint.x--------"<<endl;
//            cout<<new_point.x_<<endl;
            if (new_point.x_ > parameter.s_end) break;

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

    for (int i=*(parameter.last_column_id.cbegin()); i!=*(parameter.last_column_id.cbegin()+1) + 1; i++) {
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


std::vector<Node> Dijkstra::pathTrace(Node & p) {
    std::vector<Node> path;
    path.push_back(p);

    int ptr = p.pid_;
    while (ptr != -1) {
        for (auto c:closed_list_){
            if(c.id_ == ptr){
                path.push_back(c);
                ptr = c.pid_;
                break;
            }
        }
    }

    return path;
}


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