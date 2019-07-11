//
// Created by ustb on 19-7-8.
//

#include "latticePlanner.h"


struct plannerParameter {
    double s_max = 100.0;
    int longi_num = 5;
    int lateral_num = 9;  // 横向采样个数
    double longi_step = 20.0;
    double latera_step = 0.5;
    double lane_width = 3.75;
    int LeftSampleNum = lateral_num / 2;    // sampling number on one side of the reference line
    double s0 = 0.0;
    double s_end = s0 + s_max;
    double refLineRho = lane_width * 0.5;
    std::array<double, 4> start_SRho = {{s0, refLineRho, 0.0 * PI / 180.0}};

    // the frenet coordinates of obstacles
    std::vector<std::vector<double> > obs = {{20, refLineRho - 1},
                                             {40, refLineRho + 2},
                                             {70, refLineRho + 2}};
    double obstacleHeading = 0.0 * PI / 180.0;

    std::vector<int> last_column_id = {lateral_num * longi_num, lateral_num * (longi_num - 1) + 1};  // 最后一列的编号
}plannerpara;


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
    for(int i=0; i<plannerpara.lateral_num; i++){
        Node tmp_motion(plannerpara.longi_step, (i - plannerpara.LeftSampleNum) * plannerpara.latera_step
        + plannerpara.refLineRho, 0.0, 0, 0);
        motion.push_back(tmp_motion);
    }

    return motion;
}


int Dijkstra::calIndex(Node p) {

    double id = (p.y_ - (plannerpara.refLineRho- plannerpara.LeftSampleNum*plannerpara.latera_step)) / plannerpara.latera_step
            + ((p.x_ - start_.x_) /plannerpara.longi_step - 1.0) *plannerpara.lateral_num + 1.0;
    return int(id);
}


bool Dijkstra::nodeIsInClosed(Node &p){
    for(auto k:closed_list_){
        if (k.id_== p.id_)  return true;

    }
    return false;
}


std::vector<Node>::iterator Dijkstra::minCost() {
    auto it = open_list_.begin();
    for (auto o=open_list_.begin();o!=open_list_.end();o++){
        if(it->cost_ > o->cost_)
            it = o;
    }
    return it;
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


std::vector<Node> Dijkstra::dijkstra(Node start_in){
    start_ = start_in;
    cout<<typeid(start_.x_).name()<<endl;
    cout<<"start:"<<start_.x_ << " " << start_.y_<<endl;
    // Get possible motions, child nodes
    std::vector<Node> motion = GetMotion();
    cout<<"size:"<<motion.size()<<endl;
    open_list_.push_back(start_);
    cout<<open_list_.size()<<endl;

    // Main loop
    while(!open_list_.empty()){
        auto current = minCost();
        cout<<"-------------current id-------------"<<endl;
        cout<<current->id_<<endl;
        cout<<"open_size:"<<open_list_.size()<<endl;
//        current->id_ = calIndex(*current);
        open_list_.erase(current);
        closed_list_.push_back(*current);

        for(auto i:motion){
            Node new_point;

            new_point.x_ = current->x_ + i.x_;
            new_point.y_ = i.y_;
            new_point.id_ = calIndex(new_point);
            new_point.pid_ = current->id_;
            new_point.cost_ = total_cost(*current, new_point, plannerpara.refLineRho, plannerpara.obs);

            cout<<"------------------newpoint.x--------"<<endl;
            cout<<new_point.x_<<endl;
            if (new_point.x_ > plannerpara.s_end) break;

            auto it = open_list_.begin();
            bool flag=nodeIsInClosed(new_point);
            if (flag) continue;
            else if (NodeInOpen(new_point, it)){
                if (it->cost_ > new_point.cost_) {
                    it->cost_ = new_point.cost_;
                    it->pid_ = current->id_;
                }
            }
            else open_list_.push_back(new_point);

        }

    }
    return closed_list_;
}


Node minCostInVector(std::vector<Node> & nodeVector) {
    auto it = nodeVector.begin();
    for (auto o:nodeVector){
        if(it->cost_ < o.cost_)
            *it = o;
    }
    return *it;
}


Node Dijkstra::determineGoal() {
    std::vector<Node> temVector;

    for (int i=plannerpara.last_column_id[1]; i!=plannerpara.last_column_id[0] + 1; i++) {
        temVector.push_back(closed_list_[i]);
    }
    Node goal = minCostInVector(temVector);

    return goal;
}


std::vector<Node>::iterator searchInvector(std::vector<Node> & nodeVector, int ptr){

    for (auto i=nodeVector.begin();i!=nodeVector.end();i++){
        if(i->id_ == ptr)
            return i;
    }
}

std::vector<Node> Dijkstra::pathTrace(Node & p) {
    std::vector<Node> path;
    path.push_back(p);

    int ptr = p.pid_;
    while (ptr != -1) {
        auto it = searchInvector(closed_list_, ptr);
        path.push_back(*it);

        ptr = it->pid_;
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