//
// Created by ustb on 19-7-8.
//

#include "latticePlanner.h"


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
    tmp.y_ = this->y_ + p.y_;
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

bool Node::operator==(Node p){
    if (this->x_ == p.x_ && this->y_ == p.y_) return true;
    return false;
}

bool Node::operator!=(Node p){
    if (this->x_ != p.x_ || this->y_ != p.y_) return true;
    return false;
}

bool compare_cost::operator()(Node& p1, Node& p2){
    // Can modify this to allow tie breaks based on heuristic cost if required
    if (p1.cost_ >= p2.cost_) return true;
    return false;
}

/**
 * Possible motion primitives in lattice
 */
std::vector<std::array<double, 2>> GetMotion(){

    std::vector<std::array<double, 2>> motion;
    for(int i=0; i<lateral_num; i++){
        std::array<double, 2> tmp_motion = {longi_step, (i - LeftSampleNum) * latera_step + refLineRho};
        motion.push_back(tmp_motion);
    }

    return motion;
}


void PrintPath(std::vector<Node> path_vector, Node start, Node goal, std::vector<std::vector<int>> &grid){
    if(path_vector[0].id_ == -1){
        std::cout << "No path exists" << std::endl;

        return;
    }
    std::cout << "Path (goal to start):" << std::endl;
    int i = 0;
    for(i = 0; i < path_vector.size(); i++){
        if(goal == path_vector[i]) break;
    }
    path_vector[i].PrintStatus();
    grid[path_vector[i].x_][path_vector[i].y_] = 3;
    while(path_vector[i].id_!=start.id_){
        if(path_vector[i].id_ == path_vector[i].pid_) break;
        for(int j = 0; j < path_vector.size(); j++){
            if(path_vector[i].pid_ == path_vector[j].id_){
                i=j;
                path_vector[j].PrintStatus();
                grid[path_vector[j].x_][path_vector[j].y_] = 3;
            }
        }
    }
    grid[start.x_][start.y_] = 3;

}

void PrintCost(std::vector<std::vector<int>> &grid, std::vector<Node> point_list){
    int n = grid.size();
    std::vector<Node>::iterator it_v;
    for(int i=0;i<n;i++){
        for(int j=0;j<n;j++){
            for(it_v = point_list.begin(); it_v != point_list.end(); ++it_v){
                if(i == it_v->x_ && j== it_v->y_){
                    std::cout << std::setw(10) <<it_v->cost_ << " , ";
                    break;
                }
            }
            if(it_v == point_list.end())
                std::cout << std::setw(10) << "  , ";
        }
        std::cout << std::endl << std::endl;
    }
}


int Dijkstra::calIndex(Node p) {

    double id = (p.y_ - (refLineRho-LeftSampleNum*latera_step)) / latera_step + ((p.x_ - start_.x_) / longi_step - 1.0) * lateral_num + 1.0;
    return int(id);
}


std::vector<Node> Dijkstra::dijkstra(Node start_in){
    start_ = start_in;

    // Get possible motions
    std::vector<std::array<double, 2>> motion = GetMotion();
    open_list_.push(start_);

    // Main loop
    Node temp;
    while(!open_list_.empty()){
        Node current = open_list_.top();
        open_list_.pop();
        current.id_ = calIndex(current);

        for(auto it = motion.begin(); it!=motion.end(); ++it){
            Node new_point;
            new_point = current + *it;
            new_point.id_ = n*new_point.x_+new_point.y_;
            new_point.pid_ = current.id_;

            if(new_point == goal_){
                open_list_.push(new_point);
                break;
            }
            if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
            if(grid[new_point.x_][new_point.y_]!=0){
                continue; //obstacle or visited
            }
            open_list_.push(new_point);
        }
        closed_list_.push_back(current);
    }
    closed_list_.clear();
    Node no_path_node(-1,-1,-1,-1,-1);
    closed_list_.push_back(no_path_node);
    return closed_list_;
}

#ifdef BUILD_INDIVIDUAL
/**
* @brief Script main function. Generates start and end nodes as well as grid, then creates the algorithm object and calls the main algorithm function.
* @return 0
*/
int main(){
  int n = 11;

  std::vector<std::vector<int>> grid(n);
  std::vector<int> tmp(n);
  for (int i = 0; i < n; i++){
    grid[i] = tmp;
  }
  MakeGrid(grid);
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> distr(0,n-1); // define the range

  Node start(distr(eng),distr(eng),0,0,0,0);
  Node goal(distr(eng),distr(eng),0,0,0,0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
  //Make sure start and goal are not obstacles and their ids are correctly assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;
  PrintGrid(grid);

  Dijkstra new_dijkstra;
  std::vector<Node> path_vector = new_dijkstra.dijkstra(grid, start, goal);
  PrintPath(path_vector, start, goal, grid);

  return 0;
}
#endif