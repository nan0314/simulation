#include <iostream>
#include <nav_msgs/MapMetaData.h>
#include <unordered_map>
#include "rotor_navigation/rotor_navigation.hpp"


namespace rotor_navigation{

    // Helper functions

    std::string to_string(vector<int> pos){
        std::string out = std::to_string(pos[0]) + " " + std::to_string(pos[1]);
        return out;
    }

    // Node class functions

    Node::Node() {};

    Node::Node(std::vector<int> pos, Node *prev) : pos{pos}, prev{prev} {}

    Node::Node(std::vector<int> pos, double f, double g, double h, Node *prev) : 
        pos{pos}, f{f}, g{g}, h{h}, prev{prev} {}

    vector<vector<int>> Node::getPath(){

        vector<vector<int>> path = {pos};
        Node current = *this;

        // std::cout << current.prev->pos[0] << " " << current.prev->pos[1] << std::endl;;

        while (current.prev != nullptr){
            current = *current.prev;
            path.push_back(current.pos);
        }

        return path;
    }

    bool Node::operator==(Node right){
        return pos == right.pos;
    }

    std::string Node::string(){
        std::string out = to_string(pos);
        return out;
    }

    vector<vector<int>> Node::getSuccessors(Map map){

        vector<vector<int>> successors;
        int max_r = map.get_ox();
        int min_r = map.get_ox() - map.get_height();
        int max_c = map.get_oy();
        int min_c = map.get_oy() - map.get_width();

        vector<int> rows = {pos[0] - 1, pos[0], pos[0] + 1};
        vector<int> cols = {pos[1] - 1, pos[1], pos[1] + 1};

        for (auto r : rows){
            if (r <= max_r & r >= min_r){
                for (auto c : cols){
                    if (c <= max_c & c >= min_c){
                        if (map.getPoint(r,c) != -1 & !(r == pos[0] & c == pos[1])){
                            successors.push_back({r,c});
                        }
                    }
                }
            }

        }

        return successors;
    }



    double Node::distance(vector<int> point){
        return sqrt(pow(pos[0] - point[0],2) + pow(pos[1] - point[1],2));
    }


    // Map class functions

    Map::Map() : height{0}, width{0} {}

    Map::Map(int height, int width, int ox, int oy) : height{height}, width{width}, 
        ox{ox}, oy{oy}, data{std::vector<signed char>(height * width, char(0))} {}

    Map::Map(int height, int width, int ox, int oy, std::vector<signed char> data) : 
        height{height}, width{width}, ox{ox}, oy{oy}, data{data} {}

    Map::Map(nav_msgs::OccupancyGrid map) : height{int(map.info.height)}, width{int(map.info.width)}, 
        ox{int(-map.info.origin.position.y)}, oy{int(-map.info.origin.position.x)}, data{map.data} {}

    int Map::get_ox(){
        return ox;
    }

    int Map::get_oy(){
        return oy;
    }

    int Map::get_height(){
        return height;
    }

    int Map::get_width(){
        return width;
    }

    void Map::print(){

        using std::cout;
        using std::endl;

        // Print top border
        for (int c = 0; c < width; c++){
            cout << " -";
        }

        // Print map by row
        for (int r = 0; r < height; r++){
            cout << "\n| ";
            for (int c = 0; c < width; c++){
                cout << int(data[width * r + c]) << " ";
            }
            cout << "|";
        }

        cout << "\n";

        // Print bottom border
        for (int c = 0; c < width; c++){
            cout << " -";
        }
        cout << endl;
    }

    void Map::setPoint(int x, int y, int value){
        int r = ox - x;
        int c = oy - y;
        data[width * r + c] = value;
    }

    int Map::getPoint(int x, int y){
        int r = ox - x;
        int c = oy - y;
        return data[width * r + c];
    }

    nav_msgs::OccupancyGrid Map::getGrid(){

        nav_msgs::OccupancyGrid map;
        nav_msgs::MapMetaData map_data;

        map_data.resolution = 1;
        map_data.height = height;
        map_data.width = width;
        map_data.origin.position.x = -oy;
        map_data.origin.position.y = -ox;

        map.data = data;
        map.info = map_data;

        return map;
    }

    vector<vector<int>> Map::a_star(vector<int> start, vector<int> end){

        using std::unordered_map;
        using std::string;

        // initialize open and closed lists
        Node end_node(end,nullptr);
        Node starting_node(start,0,0,0,nullptr);
        unordered_map<string,Node> open_list;
        unordered_map<string,Node> closed_list;
        unordered_map<string,Node> nodes;

        open_list[to_string(start)] = starting_node;
        nodes[to_string(start)] = starting_node;

        // initialize cost parameters (cost = r * risk + l * length)
        double r = 1.;
        double l = 1.;

        
        while(open_list.size() > 0){

            // Find the node with lowest f value and select this as current node
            Node current_node;

            double min_f = INT_MAX;
            for (auto kv : open_list){
                if (kv.second.f < min_f){
                    current_node = kv.second;
                    min_f = kv.second.f;
                }
            }

            // If current_node is the destination, then we are done.
            if (current_node==end_node){
                end_node = current_node;
                break;
            }

            // Remove current_node from the open_list 
            open_list.erase(current_node.string());

            // Add current_node to closed list
            closed_list[current_node.string()] = current_node;

            // Evaluate successor nodes
            vector<vector<int>> successors = current_node.getSuccessors(*this);
            for (auto successor : successors){

                // If already evaluated completely (in closed list) skip the node
                if (closed_list.find(to_string(successor)) != closed_list.end() ) {
                    continue;
                }

                // Calculate marginal cost
                double dist = current_node.distance(successor);
                double risk = getPoint(successor[0], successor[1]);
                double step_cost = r * risk + l * dist;

                // Calculate g score for this node on this current path
                double tempG = current_node.g + step_cost;

                // If node is new, add it to the open list. If the node is not new but is already part
                // of a better path, skip the node.
                Node node;
                if (open_list.find(to_string(successor)) != open_list.end()){
                    node = open_list[to_string(successor)];
                    Node *next_node = new Node(node.pos,nullptr);
                    if (tempG<next_node->g){
                        next_node->prev = new Node(current_node.pos, current_node.prev);
                        next_node->g = tempG;
                        next_node->h = next_node->distance(end);
                        next_node->f = next_node->g + next_node->h;
                        open_list[to_string(successor)] = *next_node;
                    }
                } else {
                    Node *next_node = new Node(successor, &nodes[to_string(current_node.pos)]);
                    next_node->g = tempG;
                    next_node->h = next_node->distance(end);
                    next_node->f = next_node->g + next_node->h;
                    open_list[to_string(successor)] = *next_node;
                    nodes[to_string(successor)] = *next_node;
                }
                
            }

        }

        vector<vector<int>> path = end_node.getPath();

        std::reverse(path.begin(), path.end());

        for (auto i : path){
            std::cout << "\n";
            for (auto j : i){
                std::cout << j << " " ;
            }
        }

        // std::cout << std::endl;

        std::cout << path.size() << std::endl;

        return path;
    }


}