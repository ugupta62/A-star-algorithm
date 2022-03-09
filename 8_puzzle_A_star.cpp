#include <iostream>
#include <vector>
#include <map>
#include<bits/stdc++.h> 
#include <cmath>
#include <string>
using namespace std; 

class   Puzzle{
  public:
    vector<int> dimension;
    vector<vector <int>> initial_state, final_state; 
    map<vector<vector<int>>, int> mapping_from_state_to_node;
    map<int, vector<vector<int>>> mapping_from_node_to_state;
  
    vector<int> get_coord(vector<vector<int>> state, int piece=0){
      int col_index;
      for (int i=0;i<dimension[0];i++){
        // cout<<"correct1"<<endl;
        auto itr  = find(state[i].begin(),state[i].end(),piece);
        // cout<<"correct2"<<endl;
        col_index = itr - state[i].begin();
        if (col_index<state.size()){
          return {i,col_index};
        }
      }
      cout<<"error in puzzle"<<endl;
      return {-1,-1};   
    }

    Puzzle(vector<vector <int>> init_state,vector<vector <int>> fin_state, vector<int> dimen){
        dimension = dimen;
        initial_state = init_state;
        final_state = fin_state;
        mapping_from_state_to_node[initial_state] =1;
        mapping_from_node_to_state[1] = initial_state;
        mapping_from_state_to_node[final_state] = 0;
        mapping_from_node_to_state[0]=final_state;
      
    }

  // A* algorithm
  vector<int> A_star(int heuristic_type){
    int cost = 0;
    int count = 1;
    vector<int> expanded, next_adjacent_coord;
    vector<string> direction;
    vector<vector<int>> new_state, curr_state = initial_state;
    int zero_x, zero_y;
    priority_queue<vector<int>,vector<vector<int>>, greater<vector<int>> > frontier;
    vector<int> optimal_path = {count};
    frontier.push({cost, count});
    while (true){
      if (frontier.size() == 0){
        cout<<"Failed to find a path"<<endl;
        return {};
      }
      else{
        if (frontier.top().back() == mapping_from_state_to_node[final_state]){
          optimal_path = frontier.top();
          optimal_path.erase(optimal_path.begin()); // remove first element as it is the cost
          cout<<expanded.size()<<" number_of_nodes expanded"<<endl;
          return optimal_path;
        }
        expanded.push_back(frontier.top().back());
        optimal_path = frontier.top();
        optimal_path.erase(optimal_path.begin()); // remove first element as it is the cost
        frontier.pop();
        int curr_node = expanded.back();
        curr_state = mapping_from_node_to_state[curr_node];
        direction = next_possible_movement_direction(curr_state);
        vector<int> zero_coord =  get_coord(curr_state);
        for (int i=0; i<direction.size();i++){
          zero_x = zero_coord[0];
          zero_y = zero_coord[1];
          new_state = curr_state;
          if(direction[i] == "up"){next_adjacent_coord = {zero_x-1,zero_y};}
          else if (direction[i] == "down"){next_adjacent_coord = {zero_x+1,zero_y};}
          else if (direction[i] == "left"){next_adjacent_coord = {zero_x,zero_y-1};}
          else {next_adjacent_coord = {zero_x,zero_y+1};}

          swap(new_state[next_adjacent_coord[0]][next_adjacent_coord[1]], new_state[zero_x][zero_y]);
          if (mapping_from_state_to_node[new_state] == 0){ // if 0, then state doesn't exist
              count+=1;
              mapping_from_state_to_node[new_state] = count;
              mapping_from_node_to_state[count] = new_state;
          };

          int node = mapping_from_state_to_node[new_state];
          if (find(expanded.begin(),expanded.end(),node) != expanded.end()){
            continue;
          }
          frontier.push(next_node(node, optimal_path, heuristic_type));
          
        } 
      }
    }
  }

  vector<string> next_possible_movement_direction(vector<vector<int>> curr_state, int piece=0){

    vector<int> zero_coord =  get_coord(curr_state);
    vector<string> direction;
    vector<int> corner1 = {0,0}, corner2 = {dimension[0]-1,0}, corner3 = {dimension[0]-1,dimension[1]-1}, corner4 = {0,dimension[1]-1};

    if (zero_coord == corner1 || zero_coord == corner2 || zero_coord == corner3 || zero_coord == corner4){
      if (zero_coord == corner1){direction = {"down","right"};}
      else if (zero_coord == corner2){direction = {"up","right"};}
      else if (zero_coord == corner3){direction = {"left","up"};}
      else {direction = {"left","down"};}
      
    }
    else if (zero_coord[0] == 0 || zero_coord[0] == dimension[0]-1 || zero_coord[1] == dimension[1]-1 || zero_coord[1] == 0){
      if (zero_coord[0] == 0){direction = {"left", "down","right"};}
      else if (zero_coord[0] == dimension[0]-1){direction = {"left","up","right"};}
      else if (zero_coord[1] == dimension[1]-1){direction = {"left","up","down"};}
      else {direction = {"right","down","up"};}
    }
    else {
      direction = {"up","left","down","right"};
    }
    return direction;
  }



  // expansion from min_cost node
  vector<int> next_node(int curr_node, vector<int> traveled_path, int heuristic_type){
    vector<vector<int>> next_node_list;
    // current puzzle state 
    traveled_path.push_back(curr_node);
    int total_path_estimated_cost = predicted_cost(traveled_path,heuristic_type);
    
    vector<int> optimal_path_with_cost;
    optimal_path_with_cost.push_back(total_path_estimated_cost); // push new node to current path
    optimal_path_with_cost.insert(optimal_path_with_cost.end(),traveled_path.begin(), traveled_path.end()); // add cost to the new path
    return optimal_path_with_cost;
  }
  // heuristic
  int predicted_cost(vector<int> traveled_path, int heuristic_type=1){
    int heuristic;
    int count = 0;
    int manhattan_dist =0;

    vector<vector <int>> curr_state = mapping_from_node_to_state[traveled_path.back()];

    switch (heuristic_type){
      case 1:  
        for(int i=0;i<initial_state.size();i++){
            for(int j=0;j<initial_state[0].size();j++){
                if(curr_state[i][j]!=final_state[i][j]){count++;}
            }
        }
        heuristic = count;
        break;
      case 2:
         for(int i=0;i<final_state.size();i++){
            for(int j=0;j<final_state[0].size();j++){
                if(curr_state[i][j]!=final_state[i][j]){
                 manhattan_dist+= abs(get_coord(final_state,curr_state[i][j])[0] - i) + abs(get_coord(final_state,curr_state[i][j])[1] - j);
                }
            }
        }
        heuristic = manhattan_dist;
    }
   
    // // cost of traveling to adjacent node is unity
    int actual_cost = traveled_path.size()-1;
    return actual_cost + heuristic;
  }
};


int main() {
  vector<int> dimen ={3,3};
  //vector<vector <int>> initial_state = {{5,4,0},{6,1,8},{7,3,2}};
  vector<vector <int>> initial_state = {{7,2,4},{5,0,6},{8,3,1}};
  vector<vector <int>> final_state = {{1,2,3},{4,5,6},{7,8,0}};
  // heuristic_type = 1 is number of misplaced tiles
  // heuristic_type = 2 is manhattan distance of the current piece to the final place of the piece in the puzzle
  int heuristic_type = 2;
  Puzzle puzzle(initial_state,final_state,dimen);
  cout<<endl;
  vector<int> optimal_path = puzzle.A_star(heuristic_type);
  cout<<"optimal_path_count -> "<<optimal_path.size()-1<<" "<<endl;;
  for(int i = 0; i<optimal_path.size(); i++){
    vector<vector<int>> state = puzzle.mapping_from_node_to_state[optimal_path[i]];
    for (int j=0;j<dimen[0];j++){
      for (int k=0;k<dimen[1];k++){
        cout<<"| "<<state[j][k]<<" ";
      }
      cout<<"|"<<endl;
  }
  cout<<endl;
  }
  return 0;
}

