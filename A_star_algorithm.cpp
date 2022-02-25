#include <iostream>
#include <vector>
#include <map>
#include<bits/stdc++.h> 
#include <cmath>
using namespace std; 

class Map{
  public:
    // vector<int> obst_xcoord;
    // vector<int> obst_ycoord;
    vector<int> dimension;
    vector<vector<int>> grid;
    map<vector<int>, int> mapping_from_grid_to_vertices;
    map<int, vector<int>> mapping_from_node_to_grid;
    int nodes = 0;
    int edges = 0;
    int count = 0;
    int start_node = 0;
    int goal_node = 0;

    // get node from coordinates
    int get_node(int x, int y){
      return mapping_from_grid_to_vertices[{x,y}];
    }

    //get coordinates from node
    vector<int> get_coord(int node){
          return mapping_from_node_to_grid[node];
    }

    // Map with obstacles construction
    Map(vector<int> obst_xcoord, vector<int> obst_ycoord,vector<int> start_coords,vector<int> goal_coords, vector<int> dimen){
        // 0 represents unoccupied
        // 1 represents occupied
        dimension = dimen;
        nodes = dimension[0]*dimension[1] - obst_xcoord.size();
        vector<vector<int>> empty_map(dimension[0],vector<int>(dimension[1],0));
        grid = empty_map;
        for(int i = 0; i< obst_xcoord.size(); i++){
            grid[obst_xcoord[i]][obst_ycoord[i]] = 1; 
        }
        for(int i = 0; i< dimension[0]; i++){
          for(int j = 0; j< dimension[1]; j++){
            if(grid[i][j] == 1){
            continue;
            }
            mapping_from_grid_to_vertices.insert(pair<vector<int>, int>({i,j},count));
            mapping_from_node_to_grid.insert(pair<int,vector<int>>(count,{i,j}));
            count++;
          }
        }
        start_node = get_node(start_coords[0],start_coords[1]);
        goal_node = get_node(goal_coords[0],goal_coords[1]);
        
    }

    // adjacency matrix calculation
    vector<vector<int>> adjacency(){
      int count = 0;
      vector<vector<int>> adj(nodes, vector<int>(nodes,0));
      
      for(int i = 0; i <dimension[0]; i++){
        for(int j = 0; j <dimension[1]; j++){
          if(grid[i][j] == 1){
            continue;
          }
          if (i+1 <dimension[0]){ // check for not crossing the world_boundary
            if (grid[i+1][j] == 0){
                adj[get_node(i+1,j)][get_node(i,j)] = 1;
            }
          }
          if (i-1 > -1){
            if (grid[i-1][j] == 0){
                adj[get_node(i-1,j)][get_node(i,j)] = 1;
            }
          }
          if (j+1 <dimension[1]){
            if (grid[i][j+1] == 0){
                adj[get_node(i,j+1)][get_node(i,j)] = 1;
            }
          }
          if (j-1 > -1){
            if (grid[i][j-1] == 0){
                adj[get_node(i,j-1)][get_node(i,j)] = 1;
            }
          }
        }
      }
      // number of edges
      for(int i = 0; i <adj[0].size(); i++){
        for(int j = 0; j <adj[1].size(); j++){
          edges+=adj[i][j];
       }
      }
      edges = edges/2;
      return adj;
    }

    // incidence matrix calculation
    vector<vector<int>> incidence(int edges, vector<vector<int>> adj){
        vector<vector<int>> inci(nodes,vector<int>(edges,0));
        int edge =0;
        for(int i = 0; i< adj[0].size(); i++){
          for(int j = 0; j<=i; j++){
            if (adj[i][j] == 1){
              inci[i][edge] = 1;
              inci[j][edge] = 1;
              edge+=1; 
            } 
            }
          }
        return inci;
    }

  // A* algorithm
  vector<int> A_star(vector<vector<int>> graph){
  int cost = 0;
  vector<int> expanded;
  vector<int> optimal_path = {start_node};
  priority_queue<vector<int>,vector<vector<int>>, greater<vector<int>> > frontier;
  frontier.push({cost, start_node});
  while (true){
    if (frontier.size() == 0){
      cout<<"Failed to find a path"<<endl;
      return {};
    }
    else{
      expanded.push_back(frontier.top().back());
      optimal_path = frontier.top();
      optimal_path.erase(optimal_path.begin()); // remove first element as it is the cost
      frontier.pop();
      int curr_node = expanded.back();
      
      for (int i=0; i<graph.size(); i++){
        if (graph[curr_node][i] == 1){
          // check if the vertex is in expanded list
            if (find(expanded.begin(), expanded.end(), i) != expanded.end()){
              continue;
            }  
        frontier.push(next_node(i,optimal_path,graph));
        
        if (frontier.top().back() == goal_node){
             optimal_path = frontier.top();
             optimal_path.erase(optimal_path.begin()); // remove first element as it is the cost
             return optimal_path;
        }
        }  
      } 
    }
  }
  }
  // expansion from min_cost node
  vector<int> next_node(int curr_node, vector<int> traveled_path, vector<vector<int>> graph){
    vector<vector<int>> next_node_list;
    int total_path_estimated_cost = predicted_cost(traveled_path);
    traveled_path.push_back(curr_node);
    vector<int> optimal_path_with_cost;
    optimal_path_with_cost.push_back(total_path_estimated_cost); // push new node to current path
    optimal_path_with_cost.insert(optimal_path_with_cost.end(),traveled_path.begin(), traveled_path.end()); // add cost to the new path
    return optimal_path_with_cost;
  }
  // heuristic
  int predicted_cost(vector<int> traveled_path){
    vector<int> curr_coords = get_coord(traveled_path.back());
    vector<int> start_coords = get_coord(traveled_path.front());
    vector <int> goal_coords = get_coord(goal_node);
    int heuristic = sqrt(pow(curr_coords[0]-goal_coords[0],2) + pow(curr_coords[1]-goal_coords[1],2));
    // cost of traveling to adjacent node is unity
    int actual_cost = traveled_path.size()-1;
    return actual_cost + heuristic;
  }
};


int main() {
  vector<int> obst_x_coord ={0,4,5,2,3,3,0,5,6,3};
  vector<int> obst_y_coord ={0,1,1,3,3,4,5,5,5,7};
  vector<int> dimen ={7,8};
  vector<int> start_coords = {6,0};
  vector<int> goal_coords = {1,7};

 
  Map graph(obst_x_coord, obst_y_coord, start_coords, goal_coords, dimen);
  vector<vector<int>>adj_mat = graph.adjacency();
  
  // for(int i = 0; i<graph.nodes; i++){
  //       for(int j = 0; j<graph.nodes; j++){
  //       cout<<adj_mat[i][j];
  //       }
  //       cout<<endl;
  //   }
  //   cout<<endl;
  // cout<<"size of adjacency matrix :"<<adj_mat [0].size()<<"x"<<adj_mat [0].size()<<endl;;
  // cout<<endl;
  // vector<vector<int>>inci_mat = graph.incidence(graph.edges, adj_mat);
  // for(int i = 0; i<graph.nodes; i++){
  //       for(int j = 0; j<graph.edges; j++){
  //       cout<<inci_mat[i][j];
  //       }
  //       cout<<endl;
  //   }
  //   cout<<endl;
  // cout<<"size of incidence matrix :"<<inci_mat.size()<<"x"<<inci_mat[0].size()<<endl;
  cout<<endl;
  vector<int> optimal_path = graph.A_star(adj_mat);
  cout<<"optimal_path -> ";
  for(int i = 0; i<optimal_path.size(); i++){
    cout<<"("<<graph.get_coord(optimal_path[i])[0]<<","<<graph.get_coord(optimal_path[i])[1]<<"), ";
  }
  cout<<endl<<"s - start, o - goal, X - obstacle"<<endl;;
  for (int i=0;i<dimen[0];i++){
    for (int j=0;j<dimen[1];j++){
      if (graph.grid[i][j]==1){
        cout<<"|"<<" X ";
      }
      else if(find(optimal_path.begin(), optimal_path.end(), graph.get_node(i,j)) != optimal_path.end()) {
        if (graph.get_node(i,j)==graph.start_node){cout<<"|"<<" s " ; }
        else if (graph.get_node(i,j)==graph.goal_node){cout<<"|"<<" o " ; }
        else{cout<<"|"<<" * " ;}       
      }
      else {cout<<"|"<<"   ";}
    }
    cout<<"|"<<endl;
  }
  return 0;


}

