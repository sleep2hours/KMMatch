#include <iostream>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <queue>
#include <functional>

using namespace std;
int printMatrix(vector< vector<int> > matrix)
{
  for(size_t i = 0; i < matrix.size(); ++i)
  {
    for(size_t j = 0; j < matrix[i].size(); ++j)
    {
        cout << matrix[i][j] << " ";
    }
    cout << endl;
  }
  cout << endl;
}
void ConnectedComponentAnalysis(const std::vector<std::vector<int>>& graph,
                                std::vector<std::vector<int>>& components) {
  //二分图的节点数
  int num_item = static_cast<int>(graph.size());
  std::vector<int> visited;
  visited.resize(num_item, 0);
  std::queue<int> que;
  std::vector<int> component;
  component.reserve(num_item);
  components.clear();

  for (int index = 0; index < num_item; ++index) {
    if (visited[index]) {
      continue;
    }
    component.push_back(index);
    que.push(index);
    visited[index] = 1;
    while (!que.empty()) {
      int current_id = que.front();
      que.pop();
      for (size_t sub_index = 0; sub_index < graph[current_id].size();
           ++sub_index) {
        int neighbor_id = graph[current_id][sub_index];
        if (visited[neighbor_id] == 0) {
          component.push_back(neighbor_id);
          que.push(neighbor_id);
          visited[neighbor_id] = 1;
        }
      }
    }
    components.push_back(component);
    component.clear();
  }
}
void hungarian_test(vector< vector<int> > &costs_)
{  
  // init
   enum class Mark { NONE, PRIME, STAR };
  vector<bool> rows_covered_;
  vector<bool> cols_covered_;
  vector<int> star_in_col;
  vector<vector<int>> star;

  int matrix_size_ = costs_.size();
  rows_covered_.assign(matrix_size_, false);
  cols_covered_.assign(matrix_size_, false);
  star_in_col.assign(matrix_size_, 0);
  star.resize(matrix_size_, vector<int>(matrix_size_));
  

  // step1
  
  for (size_t row = 0; row < matrix_size_; ++row) {
    int min_cost = costs_[row][0];
    for (size_t col = 1; col < matrix_size_; ++col) {
      min_cost = std::min(min_cost, costs_[row][col]);
    }
    for (size_t col = 0; col < matrix_size_; ++col) {
      costs_[row][col] -= min_cost;
    }
  }

    //step2
    for (size_t row = 0; row < matrix_size_; ++row){
      if(rows_covered_[row]){
        continue;
      }
      for (size_t col = 0; col < matrix_size_; ++col){
        if(cols_covered_[col]){
          continue;
        }
        if(costs_[row][col] == 0){
          star[row][col] = 2;
          rows_covered_[row] = true;
          cols_covered_[col] = true;
        }
      }
    }

}
