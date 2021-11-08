#pragma once

#include "../problem/problem.h"
#include <vector>
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>
#include <list>
#include <stack>


class Solver {
private:
  double elapsedTime;

protected:
  Problem* P;
  std::mt19937* MT;

  Agents A;
  Graph* G;
  std::string directory_name;
  Eigen::MatrixXi dists,new_dists;

  std::vector<std::vector<std::string> > dataList;
  std::unordered_map<int,std::vector<std::string>> shortest_path_cost;

  void init();
  int getMaxLengthPaths(Paths& paths);
  void formalizePath(Paths& paths);
  int pathDist(Node* v, Node* u);
  int pathDist(Node* s, Node* g, Nodes &prohibited);
  std::vector<Agents> findAgentBlock();
  static std::string getKey(int t, Node* v);
  static std::string getKey(AN* n);

  std::vector<bool> is_visit;

  virtual void solveStart();
  virtual void solveEnd();
  std::chrono::system_clock::time_point startT;
  std::chrono::system_clock::time_point endT;

public:
  Solver(Problem* _P);
  Solver(Problem* _P, std::mt19937* _MT);
  ~Solver();

  void WarshallFloyd(std::string flieName);
  void SaveDataToFile(std::string folder_name);
  std::string dijkstra(int src_node);
  bool djikstra_source_to_goal(Agent *a, Node *s, Node *g);
  void collectCost(int src_node);
  void setDirectory(std::string folder_name);

  virtual bool solve() { return false; };
  double getElapsed() { return elapsedTime; };

  virtual std::string logStr();
};
