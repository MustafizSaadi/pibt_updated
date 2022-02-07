#pragma once

#include "solver.h"
  struct st {
    int ind;
    float conf;
  };

  struct value_for_priority {
    int task;
    float conf;
    bool is_in_goal;
    int goal_distance;
  };


class PIBT_MAPD : public Solver {
protected:
  std::vector<float> epsilon;  // tie-breaker
  std::vector<int> eta;  // usually increment every step
  std::vector<value_for_priority> priority;  // eta + epsilon

  void init();
  bool allocate();

  virtual void updatePriority(bool flag);
  Nodes createCandidates(Agent* a, Nodes CLOSE_NODE);
  Nodes createCandidates(Agent* a, Nodes CLOSE_NODE, Node* tmp);
  bool priorityInheritance(Agent* a,
                           Nodes& CLOSE_NODE,
                           Agents& OPEN_AGENT,
                           std::vector<value_for_priority>& PL);
  bool priorityInheritance(Agent* a,
                           Agent* aFrom,
                           Nodes& CLOSE_NODE,
                           Agents& OPEN_AGENT,
                           std::vector<value_for_priority>& PL);
  virtual bool priorityInheritance(Agent* a,
                                   Nodes C,
                                   Nodes& CLOSE_NODE,
                                   Agents& OPEN_AGENT,
                                   std::vector<value_for_priority>& PL);
  virtual Node* chooseNode(Agent* a, Nodes C);
  virtual Node* chooseNode(Node *v, Node *g, Nodes C, bool flag, Node *beforeNode);
  void updateC(Nodes& C, Node* target, Nodes CLOSE_NODE);

  float getDensity(Agent* a);  // density can be used as effective prioritization
  void printFloyd();
  bool compare(st *a, st *b);
  bool max_compare(value_for_priority a, value_for_priority b);
  void TieBreak(std::vector<float>& Priority);
  int conflict_count(Nodes p1, Nodes p2); 
  Nodes getShortestPath(Node *v, Node *g);
public:
  PIBT_MAPD(Problem* _P);
  PIBT_MAPD(Problem* _P, std::mt19937* _MT);
  ~PIBT_MAPD();

  bool solve();
  virtual void update(bool flag);

  virtual std::string logStr();
};


