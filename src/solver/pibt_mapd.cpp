/*
 * pibt_mapd.cpp
 *
 * Purpose: PIBT_MAPD
 *
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
 * arxiv:1901.11282
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "pibt_mapd.h"
#include <algorithm>
#include <bits/stdc++.h>
#include <random>
#include "../util/util.h"

using namespace std;


PIBT_MAPD::PIBT_MAPD(Problem* _P) : Solver(_P)
{
  init();
}
PIBT_MAPD::PIBT_MAPD(Problem* _P, std::mt19937* _MT) : Solver(_P, _MT)
{
  init();
}
PIBT_MAPD::~PIBT_MAPD() {}
void PIBT_MAPD::init() {
  G->setRegFlg(true);
}


bool PIBT_MAPD::solve() {
  solveStart();

  cout << "program started" << endl;

  // int nodeNum = G->getNodesNum();
  // initialize priroirty
  int agentNum = A.size();

  cout << A.size() << endl;

  /*Update*/




  // for(auto a:A){
  //   // a->path = G->getPath(a->getNode(), a->getGoal());

  //   cout << a->hasTask() << endl;
  // }

  // for(int i=0; i < A.size(); i++){
  //   int conf = 0;
  //   for(int j=0; j<A.size(); j++){
  //     if(i==j) continue;
  //     conf += conflict_count(A[i]->path, A[j]->path);
  //   }
  //   // std::cout << i << " " << conf << std::endl;
  //   priority.push_back(conf);
  // }

  // std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();

  // P->heuristicTime = std::chrono::duration_cast<std::chrono::milliseconds>
  //   (en-st).count();

  /*Update*/


  for (int i = 0; i < agentNum ; ++i) {
    epsilon.push_back((float)i / agentNum);
    eta.push_back(0);
    priority.push_back(epsilon[i] + eta[i]);
    A[i]->setBeforeNode(A[i]->getNode());
  }

    // cout << " checking new tasks" << endl;
    // for(auto a:A) {
    //   if(a->hasTask())
    //     cout << a->getId() << endl;
    // }
    // cout << " Task checking stopped" << endl;

  while (!P->isSolved()) {
    // cout << " Task checking started" << endl;
    // for(auto a:A) {
    //   if(!a->hasTask())
    //     cout << a->getId() << endl;
    // }
    // cout << "before allocate";
    bool flag = allocate();
    // cout << " checking new tasks " << flag << endl;
    // cout << " Task checking stopped" << endl;
    // cout << "before update" << endl;
    update(flag);
    // cout << "before P->update" << endl;
    P->update();
    if (P->getTimestep() >= P->getTimestepLimit()) {
      // cout << "break" << endl;
      break;
    }
  }

  solveEnd();
  return true;
}

bool PIBT_MAPD::allocate() {
  // cout << P->allocated() << endl;
  bool flag = false;
  if (P->allocated()) return flag;
  auto T = P->getT();
  Graph* _G = G;

  for (auto a : A) {
    if (a->hasTask()) continue;
    if (T.empty()) {
      a->releaseGoalOnly();
      // std::cout << "goal released " << a->getId() << " " << std::endl;
    } else {
      auto v = a->getNode();
      auto itr = std::min_element(T.begin(), T.end(),
                                  [v, _G] (Task* t1, Task* t2) {
                                    return _G->dist(t1->getG()[0], v)
                                      < _G->dist(t2->getG()[0], v);
                                  });
      a->setGoal((*itr)->getG()[0]);
      flag = true;
      // cout << a->getId() << " " << a->hasTask() << endl;
    }
  }

  return flag;
}

void PIBT_MAPD::update(bool flag) {
  updatePriority(flag);
  // cout << "program is running" << endl;

  std::vector<float> PL(priority.size());  // priority list
  std::copy(priority.begin(), priority.end(), PL.begin());

  Nodes CLOSE_NODE;
  Agents OPEN_AGENT(A.size());
  std::copy(A.begin(), A.end(), OPEN_AGENT.begin());

  // choose one agent with the highest priority
  auto itr = std::max_element(PL.begin(), PL.end());
  int index = std::distance(PL.begin(), itr);
  Agent* a = OPEN_AGENT[index];

  // cout << " before priority loop" << endl;

  while (!OPEN_AGENT.empty()) {
    // priorityInheritance(a, CLOSE_NODE, OPEN_AGENT, PL);

    /*update*/
    // a->getNode()->getId() != a->getGoal()->getId()
    // cout << " before if" << endl;
    // cout << a->getNode()->getId() << " " << a->hasGoal() << endl;
    /* some agents do not have any goal means they do not have any task */
    if(a->hasGoal() && (a->getNode()->getId() != a->getGoal()->getId())){
      // cout << " in PI" << endl;
      priorityInheritance(a, CLOSE_NODE, OPEN_AGENT, PL);
    }
    else
    {
      // cout << " not in PI" << endl;
      auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
      PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
      OPEN_AGENT.erase(itr);
    }
    // cout << "after if" << endl;

    /*update*/
    

    itr = std::max_element(PL.begin(), PL.end());
    index = std::distance(PL.begin(), itr);
    a = OPEN_AGENT[index];
    // if(*itr != 0)
      // std:: cout << "agent "<< a->getId() << " " << *itr << std::endl;
  }
}

bool PIBT_MAPD::compare(st *a, st *b) {
  float dista = new_dists(A[a->ind]->getGoal()->getIndex(), A[a->ind]->getNode()->getIndex());
  float distb = new_dists(A[b->ind]->getGoal()->getIndex(), A[b->ind]->getNode()->getIndex());

  return dista<distb;
}

void PIBT_MAPD::updatePriority(bool flag) {
  // std::cout<<"updatePriority\n";
  // update priority
  // for (int i = 0; i < A.size(); ++i) {
  //   // hasTask & not reach to goal
  //   if (A[i]->isUpdated()) {
  //     eta[i] = 0;
  //   } else if (A[i]->hasTask() && (A[i]->getNode() != A[i]->getGoal())) {
  //     eta[i] += 1;
  //   } else {
  //     eta[i] = 0;
  //   }
  //   priority[i] = eta[i] + epsilon[i];
  //   // priority[i] = getDensity(A[i]);
    // std::cout<<A[i]->getNode()->getId()<<"\n";

    // /* update */
      if(flag) {
        std::chrono::system_clock::time_point st = std::chrono::high_resolution_clock::now();
          for(auto a:A){
            a->path = getShortestPath(a->getNode(), a->getGoal());

    // cout << a->hasTask() << endl;
         }
  // cout << " before priority" << endl;

  for(int i=0; i < A.size(); i++){
    int conf = 0;
    for(int j=0; j<A.size(); j++){
      if(i==j) continue;
      conf += conflict_count(A[i]->path, A[j]->path);
    }
    priority[i]=conf;
    // std::cout << i << " " << priority[i] << std::endl;
  }
  // cout << " after priority" << endl;
  TieBreak(priority);
  
  std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();
  P->heuristicTime += std::chrono::duration_cast<std::chrono::milliseconds> (en-st).count();
  }
  else {
    for(int i=0; i<A.size(); i++){
    if(A[i]->getNode() == A[i]->getGoal())
      priority[i] = 0;
    }
  }
}

void PIBT_MAPD::TieBreak(std::vector<float>& priority) {
  map<float, bool> mp;
  // cout << " loop begin" << endl;
  for(int i=0; i<priority.size(); i++) {
    // cout << i << " " << priority[i] << endl;
    if(mp.find(priority[i]) == mp.end()){
      mp[priority[i]] = true;
      vector<st*> vec;
      vec.push_back(new st {i, priority[i]});
      // int cnt = 1;
      // cout << " loop begin" << endl;
      for(int j = i+1; j<priority.size(); j++) {
        if(priority[j] == priority[i]){
          // cnt ++;
          // cout << j << " " << priority[j] << endl;
          vec.push_back(new st {j, priority[j]});
        }
      }
      // cout << " loop end" << endl;
      if(vec.size() > 1){
      sort(vec.begin(), vec.end(), [this](st* w1, st* w2){return compare(w1, w2);});
      //do something
      float lb = priority[i] - 1;
      float offset = (float) 1/vec.size();
      // cout << " loop begin" << endl;
      for(int l = 0; l < vec.size(); l++) {
        // cout << vec[l]->conf << endl;
        priority[vec[l]->ind] = lb + offset*(l+1);
        // cout << lb << " " << offset << " " << priority[i] << " " << priority[vec[l]->ind] << endl;
      }
      // cout << " loop end" << endl;
    }
    vec.clear();
    }
  }
  // cout << " loop end" << endl;
}

float PIBT_MAPD::getDensity(Agent* a) {
  float density = 0;
  Node *v, *u;
  int d, tmp;
  v = a->getNode();
  Nodes Ci, Cj;
  Ci = G->neighbor(v);

  for (auto b : A) {
    if (b == a) continue;
    u = b->getNode();
    d = G->dist(u, v);
    if (G->dist(u, v) > 2) continue;
    tmp = - d + 2;
    Cj = G->neighbor(u);
    for (auto w : Cj) {
      if (w == v) {
        tmp += 2;
      } else if (G->dist(w, u) == 1) {
        ++tmp;
      }
    }
    density += (float)tmp / (float)Cj.size();
  }

  density /= (float)Ci.size();
  return density;
}

bool PIBT_MAPD::priorityInheritance(Agent* a,
                               Nodes& CLOSE_NODE,
                               Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE);
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT_MAPD::priorityInheritance(Agent* a,
                               Agent* aFrom,
                               Nodes& CLOSE_NODE,
                               Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE, aFrom->getNode());
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT_MAPD::priorityInheritance(Agent* a,
                               Nodes C,
                               Nodes& CLOSE_NODE,
                               Agents& OPEN_AGENT,
                               std::vector<float>& PL)
{
  // remove agent from OPEN_AGENT, Priority List
  auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
  PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
  OPEN_AGENT.erase(itr);

  // cout << a->getId() << " " << priority[a->getId()] << endl;

  Node* target;

  if(a->getNode() == a->getGoal()) a->visitedGoal = true;

  // if(a->visitedGoal)
  //   cout << "Goal for agent " << a->getId() << " is " << a->getGoal()->getId() << endl; 

  // main loop
  while (!C.empty()) {

    // choose target
    target = chooseNode(a, C);
    CLOSE_NODE.push_back(target);

    bool isAgent = false;
    for (auto b : OPEN_AGENT) {
      if (target == b->getNode()) {  // If there is an agent
        if(b->getGoal()->getId() == b->getNode()->getId()){
          // std::cout << b->getId() << " is pushed by " << a->getId() << std::endl;
        }
        if (priorityInheritance(b, a, CLOSE_NODE, OPEN_AGENT, PL)) {
          // priority inheritance success
          a->setBeforeNode(a->getNode());
          a->setNode(target);
          // if(a->visitedGoal)
          //   cout << "goal node" << a->getGoal()->getId() << " next node for agent " << a->getId() << " is " << target->getId() << endl;
          return true;
        } else {
          // priority inheritance fail
          updateC(C, target, CLOSE_NODE);
          isAgent = true;
          break;
        }
      }
    }

    // If there is an agent
    if (!isAgent) {
      a->setBeforeNode(a->getNode());
      a->setNode(target);
      return true;
    }
  }

  // failed
  a->setBeforeNode(a->getNode());
  a->setNode(a->getNode());
  return false;
}

Nodes PIBT_MAPD::createCandidates(Agent* a, Nodes CLOSE_NODE) {
  Nodes C;
  for (auto v : G->neighbor(a->getNode())) {
    if (!inArray(v, CLOSE_NODE)) C.push_back(v);
  }
  if (!inArray(a->getNode(), CLOSE_NODE)) C.push_back(a->getNode());
  return C;
}

Nodes PIBT_MAPD::createCandidates(Agent* a, Nodes CLOSE_NODE, Node* tmp) {
  CLOSE_NODE.push_back(tmp);
  Nodes C = createCandidates(a, CLOSE_NODE);
  CLOSE_NODE.erase(CLOSE_NODE.end() - 1);
  return C;
}

Node* PIBT_MAPD::chooseNode(Agent* a, Nodes C) {
  if (C.empty()) {
    std::cout << "error@PIBT_MAPD::chooseNode, C is empty" << "\n";
    std::exit(1);
  }

  // randomize
  std::shuffle(C.begin(), C.end(), *MT); //randomly ekta neighbouring node nisse... 
                                          // eta change kora jaite pare... 
  
  
  if (!a->hasGoal()) {
    // std::cout<< "do not have goal " << a->getId() << std::endl;
    if (inArray(a->getNode(), C)) {  // try to stay
      return a->getNode();
    } else {
      return C[0];  // random walk
    }
  }

  return chooseNode(a->getNode(), a->getGoal(), C, true, a->getBeforeNode());
  
}

Node* PIBT_MAPD::chooseNode(Node *v, Node *g, Nodes C, bool flag, Node *beforeNode)
{
  std::string ss = dijkstra(g->getIndex());
  Nodes cs;
  float minCost = 1000000;
  float cost;
  //  std::cout<<"\ncanditate for "<<a->getId()<<" at "<<a->getNode()->getId()<<",beforenode "<<a->getBeforeNode()->getId()<<"\n";
  for (auto v : C) {
    // std::cout<<v->getId()<<" cost: ";
    int v1 = v->getIndex();
    int g1 = g->getIndex();
    
    cost = new_dists(g1,v1);
    
    if(flag) {
    if (v == beforeNode) {
      cost += 0.5;
      //std::cout<<"0.5 added\n";
    }
    }
    // std::cout<<cost<<",";
    //cost = std::stoi(shortest_path_cost[g1][v1]);
    // std::cout<<"cost : "<<cost<<" , cost2 : "<<cost2<<"\n";
    if (cost < minCost) {
      minCost = cost;
      cs.clear();
      cs.push_back(v);
    } else if (cost == minCost) {
      cs.push_back(v);
    }
  }
  // std::cout<<"\nmin cost "<<minCost<<"\n\n";
  if (cs.size() == 1) return cs[0];

  // tie break
  bool contained;
  for (auto v : cs) {  // avoid tabu list
    contained = std::any_of(A.begin(), A.end(),
                            [v](Agent* b){ return b->getNode() == v; });
    if (!contained) return v;
  }

  return cs[0];
}


void PIBT_MAPD::updateC(Nodes& C, Node* target, Nodes CLOSE_NODE) {
  for (auto v : CLOSE_NODE) {
    auto itr2 = std::find_if(C.begin(), C.end(),
                             [v] (Node* u) { return v == u; });
    if (itr2 != C.end()) C.erase(itr2);
  }
}

std::string PIBT_MAPD::logStr() {
  std::string str;
  str += "[solver] type:PIBT_MAPD_V2\n";
  str += Solver::logStr();
  return str;
}

int PIBT_MAPD::conflict_count(Nodes p1, Nodes p2)
{
  int collision = 0;

  for (int t = 0; t < p1.size(); ++t) {
      if (t >= p2.size()) {
        if (p1[t] == p2[p2.size() - 1]) {
          ++collision;
          break;
        }
        continue;
      }
      if (p1[t] == p2[t]) {  // collision
        ++collision;
        break;
      }
      if (t > 0 && p1[t-1] == p2[t] && p1[t] == p2[t-1]) {
        ++collision;
        break;
      }
    }
  return collision;
}

Nodes PIBT_MAPD::getShortestPath(Node *v, Node *g)
{
  Nodes path;

  path.push_back(v);

  Nodes neighbor = G->neighbor(v->getId());

  while (v!=g) {
    v = chooseNode(v, g, neighbor, false, nullptr);
    path.push_back(v);
    neighbor = G->neighbor(v->getId());
  }

  path.push_back(g);

  return path;
}