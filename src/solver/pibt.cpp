/*
 * pibt.cpp
 *
 * Purpose: PIBT
 *
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
 * arxiv:1901.11282
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "pibt.h"
#include <algorithm>
#include <random>
#include "../util/util.h"

using namespace std;


PIBT::PIBT(Problem* _P) : Solver(_P)
{
  init();
}
PIBT::PIBT(Problem* _P, std::mt19937* _MT) : Solver(_P, _MT)
{
  init();
}
PIBT::~PIBT() {}
void PIBT::init() {
  G->setRegFlg(true);
}


bool PIBT::solve() {
  solveStart();

  cout << "program started" << endl;

  // int nodeNum = G->getNodesNum();
  // initialize priroirty
  int agentNum = A.size();

  cout << A.size() << endl;

  /*Update*/

  std::chrono::system_clock::time_point st = std::chrono::high_resolution_clock::now();



  for(auto a:A){
    // a->path = G->getPath(a->getNode(), a->getGoal());

    cout << a->hasTask() << endl;
  }

  for(int i=0; i < A.size(); i++){
    int conf = 0;
    for(int j=0; j<A.size(); j++){
      if(i==j) continue;
      conf += conflict_count(A[i]->path, A[j]->path);
    }
    // std::cout << i << " " << conf << std::endl;
    priority.push_back(conf);
  }

  std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();

  P->heuristicTime = std::chrono::duration_cast<std::chrono::milliseconds>
    (en-st).count();

  /*Update*/


  // for (int i = 0; i < agentNum ; ++i) {
  //   epsilon.push_back((float)i / agentNum);
  //   eta.push_back(0);
  //   priority.push_back(epsilon[i] + eta[i]);
  //   A[i]->setBeforeNode(A[i]->getNode());
  // }

  while (!P->isSolved()) {
    allocate();
    update();
    P->update();
    if (P->getTimestep() >= P->getTimestepLimit()) break;
  }

  solveEnd();
  return true;
}

void PIBT::allocate() {
  if (P->allocated()) return;
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
    }
  }
}

void PIBT::update() {
  updatePriority();
  cout << "program is running" << endl;

  std::vector<float> PL(priority.size());  // priority list
  std::copy(priority.begin(), priority.end(), PL.begin());

  Nodes CLOSE_NODE;
  Agents OPEN_AGENT(A.size());
  std::copy(A.begin(), A.end(), OPEN_AGENT.begin());

  // choose one agent with the highest priority
  auto itr = std::max_element(PL.begin(), PL.end());
  int index = std::distance(PL.begin(), itr);
  Agent* a = OPEN_AGENT[index];

  while (!OPEN_AGENT.empty()) {
    if(a->getNode()->getId() != a->getGoal()->getId())
      priorityInheritance(a, CLOSE_NODE, OPEN_AGENT, PL);
    else
    {
      auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
      PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
      OPEN_AGENT.erase(itr);
    }
    

    itr = std::max_element(PL.begin(), PL.end());
    index = std::distance(PL.begin(), itr);
    a = OPEN_AGENT[index];
    // if(*itr != 0)
    //   std:: cout << "agent "<< a->getId() << " " << *itr << std::endl;
  }
}

void PIBT::updatePriority() {
  // std::cout<<"updatePriority\n";
  // update priority
  for (int i = 0; i < A.size(); ++i) {
    // hasTask & not reach to goal
    // if (A[i]->isUpdated()) {
    //   eta[i] = 0;
    // } else if (A[i]->hasTask() && (A[i]->getNode() != A[i]->getGoal())) {
    //   eta[i] += 1;
    // } else {
    //   eta[i] = 0;
    // }
    // priority[i] = eta[i] + epsilon[i];
    // priority[i] = getDensity(A[i]);
    //std::cout<<A[i]->getNode()->getId()<<"\n";

    // /* update */
    if(A[i]->getNode() == A[i]->getGoal())
      priority[i] = 0;
  }
}

float PIBT::getDensity(Agent* a) {
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

bool PIBT::priorityInheritance(Agent* a,
                               Nodes& CLOSE_NODE,
                               Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE);
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT::priorityInheritance(Agent* a,
                               Agent* aFrom,
                               Nodes& CLOSE_NODE,
                               Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE, aFrom->getNode());
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT::priorityInheritance(Agent* a,
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

Nodes PIBT::createCandidates(Agent* a, Nodes CLOSE_NODE) {
  Nodes C;
  for (auto v : G->neighbor(a->getNode())) {
    if (!inArray(v, CLOSE_NODE)) C.push_back(v);
  }
  if (!inArray(a->getNode(), CLOSE_NODE)) C.push_back(a->getNode());
  return C;
}

Nodes PIBT::createCandidates(Agent* a, Nodes CLOSE_NODE, Node* tmp) {
  CLOSE_NODE.push_back(tmp);
  Nodes C = createCandidates(a, CLOSE_NODE);
  CLOSE_NODE.erase(CLOSE_NODE.end() - 1);
  return C;
}

Node* PIBT::chooseNode(Agent* a, Nodes C) {
  if (C.empty()) {
    std::cout << "error@PIBT::chooseNode, C is empty" << "\n";
    std::exit(1);
  }

  // randomize
  std::shuffle(C.begin(), C.end(), *MT); //randomly ekta neighbouring node nisse... 
                                          // eta change kora jaite pare... 
  
  
  if (!a->hasGoal()) {
    std::cout<< "do not have goal " << a->getId() << std::endl;
    if (inArray(a->getNode(), C)) {  // try to stay
      return a->getNode();
    } else {
      return C[0];  // random walk
    }
  }


  std::string ss = dijkstra(a->getGoal()->getIndex());
  Nodes cs;
  float minCost = 1000000;
  float cost;
  Node* g = a->getGoal();
  //  std::cout<<"\ncanditate for "<<a->getId()<<" at "<<a->getNode()->getId()<<",beforenode "<<a->getBeforeNode()->getId()<<"\n";
  for (auto v : C) {
    // std::cout<<v->getId()<<" cost: ";
    int v1 = v->getIndex();
    int g1 = g->getIndex();
    
    cost = new_dists(g1,v1);
    
    if (v == a->getBeforeNode()) {
      cost += 0.5;
      //std::cout<<"0.5 added\n";
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


void PIBT::updateC(Nodes& C, Node* target, Nodes CLOSE_NODE) {
  for (auto v : CLOSE_NODE) {
    auto itr2 = std::find_if(C.begin(), C.end(),
                             [v] (Node* u) { return v == u; });
    if (itr2 != C.end()) C.erase(itr2);
  }
}

std::string PIBT::logStr() {
  std::string str;
  str += "[solver] type:PIBT_V2\n";
  str += Solver::logStr();
  return str;
}

int PIBT::conflict_count(Nodes p1, Nodes p2)
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