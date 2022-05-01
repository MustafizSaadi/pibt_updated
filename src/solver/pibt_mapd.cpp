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

PIBT_MAPD::PIBT_MAPD(Problem *_P) : Solver(_P)
{
  init();
}
PIBT_MAPD::PIBT_MAPD(Problem *_P, std::mt19937 *_MT) : Solver(_P, _MT)
{
  init();
}
PIBT_MAPD::~PIBT_MAPD() {}
void PIBT_MAPD::init()
{
  G->setRegFlg(true);
}

bool PIBT_MAPD::solve()
{
  solveStart();

  // initialize priroirty
  int agentNum = A.size();

  cout << A.size() << endl;

  // since, initially no agent has any task or path, it is not possible to prioritize based on conflict count

  for (int i = 0; i < agentNum; ++i)
  {
    epsilon.push_back((float)i / agentNum);
    eta.push_back(0);
    priority.push_back(value_for_priority{0, epsilon[i] + eta[i], !A[i]->checkRunning(), INT_MAX});
    A[i]->setBeforeNode(A[i]->getNode());
    A[i]->goal_count = 0;
  }


  while (!P->isSolved())
  {
    //cout << "before allocate";
    bool flag = allocate();
    // cout << "before update" << endl;
    update(flag);
    // cout << "before P->update" << endl;
    P->update();
    if (P->getTimestep() >= P->getTimestepLimit())
    {
      break;
    }
  }

  solveEnd();
  return true;
}

bool PIBT_MAPD::allocate()
{
  bool flag = false;
  if (P->allocated())
    return flag;
  auto T = P->getT();
  Graph *_G = G;

  // start of modified allocate method

  if (T.empty())
  {
    for (auto a : A)
    { 
      if(a->hasTask()) continue;
      a->releaseGoalOnly();
    }
  }
  else
  {
    for (auto t : T)
    {
      auto itr = std::min_element(A.begin(), A.end(),
                                  [t, _G](Agent *a1, Agent *a2)
                                  {
                                    if (!a1->hasTask() && !a2->hasTask())
                                      return _G->dist(t->getG()[0], a1->getNode()) < _G->dist(t->getG()[0], a2->getNode());
                                    else if (a1->hasTask() && !a2->hasTask())
                                      return false;
                                    else
                                      return true;
                                  });

      if (!(*itr)->hasTask() && (!(*itr)->hasGoal() || (*itr)->getGoal() != t->getG()[0]))
      {
        (*itr)->setGoal(t->getG()[0]);
        (*itr)->goal_count++;
        flag = true;
      }
    }
  }

  return flag;
}

void PIBT_MAPD::update(bool flag)
{
  // prioritizing based on conflict count
  updatePriority(flag);

  std::vector<value_for_priority> PL(priority.size()); // priority list
  std::copy(priority.begin(), priority.end(), PL.begin());

  Nodes CLOSE_NODE;
  Agents OPEN_AGENT(A.size());
  std::copy(A.begin(), A.end(), OPEN_AGENT.begin());

  std::chrono::system_clock::time_point st = std::chrono::high_resolution_clock::now();

  auto itr = std::max_element(PL.begin(), PL.end(), [this](value_for_priority a, value_for_priority b)
                              { return max_compare(a, b); });

  std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();
  P->heuristicTime += std::chrono::duration_cast<std::chrono::milliseconds>(en - st).count();
  
  int index = std::distance(PL.begin(), itr);
  Agent *a = OPEN_AGENT[index];

  while (!OPEN_AGENT.empty())
  {

    /*update*/
    
    /* some agents do not have any goal means they do not have any task */
    if (a->hasGoal() && (a->getNode()->getId() != a->getGoal()->getId()))
    {
      priorityInheritance(a, CLOSE_NODE, OPEN_AGENT, PL);
    }
    else
    {
      auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
      PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
      OPEN_AGENT.erase(itr);
    }

    /*update*/
    std::chrono::system_clock::time_point st = std::chrono::high_resolution_clock::now();

    itr = std::max_element(PL.begin(), PL.end(), [this](value_for_priority a, value_for_priority b)
                           { return max_compare(a, b); });

    std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();
    P->heuristicTime += std::chrono::duration_cast<std::chrono::milliseconds>(en - st).count();

    index = std::distance(PL.begin(), itr);
    a = OPEN_AGENT[index];
  }
}

bool PIBT_MAPD::compare(st *a, st *b)
{
  float dista = new_dists(A[a->ind]->getGoal()->getIndex(), A[a->ind]->getNode()->getIndex());
  float distb = new_dists(A[b->ind]->getGoal()->getIndex(), A[b->ind]->getNode()->getIndex());

  return dista < distb;
}

bool PIBT_MAPD::max_compare(value_for_priority a, value_for_priority b)
{
  if (a.is_in_goal && b.is_in_goal)
    return true;
  else if (a.is_in_goal || b.is_in_goal)
  {
    return b.is_in_goal;
  }
  else
  {
    if (a.task == b.task)
    {
      if (a.conf == b.conf)
        return a.goal_distance > b.goal_distance;
      else
        return a.conf < b.conf;
    }
    else
    {
      return a.task > b.task;
    }
  }
}

void PIBT_MAPD::updatePriority(bool flag)
{
  // /* update */
  if (flag)
  {
    std::chrono::system_clock::time_point st = std::chrono::high_resolution_clock::now();
    for (auto a : A)
    {
      if(a->hasGoal())
        a->path = getShortestPath(a->getNode(), a->getGoal());
    }

    for (int i = 0; i < A.size(); i++)
    {
      int conf = 0;
      if(! A[i]->hasGoal()) continue;
      for (int j = 0; j < A.size(); j++)
      {
        if (i == j || !A[j]->hasGoal())
          continue;
        conf += conflict_count(A[i]->path, A[j]->path);
      }
      
      priority[i] = value_for_priority{A[i]->goal_count, (float)conf, A[i]->checkRunning(), (int)A[i]->path.size()};

      A[i]->conf = conf;
      
    }

    std::chrono::system_clock::time_point en = std::chrono::high_resolution_clock::now();
    P->heuristicTime += std::chrono::duration_cast<std::chrono::milliseconds>(en - st).count();
  }
  else
  {
    for (int i = 0; i < A.size(); i++)
    {
      if (!A[i]->hasGoal() || A[i]->getNode() == A[i]->getGoal())
      {
        priority[i] = value_for_priority{A[i]->goal_count, 0, true, 0};
      }
    }
  }
}

float PIBT_MAPD::getDensity(Agent *a)
{
  float density = 0;
  Node *v, *u;
  int d, tmp;
  v = a->getNode();
  Nodes Ci, Cj;
  Ci = G->neighbor(v);

  for (auto b : A)
  {
    if (b == a)
      continue;
    u = b->getNode();
    d = G->dist(u, v);
    if (G->dist(u, v) > 2)
      continue;
    tmp = -d + 2;
    Cj = G->neighbor(u);
    for (auto w : Cj)
    {
      if (w == v)
      {
        tmp += 2;
      }
      else if (G->dist(w, u) == 1)
      {
        ++tmp;
      }
    }
    density += (float)tmp / (float)Cj.size();
  }

  density /= (float)Ci.size();
  return density;
}

bool PIBT_MAPD::priorityInheritance(Agent *a,
                                    Nodes &CLOSE_NODE,
                                    Agents &OPEN_AGENT,
                                    std::vector<value_for_priority> &PL)
{
  Nodes C = createCandidates(a, CLOSE_NODE);
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT_MAPD::priorityInheritance(Agent *a,
                                    Agent *aFrom,
                                    Nodes &CLOSE_NODE,
                                    Agents &OPEN_AGENT,
                                    std::vector<value_for_priority> &PL)
{
  Nodes C = createCandidates(a, CLOSE_NODE, aFrom->getNode());
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT_MAPD::priorityInheritance(Agent *a,
                                    Nodes C,
                                    Nodes &CLOSE_NODE,
                                    Agents &OPEN_AGENT,
                                    std::vector<value_for_priority> &PL)
{
  // remove agent from OPEN_AGENT, Priority List
  auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
  PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
  OPEN_AGENT.erase(itr);


  Node *target;

  if (a->getNode() == a->getGoal())
    a->visitedGoal = true;

  // main loop
  while (!C.empty())
  {

    // choose target
    target = chooseNode(a, C);
    CLOSE_NODE.push_back(target);

    bool isAgent = false;
    for (auto b : OPEN_AGENT)
    {
      if (target == b->getNode())
      { 
        P->pibt_count++;

        if (priorityInheritance(b, a, CLOSE_NODE, OPEN_AGENT, PL))
        {
          // priority inheritance success
          a->setBeforeNode(a->getNode());
          a->setNode(target);
          return true;
        }
        else
        {
          // priority inheritance fail
          updateC(C, target, CLOSE_NODE);
          isAgent = true;
          break;
        }
      }
    }

    // If there is an agent
    if (!isAgent)
    {
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

Nodes PIBT_MAPD::createCandidates(Agent *a, Nodes CLOSE_NODE)
{
  Nodes C;
  for (auto v : G->neighbor(a->getNode()))
  {
    if (!inArray(v, CLOSE_NODE))
      C.push_back(v);
  }
  if (!inArray(a->getNode(), CLOSE_NODE))
    C.push_back(a->getNode());
  return C;
}

Nodes PIBT_MAPD::createCandidates(Agent *a, Nodes CLOSE_NODE, Node *tmp)
{
  CLOSE_NODE.push_back(tmp);
  Nodes C = createCandidates(a, CLOSE_NODE);
  CLOSE_NODE.erase(CLOSE_NODE.end() - 1);
  return C;
}

Node *PIBT_MAPD::chooseNode(Agent *a, Nodes C)
{
  if (C.empty())
  {
    std::cout << "error@PIBT_MAPD::chooseNode, C is empty"
              << "\n";
    std::exit(1);
  }

  // randomize
  std::shuffle(C.begin(), C.end(), *MT); //randomly ekta neighbouring node nisse...
                                         // eta change kora jaite pare...

  if (!a->hasGoal())
  {
    if (inArray(a->getNode(), C))
    { // try to stay
      return a->getNode();
    }
    else
    {
      return C[0]; // random walk
    }
  }

  return chooseNode(a->getNode(), a->getGoal(), C, true, a->getBeforeNode());
}

Node *PIBT_MAPD::chooseNode(Node *v, Node *g, Nodes C, bool flag, Node *beforeNode)
{
  std::string ss = dijkstra(g->getIndex());
  Nodes cs;
  float minCost = 1000000;
  float cost;

  for (auto v : C)
  {
    int v1 = v->getIndex();
    int g1 = g->getIndex();

    cost = new_dists(g1, v1);

    if (flag)
    {
      if (v == beforeNode)
      {
        cost += 0.5;
      }
    }
    
    if (cost < minCost)
    {
      minCost = cost;
      cs.clear();
      cs.push_back(v);
    }
    else if (cost == minCost)
    {
      cs.push_back(v);
    }
  }

  if (cs.size() == 1)
    return cs[0];

  // tie break
  bool contained;
  for (auto v : cs)
  { // avoid tabu list
    contained = std::any_of(A.begin(), A.end(),
                            [v](Agent *b)
                            { return b->getNode() == v; });
    if (!contained)
      return v;
  }

  return cs[0];
}

void PIBT_MAPD::updateC(Nodes &C, Node *target, Nodes CLOSE_NODE)
{
  for (auto v : CLOSE_NODE)
  {
    auto itr2 = std::find_if(C.begin(), C.end(),
                             [v](Node *u)
                             { return v == u; });
    if (itr2 != C.end())
      C.erase(itr2);
  }
}

std::string PIBT_MAPD::logStr()
{
  std::string str;
  str += "[solver] type:PIBT_MAPD_V2\n";
  str += Solver::logStr();
  return str;
}

int PIBT_MAPD::conflict_count(Nodes p1, Nodes p2)
{
  int collision = 0;

  for (int t = 0; t < p1.size(); ++t)
  {
    if (t >= p2.size())
    {
      if (p1[t] == p2[p2.size() - 1])
      {
        ++collision;
        break;
      }
      continue;
    }
    if (p1[t] == p2[t])
    { // collision
      ++collision;
      break;
    }
    if (t > 0 && p1[t - 1] == p2[t] && p1[t] == p2[t - 1])
    {
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

  while (v != g)
  {
    v = chooseNode(v, g, neighbor, false, nullptr);
    path.push_back(v);
    neighbor = G->neighbor(v->getId());
  }

  path.push_back(g);

  return path;
}