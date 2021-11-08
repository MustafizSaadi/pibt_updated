/*
 * mapf.cpp
 *
 * Purpose: MAPF
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "mapf.h"
#include "../util/util.h"
//#include "../graph/vec2f.h"

MAPF::MAPF(Graph* _G,
           Agents _A,
           std::vector<Task*> _T) : Problem(_G, _A, _T)
{
  init();
}

MAPF::MAPF(Graph* _G,
           Agents _A,
           std::vector<Task*> _T,
           std::mt19937* _MT) : Problem(_G, _A, _T, _MT)
{
  init();
}

void MAPF::init() {
  // error check
  if (A.size() != T_OPEN.size()) {
    std::cout << "error@MAPF::init, this is not a MAPF, |A| != |T|" << "\n";
    std::exit(1);
  }

  // allocation, t=0
  //std::cout<<"Coordinate "<<std::endl;
  //std::cout<<"agents: "<<A.size()<<"\n";
  for (int i = 0; i < A.size(); ++i) {
    A[i]->setTask(T_OPEN[i]);
    A[i]->setGoal(A[i]->getTask()->getG()[0]);
    Node* posst = A[i]->getNode();
    Node* posen = A[i]->getGoal(); 
    //std::cout<<posst->getPos().x<<","<<posst->getPos().y<<","<<posen->getPos().x<<","<<posen->getPos().y<<",\n";
    A[i]->updateHist();
  }
}

MAPF::~MAPF() {}

bool MAPF::isSolved() {
  return T_OPEN.empty();
}

void MAPF::update() {
  ++timestep;
  //std::cout<<"src/problem/mapf.cpp\n";
  for (auto a : A) {
    auto tau = a->getTask();
    if (tau) {  // if agent has a task
      tau->update(a->getNode());  // update task status
      if (tau->completed()) {  // if task is completed
        a->releaseTaskOnly();  // agent release task, not goal
        openToClose(tau, T_OPEN, T_CLOSE);
      }
    } else if (a->getNode() != a->getGoal()) {
      // create new task and assign
      Task* newTau = new Task(a->getGoal());
      a->setTask(newTau);
      a->setGoal(newTau->getG()[0]);
      T_OPEN.push_back(newTau);
    }
    a->updateHist();
  }
}

std::string MAPF::logStr() {
  std::string str = Problem::logStr();
  str += "[problem] type:MAPF\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += G->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();

  int cnt;
  int pathsize,solnCost=0;
  for (auto a : A) {
    auto hist = a->getHist();
    auto itr = hist.end() - 1;
    cnt = 0;
    while ((*itr)->v == a->getGoal()) {
      ++cnt;
      --itr;
    }
    pathsize = getTerminationTime() - cnt + 1;
    solnCost += pathsize;
    str += a->logStr();
    str += "size:" + std::to_string(pathsize) + "\n";
  }
  str += "SolutionCost:" + std::to_string(solnCost) + "\n";
  std::cout<<"SolutionCost: " << std::to_string(solnCost)<<"\n";
  return str;
}
