/*
 * mapf.cpp
 *
 * Purpose: MAPF
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "mapd.h"
#include "../util/util.h"

MAPD::MAPD(Graph* _G, Agents _A, Nodes _s, Nodes _g, int _n, float _f)
  : Problem(_G, _A), pickupNodes(_s), deliveryNodes(_g), taskNum(_n), taskFrequency(_f)
{
  init();
}

MAPD::MAPD(Graph* _G, Agents _A,
           std::vector<int> _s, std::vector<int> _g,
           int _n, float _f)
  : Problem(_G, _A), taskNum(_n), taskFrequency(_f)
{
  for (auto s : _s) pickupNodes.push_back(G->getNode(s));
  for (auto g : _g) deliveryNodes.push_back(G->getNode(g));
  init();
}

MAPD::MAPD(Graph* _G, Agents _A, Nodes _s, Nodes _g,
           int _n, float _f, std::mt19937* _MT)
  : Problem(_G, _A, _MT), pickupNodes(_s), deliveryNodes(_g),
  taskNum(_n), taskFrequency(_f)
{
  init();
}

MAPD::MAPD(Graph* _G, Agents _A, std::vector<int> _s, std::vector<int> _g,
           int _n, float _f, std::mt19937* _MT)
  : Problem(_G, _A, _MT), taskNum(_n), taskFrequency(_f)
{
  for (auto s : _s) pickupNodes.push_back(G->getNode(s));
  for (auto g : _g) deliveryNodes.push_back(G->getNode(g));
  init();
}


void MAPD::init() {
  taskCnt = 0;
  autoAssignment = true;
  --timestep;
  update();
}

MAPD::~MAPD() {}

bool MAPD::isSolved() {
  // std::cout << " in solved" << std::endl;
  if (taskCnt >= taskNum && T_OPEN.empty()) {
    // std::cout << " in condition" << std::endl;
    if (std::all_of(A.begin(), A.end(),
                    [](Agent* a) { return !a->hasTask(); })) {
      return true;
    }
  }
  // std::cout << " condition not fulfilled" << std::endl;
  return false;
}

void MAPD::update() {
  // std::cout << autoAssignment;
  ++timestep;
  // std::cout << "mapd update" << std::endl;
  updateStatus();
  // std::cout << "mapd task" << std::endl;
  createTask();
  // std::cout << "mapd assign" << std::endl;
  if (autoAssignment) autoAssign();
  // std::cout << "mapd hist" << std::endl;
  for (auto a : A) a->updateHist();
}

void MAPD::autoAssign() {
  for (auto a : A) {
    // std:: cout << a->hasTask() << std::endl;
    if (a->hasTask()) continue;
    for (auto tau : T_OPEN) {
      // std::cout << a->getNode()->getId() << " " << tau->getG()[0]->getId() << std::endl;
      if (a->getNode() == tau->getG()[0]) {
        // std::cout << a->getId() << " " << a->getNode()->getId() << " in task " << tau->getG()[0]->getId() << std::endl;
        a->setTask(tau);
        if (tau->getG()[0] == a->getNode()) {
          tau->update(a->getNode());
        }
        a->setGoal(tau->getG()[0]);
        // std::cout << " " << tau->getG()[0]->getId();
        openToClose(tau, T_OPEN, T_CLOSE);
        break;
      }
    }
  }
}

void MAPD::updateStatus() {
  // update agent status
  for (auto a : A) {
    // std::cout << " loop begin" << std::endl;
    if (!a->hasTask()) continue;
    // std::cout << " before goal test" << std::endl;
    if (a->hasGoal()) {
      if (a->getTask()->getG()[0] == a->getNode()) {
        a->getTask()->update(a->getNode());
      }
    }
    // std::cout << " before task test" << std::endl;
    if (a->getTask()->completed()) {
      a->getTask()->setEndTime(timestep);
      a->releaseTask();
    }
    // std::cout << " loop end" << std::endl;
  }
}


void MAPD::createTask() {
  if (taskCnt >= taskNum) return;

  int num;
  if (taskFrequency >= 1) {
    num = (int)taskFrequency;
  } else if (timestep % (int)(1 / taskFrequency) == 0) {
    num = 1;
  } else {
    num = 0;
  }

  Node *p, *d;

  for (int i = 0; i < num; ++i) {
    if (taskCnt < taskNum) {  // limit
      do {
        p = randomChoose(pickupNodes, MT);
        d = randomChoose(deliveryNodes, MT);
      } while (p == d);
      Task* tau = new Task(timestep);
      tau->addNode(p);  // pickup
      tau->addNode(d);  // delivery
      T_OPEN.push_back(tau);
      ++taskCnt;
    }
  }
}

std::string MAPD::logStr() {
  std::string str;
  str += Problem::logStr();
  str += "[problem] type:MAPD\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += "[problem] tasknum:" + std::to_string(taskNum) + "\n";
  str += "[problem] frequency:" + std::to_string(taskFrequency) + "\n";
  str += G->logStr();
  for (auto tau : T_OPEN) str += tau->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();
  for (auto a : A) str += a->logStr();
  return str;
}
