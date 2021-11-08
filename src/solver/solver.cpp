/*
 * solver.cpp
 *
 * Purpose: utility of solver class
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "solver.h"
#include <random>
#include "../util/util.h"
#include <typeinfo>

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <set>



class CSVReader
{
    std::string fileName;
    std::string delimeter;
public:
    CSVReader(std::string filename, std::string delm = " ") :
            fileName(filename), delimeter(delm)
    { }
    // Function to fetch data from a CSV File
    std::vector<std::vector<std::string> > getData();
};
/*
* Parses through csv file line by line and returns the data
* in vector of vector of strings.
*/
std::vector<std::vector<std::string> > CSVReader::getData()
{
    std::ifstream file(fileName);
    std::vector<std::vector<std::string> > dataList;
    std::string line = "";
    // Iterate through each line and split the content using delimeter
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        dataList.push_back(vec);
    }
    // Close the File
    file.close();
    return dataList;
}




Solver::Solver(Problem* _P) : P(_P) {
  std::random_device seed_gen;
  MT = new std::mt19937(seed_gen());
  init();
}
Solver::Solver(Problem* _P, std::mt19937* _MT) : P(_P), MT(_MT) {
  
  init();
  
}

Solver::~Solver() {}

void Solver::init() {
  G = P->getG();
  A = P->getA();
  int nodeNum = G->getNodesNum();
  for (int i = 0; i < nodeNum; ++i)
  {
    is_visit.push_back(false);
    /* code */
  }
  dists = Eigen::MatrixXi::Zero(nodeNum, nodeNum);
  new_dists = Eigen::MatrixXi::Ones(nodeNum, nodeNum)*10000;
}

void Solver::setDirectory(std::string folder_name){
  std::string cost = "cost";
  folder_name.replace(2,3,cost);
  folder_name.replace(folder_name.size()-4,4,"/");
  directory_name = folder_name;
  return;
}

void Solver::solveStart() {
  startT = std::chrono::high_resolution_clock::now();
}

void Solver::solveEnd() {
  endT = std::chrono::high_resolution_clock::now();
  elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>
    (endT-startT).count();
}

void Solver::WarshallFloyd(std::string flieName) {
  int nodeNum = G->getNodesNum();
  Nodes neighbor;
  int INF = 100000;
  dists = Eigen::MatrixXi::Ones(nodeNum, nodeNum) * INF;

  // initialize weight
  for (int i = 0; i < nodeNum; ++i) {
    neighbor = G->neighbor(G->getNodeFromIndex(i));
    for (auto v : neighbor) {
      dists(i, v->getIndex()) = 1;
    }
    dists(i, i) = 0;
  }

  //std::cout<<"Node num "<<nodeNum<<" shuru\n";

  // // main loop
  for (int k = 0; k < nodeNum; ++k) {
    for (int i = 0; i < nodeNum; ++i) {
      for (int j = 0; j < nodeNum; ++j) {
        if (dists(i, j) > dists(i, k) + dists(k, j)) {
          dists(i, j) = dists(i, k) + dists(k, j);
        }
      }
    }
  }
}

int Solver::getMaxLengthPaths(Paths& paths) {
  if (paths.empty()) return 0;
  auto itr = std::max_element(paths.begin(), paths.end(),
                              [] (Nodes p1, Nodes p2) {
                                return p1.size() < p2.size(); });
  return itr->size();
}

void Solver::formalizePath(Paths& paths) {
  int maxLength = getMaxLengthPaths(paths);
  Node* g;
  for (int i = 0; i < paths.size(); ++i) {
    g = paths[i][paths[i].size() - 1];
    while (paths[i].size() != maxLength) paths[i].push_back(g);
  }
}

int Solver::pathDist(Node* s, Node* g) {
  // same place?
  if (s == g) return 0;

  // has already explored?
  int s_index = G->getNodeIndex(s);
  int g_index = G->getNodeIndex(g);
  int dist = dists(s_index, g_index);
  if (dist != 0) return dist;

  // new
  Nodes path = G->getPath(s, g);

  dist = path.size() - 1;  // without start node
  int index, d, cost;
  bool directed = G->isDirected();

  cost = dist;
  for (auto v : path) {
    index = G->getNodeIndex(v);
    d = dists(index, g_index);
    if ((index != g_index) && (d == 0)) {
      dists(index, g_index) = cost;
      // if not directed graph
      if (!directed) dists(g_index, index) = cost;
      --cost;
    } else if (d == cost) {
      break;
    } else {
      std::cout << "error@Solver::pathDist, "
                << s->getId() << " -> " << g->getId() << ", "
                << s->getPos() << " -> " << g->getPos() << ", "
                << "not optimal path is obtained" << "\n";
      std::exit(1);
    }
  }

  return dist;
}

int Solver::pathDist(Node* s, Node* g, Nodes &prohibited) {
  // same place?
  if (s == g) return 0;

  return G->getPath(s, g, prohibited).size() - 1;
}

std::string Solver::getKey(int t, Node* v) {
  std::string key = "";
  key += std::to_string(t);
  key += "-";
  key += std::to_string(v->getId());
  return key;
}

std::string Solver::getKey(AN* n) {
  return getKey(n->g, n->v);
}

std::string Solver::logStr() {
  std::string str;

  str += "[solver] solved:" + std::to_string(P->isSolved()) + "\n";
  str += "elapsed: " + std::to_string((int)elapsedTime) + "\n";
  str += "makespan: " + std::to_string(P->getTerminationTime()) + "\n";
  std::cout<<str;
  str += P->logStr();

  return str;
}


void Solver::SaveDataToFile(std::string folder_name) {
    //std::cout<<"asche\n";
    std::string cost = "cost";
    folder_name.replace(2,3,cost);
    folder_name.replace(folder_name.size()-4,4,"/");
    directory_name = folder_name;

    int nodeNum = G->getNodesNum();
    for (int i = 0; i < nodeNum ; ++i)
    {
      std::string cost_data = dijkstra(i);
      //std::cout<<"counting for : "<<i<<"\n";
      std::string filename = folder_name+"node_"+std::to_string(i);
      std::ofstream file;
      file.open (filename);
      file << cost_data;
      file.close();
    }
}


bool Solver::djikstra_source_to_goal(Agent *a, Node *s, Node *g){

    int SIZE = G->getNodesNum();

    int dista [SIZE];
    bool vis [SIZE];                                       
    std::memset(vis, false , sizeof vis);            
    std::memset(dista,10000,sizeof dista);
    int src_id = s->getId(); 
    dista[src_id] = 0;
    std::multiset < std::pair < int , int > > m_set;         

    m_set.insert({0 , src_id});                          
    int cnt = 0;
    while(!m_set.empty()){
        cnt++;
        std::pair <int , int> p = *m_set.begin();        
        m_set.erase(*m_set.begin());

        int x = p.second; int wei = p.first;
        if(g->getId() == x) return true;
        if( vis[x] ) continue;                  
        vis[x] = true;
        Nodes neighbor;
        neighbor = G->neighbor(G->getNode(x));
        for (auto v : neighbor) {
          int e = v->getId();
          int w = 1;
          if(dista[x] + w < dista[e]  ){            
                dista[e] = dista[x] + w;
                m_set.insert({dista[e],  e} );           
          }
        }
    }

    //std::string shortest_path = ""+std::to_string(dista[0]);
    // for (int i = 0; i < SIZE; ++i)
    // {
    //   new_dists(src_node,i) = dista[i];
    //   //shortest_path = shortest_path + " " + std::to_string(dista[i]);
    // }
    // //shortest_path = shortest_path + "\n";
    // is_visit[src_node] = true;
    // return "";

    return false;
}



std::string Solver::dijkstra(int src_node){
    if (is_visit[src_node]){
      return " ";
    }

    int SIZE = G->getNodesNum();

    int dista [SIZE];
    bool vis [SIZE];                                       
    std::memset(vis, false , sizeof vis);            
    std::memset(dista,10000,sizeof dista);
    dista[src_node] = 0;
    std::multiset < std::pair < int , int > > s;         

    s.insert({0 , src_node});                          
    int cnt = 0;
    while(!s.empty()){
        cnt++;
        std::pair <int , int> p = *s.begin();        
        s.erase(s.begin());

        int x = p.second; int wei = p.first;
        if( vis[x] ) continue;                  
        vis[x] = true;
        Nodes neighbor;
        neighbor = G->neighbor(G->getNodeFromIndex(x));
        for (auto v : neighbor) {
          int e = v->getIndex();
          int w = 1;
          if(dista[x] + w < dista[e]  ){            
                dista[e] = dista[x] + w;
                s.insert({dista[e],  e} );           
          }
        }
    }

    //std::string shortest_path = ""+std::to_string(dista[0]);
    for (int i = 0; i < SIZE; ++i)
    {
      new_dists(src_node,i) = dista[i];
      //shortest_path = shortest_path + " " + std::to_string(dista[i]);
    }
    //shortest_path = shortest_path + "\n";
    is_visit[src_node] = true;
    return "";
}

void Solver::collectCost(int src_node){
  std::string flieName = directory_name+"node_"+std::to_string(src_node);
  // std::cout<<"collectCost e file name : "<<flieName<<"\n";
  // std::vector<std::vector<std::string> > dataList;
  CSVReader reader(flieName);
//     // Get the data from CSV File
  dataList = reader.getData();
  int nodeNum = G->getNodesNum();
  // std::cout<<"dataList size : "<<dataList.size()<<"\n";
  // std::cout<<"dataList er vitore size : "<<dataList[0].size()<<"\n";
  shortest_path_cost[src_node] = dataList[0];
}