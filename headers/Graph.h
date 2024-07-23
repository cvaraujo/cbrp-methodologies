//
// Created by carlos on 27/05/19.
//

#ifndef DPARP_GRAPH_H
#define DPARP_GRAPH_H

#include "../headers/Arc.h"
#include "../headers/Scenario.h"
#include "../headers/Include.h"

#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <map>

using namespace boost;
using namespace std;

struct SPPRC_Graph_Vert_Prop
{
  SPPRC_Graph_Vert_Prop(int n = 0, int l = 0, int m = 0) : num(n), lim(l), max(m) {}
  int num;
  int lim;
  int max;
};

struct SPPRC_Graph_Arc_Prop
{
  SPPRC_Graph_Arc_Prop(int n = 0, float c = 0, int t = 0, int o = 0) : num(n), cost(c), time(t), cnt(o) {}
  int num;
  // traversal cost
  float cost;
  // traversal time
  int time;
  // Arcs count
  int cnt;
};

// data structures for shortest path problem with time constraint
// ResourceContainer model
struct spp_res_cont
{
  spp_res_cont(float c = 0, int t = 0, int o = 0) : cost(c), time(t), cnt(o) {}
  spp_res_cont &operator=(const spp_res_cont &other)
  {
    if (this == &other)
      return *this;
    this->~spp_res_cont();
    new (this) spp_res_cont(other);
    return *this;
  }
  float cost;
  int time;
  int cnt;
};

typedef adjacency_list<vecS, vecS, directedS, SPPRC_Graph_Vert_Prop, SPPRC_Graph_Arc_Prop> SPPRC_Graph;
typedef graph_traits<SPPRC_Graph>::vertex_descriptor vertex_descriptor;
typedef graph_traits<SPPRC_Graph>::edge_descriptor edge_descriptor;

typedef adjacency_list<vecS, vecS, directedS, no_property, property<edge_weight_t, int, property<edge_weight2_t, int>>> boostGraph;

class Graph
{
  int N, M, B, S, PB = 0, T = 1200;

public:
  vector<vector<Arc *>> arcs, arcs_matrix;
  vector<pair<int, set<int>>> nodes;
  vector<set<int>> nodes_per_block, backup_nodes_per_block;
  vector<vector<Arc *>> arcs_per_block, backup_arcs_per_block;
  vector<int> time_per_block, backup_time_per_block;
  vector<float> cases_per_block, backup_cases_per_block;
  SPPRC_Graph G;
  vector<Scenario> scenarios;
  vector<vector<int>> dist, next;
  vector<vector<vector<int>>> ij_path;
  map<int, int> positive_block_to_block;
  vector<vector<int>> block_2_block_shp;
  vector<vector<set<int>>> block_2_block_shp_nodes;
  vector<vector<vector<Arc *>>> block_2_block_shp_arcs;

  Graph(string instance, string scenarios, int graph_adapt, int km_path, int km_nebulize, int T, int s);

  void load_instance(string instance, int graph_adapt, int km_path, int km_nebulize, int T);

  int getShortestPath(int s, int t, vector<int> &path);

  void reduceGraphToPositiveCases();

  void reduceGraphToCompleteDigraphBlocks();

  void AllPairsShortestPath();

  int SHPBetweenBlocks(int b1, int b2, set<int> &nodes, map<int, map<int, bool>> &arcs);

  void load_scenarios_instance(string instance);

  double run_spprc(set<pair<int, int>> &x);

  double knapsack(vector<int> &y, vector<double> cases, vector<int> time, int MT);

  set<int> getBlocksFromRoute(set<pair<int, int>> x);

  void populateKnapsackVectors(set<int> blocks, vector<double> &cases, vector<int> &time);

  void updateBoostArcCost(int i, int j, double new_cost);

  int getN() const;

  int getM() const;

  int getB() const;

  int getPB() const;

  int getT() const;

  int getS() const;

  void setS(int s);

  int getDepot() const;

  int getSink() const;

  void showGraph();

  void showScenarios();

  Arc *getArc(int i, int j);
};

#endif
