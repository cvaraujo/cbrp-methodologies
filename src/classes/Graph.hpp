//
// Created by carlos on 27/05/19.
//

#ifndef DPARP_GRAPH_H
#define DPARP_GRAPH_H

#include "Arc.hpp"
#include "Scenario.hpp"
#include "Parameters.hpp"

class Graph
{
  int N, M, B, PB;

  vector<vector<Arc *>> arcs, arcs_matrix, arcs_per_block;
  vector<pair<int, set<int>>> nodes;
  vector<set<int>> nodes_per_block;
  vector<int> time_per_block;
  vector<double> cases_per_block;

public:
  Graph(string instance, int km_path, int km_nebulize);

  void LoadGraph(string instance, int km_path, int km_nebulize);

  void ShowGraph()
  {
    for (int i = 0; i <= N; i++)
      for (auto *arc : arcs[i])
        cout << "[" << i << ", " << arc->getD() << "] - " << arc->getBlock() << endl;
  }

  set<int> getBlocksFromRoute(set<pair<int, int>> x)
  {
    int i;
    set<int> blocks;

    for (auto p : x)
    {
      i = p.first;
      for (auto b : this->nodes[i].second)
        if (b != -1)
          blocks.insert(b);
    }

    return blocks;
  }

  Arc *getArc(int i, int j)
  {
    if (arcs_matrix[i][j] != nullptr)
      return arcs_matrix[i][j];

    for (auto arc : arcs[i])
      if (arc->getD() == j)
      {
        arcs_matrix[i][j] = arc;
        return arc;
      }

    return nullptr;
  }

  void addArtificialNode(int n)
  {
    this->nodes.push_back(make_pair(n, set<int>()));
    this->arcs.push_back(vector<Arc *>());

    for (int i = 0; i < N; i++)
      this->arcs[N].push_back(new Arc(n, i, 0, -1)), this->arcs[i].push_back(new Arc(i, n, 0, -1));
  };

  vector<Arc *> getArcsPerBlock(int block) { return arcs_per_block[block]; }

  void setCasesPerBlock(int block, double cases) { cases_per_block[block] = cases; }

  void setCasesPerBlock(vector<double> cases) { cases_per_block = cases; }

  void setTimePerBlock(int block, int time) { time_per_block[block] = time; }

  void setTimePerBlock(vector<int> time) { time_per_block = time; }

  void setNodesPerBlock(int block, set<int> nodes) { nodes_per_block[block] = nodes; }

  void setNodesPerBlock(vector<set<int>> nodes) { nodes_per_block = nodes; }

  double getCasesPerBlock(int block) { return cases_per_block[block]; }

  vector<double> getCasesPerBlock() { return cases_per_block; }

  int getTimePerBlock(int block) { return time_per_block[block]; }

  vector<int> getTimePerBlock() { return time_per_block; }

  pair<int, set<int>> getNodes(int i) { return nodes[i]; }

  void setBlocksFromNode(int i, set<int> blocks) { nodes[i].second = blocks; }

  set<int> getNodesFromBlock(int block) { return nodes_per_block[block]; }

  vector<Arc *> getArcs(int i) { return arcs[i]; }

  pair<int, set<int>> getNode(int i) { return nodes[i]; }

  vector<pair<int, set<int>>> getNodes() { return nodes; }

  void setNodes(vector<pair<int, set<int>>> nodes) { this->nodes = nodes; }

  void setArcs(vector<vector<Arc *>> arcs) { this->arcs = arcs; }

  void setArcsMatrix(vector<vector<Arc *>> arcs) { this->arcs_matrix = arcs; }

  vector<set<int>> getNodesPerBlock() { return nodes_per_block; }

  int getN() const { return N; }

  void setN(int N) { this->N = N; }

  int getM() const { return M; }

  void setM(int M) { this->M = M; }

  int getB() const { return B; }

  void setB(int B) { this->B = B; }

  int getPB() const { return PB; }

  void setPB(int PB) { this->PB = PB; }

  int getDepot() const { return N; }

  void setDepot(int N) { this->N = N; }

  int getSink() const { return N + 1; }

  void setSink(int N) { this->N = N; }
};

#endif
