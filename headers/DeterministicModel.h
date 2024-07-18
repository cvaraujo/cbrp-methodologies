//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_DETERMINISTIC_MODEL_H
#define DPARP_DETERMINISTIC_MODEL_H

#include "Include.h"
#include "Graph.h"
#include <gurobi_c++.h>
#include <vector>
#include "WarmStart.h"
using namespace std;

class DeterministicModel
{
public:
  Graph *graph;
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  vector<vector<GRBVar>> x, y, t;
  int num_lazy_cuts, num_frac_cuts;

  DeterministicModel(Graph *graph);

  ~DeterministicModel();

  void objectiveFunction();

  void WarmStart();

  void createVariables();

  void initModelCompact(bool warm_start);

  void artificialNodes();

  void flowConservation();

  void maxAttending();

  void attendingPath();

  void timeConstraint();

  void compactTimeConstraint();

  void solveCompact(string timeLimit);

  void writeSolution(string result);

  bool check_solution(float max_time, float max_insecticide);
};

#endif // DPARP_MODEL_H
