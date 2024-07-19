//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_STOCHASTIC_MODEL_H
#define DPARP_STOCHASTIC_MODEL_H

#include "Include.h"
#include "Graph.h"
#include <gurobi_c++.h>
#include <vector>
using namespace std;

class StochasticModel
{
public:
  Graph *graph;
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  vector<vector<vector<GRBVar>>> x, y, t;
  vector<vector<GRBVar>> z;
  int num_lazy_cuts, num_frac_cuts;
  float default_vel, spraying_vel, insecticide_ml_min, alpha = 0.8;

  void objectiveFunction();

  void waitAndSeeOF();

  StochasticModel(Graph *graph);

  void createVariables();

  void initModelCompact(bool warm_start);

  void zValue();

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
