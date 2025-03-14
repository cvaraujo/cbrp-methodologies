//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_LAGRANGEAN_H
#define DPARP_LAGRANGEAN_H

#include "../classes/Input.hpp"
#include "../common/Knapsack.hpp"
#include "../common/BoostLibrary.hpp"
#include "GreedyHeuristic.hpp"
#include "gurobi_c++.h"
#include <vector>

class Lagrangean
{
  Input *input;
  double mult_time, UB, LB;
  vector<double> mult_conn;
  int num_lazy_cuts, num_frac_cuts, T;
  bool is_feasible;
  double curr_route_time = 0.0;
  BoostLibrary *boost;
  GreedyHeuristic *greedyHeuristic;

public:
  Lagrangean(Input *input);

  int lagrangean_relax();

  int bestAttendFromRoute(const map<pair<int, int>, int> &x, vector<int> &y);

  double solve_ppl(map<pair<int, int>, int> &x, vector<int> &y);

  pair<int, double> runSolverERCSPP(set<pair<int, int>> &x);

  pair<int, double> runSHPRC(map<pair<int, int>, int> &x);

  void getGradientConnection(vector<double> &gradient_lambda, map<pair<int, int>, int> x, vector<int> y);

  int getGradientTime(map<pair<int, int>, int> x, vector<int> y);

  double getNorm(vector<double> &gradient);

  bool isFeasible();

  int getOriginalObjValue(vector<int> y);
};

#endif // DPARP_MODEL_H
