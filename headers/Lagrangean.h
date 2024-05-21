//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_LAGRANGEAN_H
#define DPARP_LAGRANGEAN_H

#include "Include.h"
#include "Graph.h"
#include <vector>

using namespace std;

class Lagrangean
{
  Graph *graph;
  double multTime, UB, LB;
  vector<double> multConn;
  int num_lazy_cuts, num_frac_cuts;
  float default_vel, spraying_vel, insecticide_ml_min, max_time;
  bool feasible;

public:
  Lagrangean(Graph *graph);

  int lagrangean_relax();

  double solve_ppl(set<pair<int, int>> &x, vector<int> &y);

  void createVariables();

  void getGradientConnection(vector<double> &lambda, set<pair<int, int>> x, vector<int> y);

  void getGradientTime(double &sigma, set<pair<int, int>> x, vector<int> y);

  double getNorm(vector<double> &gradient);

  bool isFeasible();

  int getOriginalObjValue(vector<int> y);

  int bestAttendFromRoute(set<pair<int, int>> &x, vector<int> &y);
};

#endif // DPARP_MODEL_H
