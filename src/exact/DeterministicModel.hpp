//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_DETERMINISTIC_MODEL_H
#define DPARP_DETERMINISTIC_MODEL_H

#include "../classes/Parameters.hpp"
#include "../classes/Input.hpp"
#include "../classes/Solution.hpp"
#include <gurobi_c++.h>

class DeterministicModel
{
  Input *input;
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  vector<vector<GRBVar>> x, y, t;
  int num_lazy_cuts, num_frac_cuts;

public:
  DeterministicModel(Input *input)
  {
    if (input != nullptr)
      this->input = input;
    else
      exit(EXIT_FAILURE);
  }

  ~DeterministicModel()
  {
    x.clear(), y.clear(), t.clear();
    model.terminate();
  }

  Solution getSolution();

  Solution Run(bool use_warm_start, string time_limit, string output_file);

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

  void solveCompact(string time_limit);

  void writeSolution(string result);

  bool checkSolution();
};

#endif // DPARP_MODEL_H
