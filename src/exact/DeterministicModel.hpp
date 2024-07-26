//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_DETERMINISTIC_MODEL_H
#define DPARP_DETERMINISTIC_MODEL_H

#include "../classes/Parameters.hpp"
#include "../classes/Input.hpp"
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

  bool Run();

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
