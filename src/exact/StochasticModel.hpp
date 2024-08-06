//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_STOCHASTIC_MODEL_H
#define DPARP_STOCHASTIC_MODEL_H

#include "../classes/Parameters.hpp"
#include "../classes/Input.hpp"
#include "../classes/Solution.hpp"
#include <gurobi_c++.h>

using namespace lemon;
using namespace std;

class StochasticModel
{
  Input *input;
  GRBEnv env = GRBEnv();
  GRBModel model = GRBModel(env);
  vector<vector<GRBVar>> z, y;
  vector<vector<vector<GRBVar>>> x, t;
  int num_lazy_cuts = 0, num_frac_cuts = 0;

public:
  StochasticModel(Input *input)
  {
    if (input != nullptr)
      this->input = input;
    else
      exit(EXIT_FAILURE);
  }

  ~StochasticModel()
  {
    x.clear(), y.clear(), t.clear(), z.clear();
    model.terminate();
  }

  void setStartSolution(Solution solution)
  {
    // Route
    cout << "Setting start solution" << endl;
    for (int s = 0; s <= input->getS(); s++)
    {
      // Arc
      for (auto arc : solution.getXFromScenario(s))
      {
        // cout << "S: " << s << " O: " << arc.first << " D: " << arc.second << endl;
        x[arc.first][arc.second][s].set(GRB_DoubleAttr_Start, 1.0);
        // model.addConstr(x[arc.first][arc.second][s] == 1, "solution_x");
      }

      // Blocks
      for (int b : solution.getYFromScenario(s))
      {
        // cout << "S: " << s << " B: " << b << endl;
        y[b][s].set(GRB_DoubleAttr_Start, 1.0);
        // model.addConstr(y[b][s] == 1, "solution_y");
      }
    }
    model.update();
  }

  Solution getSolution();

  Solution Run(bool use_warm_start, string time_limit, string model, bool use_cuts, Solution solution);

  void solveExponential(string time_limit, bool frac_cut);

  void objectiveFunction();

  void createVariables();

  void initModel(string model);

  void zValue();

  void artificialNodes();

  void flowConservation();

  void maxAttending();

  void attendingPath();

  void timeConstraint();

  void compactTimeConstraint();

  void solveCompact(string time_limit);

  bool checkSolution();
};

#endif // DPARP_MODEL_H
