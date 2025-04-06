#include "src/classes/Input.hpp"
#include "src/heuristic/stochastic/StartSolution.hpp"
#include "src/heuristic/stochastic/LocalSearch.hpp"

// #include "src/heuristic/Lagrangean.hpp"
// #include "src/exact/DeterministicModel.hpp"
// #include "src/exact/DeterministicModelWalk.hpp"
// #include "src/exact/StochasticModel.hpp"
// #include "src/exact/StochasticModelWalk.hpp"

int main(int argc, const char *argv[])
{
  string file_graph = argv[1];
  string file_scenarios = argv[2];
  string result_file = argv[3];
  stringstream convTime(argv[4]),
      convDeltaSwap(argv[5]),
      convFirstImprove(argv[6]);

  int T = 1200;
  int default_vel = 20, neblize_vel = 10;
  bool first_improve = true;
  string delta_type = "moderate";

  convTime >> T;
  convFirstImprove >> first_improve;
  convDeltaSwap >> delta_type;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, default_vel, neblize_vel, T, alpha);

  Solution sol = StartSolution::CreateStartSolution(input);
  LocalSearch *local_search = new LocalSearch(input, &sol);

  // vector<pair<int, int_pair>> best_swaps;
  // local_search->ComputeInRouteRandomSwapBlocksStartScenario(input, &sol, delta_type, best_swaps);

  // Lagrangean *lag = new Lagrangean(input);
  // lag->lagrangean_relax(result_file, lambda, maxIters, reduction);

  // if (stochastic_model == "FALSE")
  // {
  //   // DeterministicModel *dm = new DeterministicModel(input);
  //   DeterministicModelWalk *dm = new DeterministicModelWalk(input);
  //   Solution sol = dm->Run(false, "3600", model, frac_cut);
  //   sol.WriteSolution(result_file);
  // }
  // else
  // {
  //   StochasticModel *sm = new StochasticModel(input);
  //   // StochasticModelWalk *sm = new StochasticModelWalk(input);
  //   Solution sol = sm->Run(false, "3600", model, frac_cut);
  //   sol.WriteSolution(result_file);
  // }

  return 0;
}
