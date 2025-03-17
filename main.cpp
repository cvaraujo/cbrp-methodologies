#include "src/classes/Input.hpp"
#include "src/heuristic/Lagrangean.hpp"
// #include "src/exact/DeterministicModel.hpp"
// #include "src/exact/DeterministicModelWalk.hpp"
// #include "src/exact/StochasticModel.hpp"
// #include "src/exact/StochasticModelWalk.hpp"
// #include "src/heuristic/stochastic/StartSolution.hpp"
// #include "src/heuristic/stochastic/LocalSearch.hpp"

/**
 * @brief Main function for running the deterministic or stochastic model.
 *
 * @param[in] argc Number of command line arguments.
 * @param[in] argv Command line arguments.
 *
 * The arguments should be:
 * - argv[1]: The graph file.
 * - argv[2]: The scenarios file.
 * - argv[3]: The output file.
 * - argv[4]: The model type (MTZ or EXP).
 * - argv[5]: The solution type (WALK or TRAIL).
 * - argv[6]: The time limit.
 * - argv[7]: Boolean indicating if preprocessing should be applied.
 * - argv[8]: Boolean indicating if fractional cuts should be applied.
 * - argv[9]: Optional boolean indicating if the stochastic model should be run.
 *
 * The function creates an instance of the Input class, and then
 * either runs the deterministic or stochastic model depending on the
 * value of argv[9].
 *
 * @return 0 if the function runs successfully.
 */
int main(int argc, const char *argv[])
{
  string file_graph = argv[1];
  string file_scenarios = argv[2];
  string result_file = argv[3];
  string model = argv[4];
  string solution_type = argv[5];
  stringstream convTime(argv[6]), convPreprocessing(argv[7]), convFracCut(argv[8]);
  stringstream convMaxIters(argv[9]), convReduc(argv[10]);
  // Temp
  // string stochastic_model = argv[9];

  // if (model != "MTZ" && model != "EXP")
  // {
  //   cout << "[!] Solution type not found!" << endl;
  //   exit(EXIT_FAILURE);
  // }

  // if (solution_type != "WALK" && solution_type != "TRAIL")
  // {
  //   cout << "[!] Solution type not found!" << endl;
  //   exit(EXIT_FAILURE);
  // }
  // as-1: 39
  // as-2: 240

  int T = 1200;
  bool frac_cut = false, preprocessing = false;
  int default_vel = 20, neblize_vel = 10, maxIters = 100;
  double alpha = 0.8, lambda = 2.0, reduction = 0.99;
  bool is_trail = (solution_type == "TRAIL") ? true : false;
  bool walk_mtz = false; //(model == "MTZ" && !is_trail) ? true : false;

  convTime >> T;
  convFracCut >> frac_cut;
  convPreprocessing >> preprocessing;
  convMaxIters >> maxIters;
  convReduc >> reduction;

  Input *input = new Input(file_graph, file_scenarios, preprocessing, is_trail, walk_mtz, default_vel, neblize_vel, T, alpha);
  Lagrangean *lag = new Lagrangean(input);
  lag->lagrangean_relax(result_file, lambda, maxIters, reduction);

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
