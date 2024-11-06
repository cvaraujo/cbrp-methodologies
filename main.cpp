#include "src/common/BlockConnection.hpp"
#include "src/common/Knapsack.hpp"
#include "src/classes/Graph.hpp"
#include "src/classes/Input.hpp"
#include "src/exact/DeterministicModel.hpp"
#include "src/exact/StochasticModel.hpp"
#include "src/heuristic/GreedyHeuristic.hpp"

int main(int argc, const char *argv[])
{
  string file_graph = argv[1];
  string file_scenarios = argv[2];
  string result_file = argv[3];
  string model = argv[4];
  string solution_type = argv[5];
  stringstream convTime(argv[6]), convPreprocessing(argv[7]), convFracCut(argv[8]);

  if (model != "MTZ" && model != "EXP")
  {
    cout << "[!] Solution type not found!" << endl;
    exit(EXIT_FAILURE);
  }

  if (solution_type != "WALK" && solution_type != "TRAIL")
  {
    cout << "[!] Solution type not found!" << endl;
    exit(EXIT_FAILURE);
  }

  int T;
  bool frac_cut = false, preprocessing = false;
  int default_vel = 20, neblize_vel = 10;
  double alpha = 0.8;
  bool is_trail = (solution_type == "TRAIL") ? true : false;
  bool walk_mtz = (model == "MTZ" && !is_trail) ? true : false;

  convTime >> T;
  convFracCut >> frac_cut;
  convPreprocessing >> preprocessing;

  Input *input = new Input(file_graph, file_scenarios, preprocessing, is_trail, walk_mtz, default_vel, neblize_vel, T, alpha);
  DeterministicModel *dm = new DeterministicModel(input);

  Solution sol = dm->Run(false, "3600", model, frac_cut);
  sol.WriteSolution(result_file);

  return 0;
}
