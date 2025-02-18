#include "src/classes/Graph.hpp"
#include "src/classes/Input.hpp"
#include "src/heuristic/stochastic/StartSolution.hpp"
#include "src/heuristic/stochastic/LocalSearch.hpp"

/*
void test_local_search(const char *argv[])
{
  // string file_graph = argv[1];
  // string file_scenarios = argv[2];
  // string result_file = argv[3];

  // Input *input = new Input(file_graph, file_scenarios, false, true, false, 20, 10, 1200, 0.8);
  // Solution *sol = new Solution();
  // vector<int> start_route = vector<int>{10, 0, 3, 1, 2, 8, 7, 4, 5, 6, 10};
  // sol->setRoute(start_route), sol->setRouteTime(36);

  // LocalSearch ls = LocalSearch(input, sol);
  // ls.Run2Opt(2, 6), ls.Run2Opt(3, 5), ls.Run2Opt(2, 5);

  // cout << ls.getBestSolution()->getRouteTime() << endl;
  // for (auto i : start_route)
  //   cout << i << " ";
  // cout << endl;
}

int test_mathematical_models(int argc, const char *argv[])
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
*/

void test_stochastic_start_solution(const char *argv[])
{
  string file_graph = argv[1];
  string file_scenarios = argv[2];
  string result_file = argv[3];

  Input *input = new Input(file_graph, file_scenarios, false, true, false, 20, 10, 600, 0.8);
  // input->setS(24);
  cout << "[*] Input loaded!" << endl;
  Solution sol = StartSolution::CreateStartSolution(input);
  cout << "[*] Creating Solution!" << endl;
  LocalSearch::RunMoreProfitable2OPT(input, &sol, "moderate");
  // WEAK:     195.396
  // MODERATE:
  sol.WriteSolution(result_file);
  cout << "[*] Writed!" << endl;
}

int main(int argc, const char *argv[])
{
  test_stochastic_start_solution(argv);
}