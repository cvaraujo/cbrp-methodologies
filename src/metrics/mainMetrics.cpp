#include <sys/stat.h>
#include <string>
#include <chrono>
#include "../src/classes/Input.hpp"
#include "../src/exact/DeterministicModel.hpp"
#include "../src/exact/StochasticModel.hpp"

Solution solveDM(Input *input, string result_file)
{
  DeterministicModel dm(input);
  dm.createVariables();
  dm.initModel("MTZ");
  dm.solveCompact("600");
  return dm.getSolution();
}

Solution solveSM(Input *input, string result_file)
{
  StochasticModel sm(input);
  sm.createVariables();
  sm.initModel("MTZ");
  sm.solveCompact("600");
  return sm.getSolution();
}

Solution solveSMfromStartSol(Input *input, Solution start_sol, string result_file)
{
  StochasticModel sm(input);
  sm.createVariables();
  sm.initModel("MTZ");
  sm.setStartSolution(start_sol);

  sm.solveCompact("600");
  return sm.getSolution();
}

float getRealOfFromSolution(Input *input, int s, vector<double> cases_fs, vector<double> cases_ss, Solution sol, float alpha)
{
  float of = 0.0;
  auto Y = sol.getY();

  // First Stage Solution
  for (auto b : Y[0])
    of += cases_fs[b] + alpha * cases_ss[b];

  // Second Stage Solution
  for (auto b : Y[s])
  {
    bool found = false;

    if (find(Y[0].begin(), Y[0].end(), b) == Y[0].end())
      of += float(cases_ss[b]);
    else
      of += (1.0 - alpha) * float(cases_ss[b]);
  }

  return of;
}

float WaitNSeeResults(Input *input, float alpha)
{
  Input *input1 = new Input(*input);
  Graph *g1 = input1->getGraph();

  float of = 0.0;

  // Reset scenarios from G1
  input1->setS(1);
  input1->setScenarios(vector<Scenario>(1));
  float probability = 1.0 / float(input->getS());

  for (int s = 0; s < input1->getS(); s++)
  {

    Scenario scn = input->getScenario(s);
    scn.setProbability(1.0);
    input1->setScenario(0, scn);

    // Get Pair solution
    Solution sol = solveSM(input1, "result_ws.txt");

    // Get Real Objective Function
    auto scenarioCasesPerBlocks = input1->getScenario(s).getCases();
    of += probability * getRealOfFromSolution(input1, s, g1->getCasesPerBlock(), scenarioCasesPerBlocks, sol, alpha);
  }

  return of;
}

float ExpectationExpectedValueResults(input *Input, float alpha, vector<vector<pair<int, int>>> &x_sol, vector<vector<pair<int, int>>> &y_sol)
{
  Input *input1 = new Input(*input);
  Input *input2 = new Input(*input);

  Graph g1 = input1.getGraph();
  Graph g2 = input2.getGraph();

  float of = 0.0;

  // Reset scenarios from G1
  g1->setS(1);
  g1->scenarios = vector<Scenario>(1);
  float probability = 1.0;

  // Construct the average Scenario
  vector<float> cases_per_block = vector<float>(graph->getB(), 0);
  for (int b = 0; b < graph->getB(); b++)
  {
    float avg_cases = 0.0;
    for (int s = 0; s < graph->getS(); s++)
      avg_cases += graph->scenarios[s].cases_per_block[b];
    cases_per_block[b] = avg_cases / float(graph->getS());
  }

  // Put AVG Scenario in G1
  Scenario avg_scenario = Scenario(probability, cases_per_block);
  g1->scenarios[0] = avg_scenario;

  // Get First Stage Solution
  vector<vector<pair<int, int>>> x_s(2), y_s(2);
  solveSM(g1, "result_ev_g1.txt", x_s, y_s);

  y_sol.push_back(y_s[0]), x_sol.push_back(x_s[0]);

  // Compute second stage solutions
  probability = 1.0 / float(graph->getS());

  for (int s = 0; s < graph->getS(); s++)
  {
    // Update cases in Scenario s
    g1->cases_per_block = g2->scenarios[s].cases_per_block;
    bool all_zero = true;

    for (auto pair : y_s[0])
    {
      g1->cases_per_block[pair.second] = g1->cases_per_block[pair.second] * (1.0 - alpha);
    }

    for (int b = 0; b < graph->getB(); b++)
    {
      if (g1->cases_per_block[b] > 0)
      {
        all_zero = false;
        break;
      }
    }

    // Solve Second Stage
    vector<pair<int, int>> x, y;
    if (!all_zero)
      solveDM(g1, "result_ws_g2.txt", x, y);

    y_sol.push_back(y), x_sol.push_back(x);

    // Get Real Objective value
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y_s[0], y, alpha);
  }
  return of;
}

float StochasticModelResults(Graph *graph, float alpha, vector<vector<pair<int, int>>> x_sol, vector<vector<pair<int, int>>> y_sol, bool start_from_sol)
{
  // Get First Stage Solution
  int S = graph->getS();
  float probability = 1.0 / float(S), of = 0.0;
  vector<vector<pair<int, int>>> x_s, y_s;
  for (int s = 0; s <= S; s++)
  {
    x_s.push_back(vector<pair<int, int>>());
    y_s.push_back(vector<pair<int, int>>());
  }

  if (start_from_sol)
    solveSMfromStartSol(graph, "result_stochastic_g1.txt", x_s, y_s, x_sol, y_sol);
  else
    solveSM(graph, "result_stochastic_g1.txt", x_s, y_s);

  for (int s = 0; s < S; s++)
  {
    // Get Real Objective value
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y_s[0], y_s[s + 1], alpha);
  }
  return of;
}

float DeterministicModelResults(Graph *graph, float alpha)
{
  Graph *g1 = new Graph(*graph);
  Graph *g2 = new Graph(*graph);
  float of = 0.0;

  // Reset scenarios from G1
  float probability = 1.0 / float(graph->getS());

  vector<pair<int, int>> x, y;
  solveDM(g1, "result_ds_g1.txt", x, y);

  for (int s = 0; s < graph->getS(); s++)
  {
    // Update cases in Scenario s
    g1->cases_per_block = g2->scenarios[s].cases_per_block;
    bool all_zero = true;

    for (auto pair : y)
    {
      g1->cases_per_block[pair.second] = g1->cases_per_block[pair.second] * (1.0 - alpha);
    }

    for (int b = 0; b < graph->getB(); b++)
    {
      if (g1->cases_per_block[b] > 0)
      {
        all_zero = false;
        break;
      }
    }

    // Solve Second Stage
    vector<pair<int, int>> x_s, y_s;
    if (!all_zero)
      solveDM(g1, "result_dt_g2.txt", x_s, y_s);

    // Get Real Objective value
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y, y_s, alpha);
  }

  return of;
}

int main(int argc, const char *argv[])
{
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/simulated-alto-santo/alto-santo-300-1.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/instances/simulated-alto-santo/scenarios-alto-santo-300-1.txt";
  int default_vel = 20, neblize_vel = 10, T = 200;
  double alpha = 0.8;
  bool use_preprocessing = false, is_trail = false, block_2_block_graph = false;

  Input *input = new Input(file_graph, file_scenarios, use_preprocessing, is_trail, block_2_block_graph, default_vel, neblize_vel, T, alpha);
  input->filterMostDifferentScenarios(5);

  vector<vector<pair<int, int>>> x, y;

  float ws = 0; // WaitNSeeResults(graph, alpha);
  float ev = ExpectationExpectedValueResults(graph, alpha, x, y);
  float sm = StochasticModelResults(graph, alpha, x, y, true);
  float dt = DeterministicModelResults(graph, alpha);

  cout << "DT: " << dt << ", EEV: " << ev << ", " << "RP: " << sm << ", WS: " << ws << endl;

  // // File name
  // std::string filename = "analysis.txt";

  // // Open the file in append mode
  // std::ofstream file;
  // file.open(filename, std::ios::app);

  // // Check if the file is open
  // if (!file.is_open())
  // {
  //   std::cerr << "Failed to open the file." << std::endl;
  //   return 1;
  // }

  // // Write to the file
  // file << "DT: " << dt << " EEV: " << ev << " " << "RP: " << sm << " WS: " << ws << std::endl;

  // // Close the file
  // file.close();
  return 0;
}