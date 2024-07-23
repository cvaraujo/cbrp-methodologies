#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"
#include "headers/DeterministicModel.h"
#include "headers/StochasticModel.h"

float solveDM(Graph *graph, string result_file, vector<pair<int, int>> &x, vector<pair<int, int>> &y)
{
  DeterministicModel dm(graph);
  dm.createVariables();
  dm.initModelCompact(true);
  dm.solveCompact("600");
  dm.writeSolution(result_file);

  for (int i = 0; i < graph->getN(); i++)
    for (auto b : graph->nodes[i].second)
      if (dm.y[i][b].get(GRB_DoubleAttr_X) > 0.5)
        y.push_back(make_pair(i, b));

  for (int i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->arcs[i])
      if (dm.x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
        x.push_back(make_pair(i, arc->getD()));

  return dm.model.get(GRB_DoubleAttr_ObjVal);
}

float solveSM(Graph *graph, string result_file, vector<vector<pair<int, int>>> &x, vector<vector<pair<int, int>>> &y)
{
  StochasticModel sm(graph);
  sm.createVariables();
  sm.initModelCompact(true);
  sm.solveCompact("600");
  sm.writeSolution(result_file);

  for (int s = 0; s <= graph->getS(); s++)
  {
    for (int i = 0; i < graph->getN(); i++)
      for (auto b : graph->nodes[i].second)
      {
        if (b == -1)
          continue;

        if (sm.y[i][b][s].get(GRB_DoubleAttr_X) > 0.5)
          y[s].push_back(make_pair(i, b));
      }

    for (int i = 0; i <= graph->getN(); i++)
      for (auto arc : graph->arcs[i])
        if (sm.x[i][arc->getD()][s].get(GRB_DoubleAttr_X) > 0.5)
          x[s].push_back(make_pair(i, arc->getD()));
  }
  return sm.model.get(GRB_DoubleAttr_ObjVal);
}

float getRealOfFromSolution(vector<float> cases_fs, vector<float> cases_ss, vector<pair<int, int>> y_fs, vector<pair<int, int>> y_ss, float alpha)
{
  float of = 0.0;

  // First Stage Solution
  for (auto pair : y_fs)
  {
    int b = pair.second;
    of += cases_fs[b] + alpha * cases_ss[b];
  }

  // Second Stage Solution
  for (auto pair : y_ss)
  {
    int b = pair.second;
    bool found = false;

    for (auto pair2 : y_fs)
    {
      if (b == pair2.second)
      {
        found = true;
        break;
      }
    }

    if (!found)
      of += float(cases_ss[b]);
    else
      of += (1.0 - alpha) * float(cases_ss[b]);
  }

  return of;
}

float WaitNSeeResults(Graph *graph, float alpha)
{
  Graph *g1 = new Graph(*graph);
  float of = 0.0;

  // Reset scenarios from G1
  g1->setS(1);
  g1->scenarios = vector<Scenario>(1);
  float probability = 1.0 / float(graph->getS());

  for (int s = 0; s < graph->getS(); s++)
  {

    g1->scenarios[0] = graph->scenarios[s];
    g1->scenarios[0].probability = 1.0;

    // Get Pair solution
    vector<vector<pair<int, int>>> x_s(2), y_s(2);
    solveSM(g1, "result_ws.txt", x_s, y_s);

    // Get Real Objective Function
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y_s[0], y_s[1], alpha);
  }

  return of;
}

float ExpectationExpectedValueResults(Graph *graph, float alpha)
{
  Graph *g1 = new Graph(*graph);
  Graph *g2 = new Graph(*graph);
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

  // Compute second stage solutions
  probability = 1.0 / float(graph->getS());

  for (int s = 0; s < graph->getS(); s++)
  {
    // Update cases in Scenario s
    g1->cases_per_block = g2->scenarios[s].cases_per_block;
    for (auto pair : y_s[0])
      g1->cases_per_block[pair.second] *= (1.0 - alpha);

    // Solve Second Stage
    vector<pair<int, int>> x, y;
    solveDM(g1, "result_ws_g2.txt", x, y);

    // Get Real Objective value
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y_s[0], y, alpha);
  }
  return of;
}

float StochasticModelResults(Graph *graph, float alpha)
{
  // Get First Stage Solution
  int S = graph->getS();
  float probability = 1 / float(S), of = 0.0;
  vector<vector<pair<int, int>>> x_s, y_s;
  for (int s = 0; s <= S; s++)
  {
    x_s.push_back(vector<pair<int, int>>());
    y_s.push_back(vector<pair<int, int>>());
  }

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
    for (auto pair : y)
      g1->cases_per_block[pair.second] *= (1.0 - alpha);

    // Solve Second Stage
    vector<pair<int, int>> x_s, y_s;
    solveDM(g1, "result_ws_g2.txt", x_s, y_s);

    // Get Real Objective value
    of += probability * getRealOfFromSolution(graph->cases_per_block, graph->scenarios[s].cases_per_block, y, y_s, alpha);
  }

  return of;
}

int main(int argc, const char *argv[])
{
  int T = 7000;
  float alpha = 0.8;
  Graph *graph = new Graph(argv[1], argv[2], stoi(argv[3]), 20, 10, T);
  graph->scenarios.erase(graph->scenarios.begin());
  graph->setS(5);

  // graph->showScenarios();
  // StochasticModel dm(graph);
  // dm.createVariables();
  // dm.initModelCompact(false);
  // dm.solveCompact("3600");
  // dm.check_solution(T, 9999);
  // dm.writeSolution("reduced.txt");
  // if (graph->getS() > 10)
  //   graph->setS(10);

  float ws = 0; // WaitNSeeResults(graph, alpha);
  float ev = ExpectationExpectedValueResults(graph, alpha);
  float sm = StochasticModelResults(graph, alpha);
  float dt = 0; // DeterministicModelResults(graph, alpha);

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
