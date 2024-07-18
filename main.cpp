#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"
#include "headers/DeterministicModel.h"

float solveDM(Graph *graph, string result_file, vector<bool> &attended, vector<pair<int, int>> &x)
{
  DeterministicModel dm(graph);
  dm.createVariables();
  dm.initModelCompact(true);
  dm.solveCompact("60");
  dm.writeSolution(result_file);

  for (int i = 0; i < graph->getN(); i++)
    for (auto b : graph->nodes[i].second)
      if (dm.y[i][b].get(GRB_DoubleAttr_X) > 0.5)
        attended[b] = true;

  for (int i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->arcs[i])
      if (dm.x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
        x.push_back(make_pair(i, arc->getD()));

  return dm.model.get(GRB_DoubleAttr_ObjVal);
}

float solveDMGetRoute(Graph *graph, string result_file, vector<pair<int, int>> &x)
{
  DeterministicModel dm(graph);
  dm.createVariables();
  dm.initModelCompact(true);
  dm.solveCompact("60");
  dm.writeSolution(result_file);

  for (int i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->arcs[i])
      if (dm.x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
        x.push_back(make_pair(i, arc->getD()));

  return dm.model.get(GRB_DoubleAttr_ObjVal);
}

float runScenarioWithRoute(Graph *graph, vector<bool> &attended, vector<pair<int, int>> x)
{
  DeterministicModel dm(graph);
  dm.createVariables();
  dm.initModelCompact(false);

  for (int i = 0; i <= graph->getN(); i++)
  {
    for (auto arc : graph->arcs[i])
    {
      bool found = false;
      for (auto p : x)
      {
        if (p.first == i && p.second == arc->getD())
        {
          dm.model.addConstr(dm.x[i][arc->getD()] == 1);
          found = true;
          break;
        }
      }
      if (!found)
        dm.model.addConstr(dm.x[i][arc->getD()] == 0);
    }
  }
  dm.model.update();
  dm.solveCompact("3600");

  for (int i = 0; i < graph->getN(); i++)
    for (auto b : graph->nodes[i].second)
      if (dm.y[i][b].get(GRB_DoubleAttr_X) > 0.5)
        attended[b] = true;

  return dm.model.get(GRB_DoubleAttr_ObjVal);
}

float ResourceActionResults(string graph_filename, string scenarios_filename, int T, float alpha, int s)
{
  vector<float> individual_results;

  Graph *default_graph = new Graph(graph_filename, scenarios_filename, 0, 20, 10, T);
  Graph *g1 = new Graph(*default_graph);
  Graph *g2 = new Graph(*default_graph);

  int B = default_graph->getB();
  for (int b = 0; b < B; b++)
    g2->cases_per_block[b] = default_graph->scenarios[s].cases_per_block[b];

  // Update Graph 1
  vector<bool> attended_g1 = vector<bool>(B, false);
  for (int b = 0; b < B; b++)
    g1->cases_per_block[b] += alpha * g2->cases_per_block[b];

  // Solve Base Scenario
  vector<pair<int, int>> x_base = vector<pair<int, int>>();
  solveDM(g1, "result_g1.txt", attended_g1, x_base);

  attended_g1 = vector<bool>(B, false);
  float base_val = runScenarioWithRoute(default_graph, attended_g1, x_base);

  // Solve Second Stage
  for (int b = 0; b < B; b++)
    if (attended_g1[b])
      g2->cases_per_block[b] = g2->cases_per_block[b] * (1 - alpha);

  vector<pair<int, int>> x = vector<pair<int, int>>();
  float sec_stage_val = default_graph->scenarios[s].probability * solveDMGetRoute(g2, "results_g2.txt", x);

  // Apply solution to all other scenarios
  float final_of = base_val + sec_stage_val;

  for (int scn = 0; scn < default_graph->getS(); scn++)
  {
    if (scn == s)
      continue;

    for (int b = 0; b < B; b++)
    {
      g2->cases_per_block[b] = 0;

      if (attended_g1[b])
        g2->cases_per_block[b] = (1 - alpha) * float(default_graph->scenarios[scn].cases_per_block[b]);
      else
        g2->cases_per_block[b] = default_graph->scenarios[scn].cases_per_block[b];
    }

    final_of += default_graph->scenarios[scn].probability * runScenarioWithRoute(g2, attended_g1, x);
  }

  cout << "First Stage: " << base_val << ", Sec. Stage: " << sec_stage_val << ", Final OF: " << final_of << endl;

  return final_of;
}

float ExpectationExpectedValueResults(string graph_filename, string scenarios_filename, int T, float alpha)
{
  vector<int> scenarios;
  vector<float> individual_results;

  Graph *default_graph = new Graph(graph_filename, scenarios_filename, 0, 20, 10, T);
  Graph *g1 = new Graph(*default_graph);
  Graph *g2 = new Graph(*default_graph);

  float probability = default_graph->scenarios[0].probability;
  for (int s = 0; s < default_graph->getS(); s++)
    scenarios.push_back(s);

  int B = default_graph->getB();
  for (int b = 0; b < B; b++)
  {
    g2->cases_per_block[b] = 0;
    for (auto s : scenarios)
      g2->cases_per_block[b] += default_graph->scenarios[s].cases_per_block[b];
    g2->cases_per_block[b] = g2->cases_per_block[b] / float(scenarios.size());
  }

  // Update Graph 1
  vector<bool> attended_g1 = vector<bool>(B, false);
  for (int b = 0; b < B; b++)
    g1->cases_per_block[b] += alpha * g2->cases_per_block[b];

  // Solve Base Scenario
  vector<pair<int, int>> x_base = vector<pair<int, int>>();
  solveDM(g1, "result_g1.txt", attended_g1, x_base);

  attended_g1 = vector<bool>(B, false);
  float base_val = runScenarioWithRoute(default_graph, attended_g1, x_base);

  // Solve Second Stage
  for (int b = 0; b < B; b++)
    if (attended_g1[b])
      g2->cases_per_block[b] = g2->cases_per_block[b] * (1 - alpha);

  vector<pair<int, int>> x = vector<pair<int, int>>();
  float sec_stage_val = probability * solveDMGetRoute(g2, "results_g2.txt", x);

  // Apply solution to all other scenarios
  float final_of = base_val;

  for (int scn = 0; scn < default_graph->getS(); scn++)
  {
    for (int b = 0; b < B; b++)
    {
      g2->cases_per_block[b] = 0;

      if (attended_g1[b])
        g2->cases_per_block[b] = (1 - alpha) * float(default_graph->scenarios[scn].cases_per_block[b]);
      else
        g2->cases_per_block[b] = default_graph->scenarios[scn].cases_per_block[b];
    }

    final_of += default_graph->scenarios[scn].probability * runScenarioWithRoute(g2, attended_g1, x);
  }

  cout << "First Stage: " << base_val << ", Sec. Stage: " << sec_stage_val << ", Final OF: " << final_of << endl;

  return final_of;
}

int main(int argc, const char *argv[])
{
  float eev = ExpectationExpectedValueResults(argv[1], argv[2], 700, 0.8);
  float rs = ResourceActionResults(argv[1], argv[2], 700, 0.8, 0);

  cout << "EEV: " << eev << " -- " << "RAP: " << rs << endl;
  // Create Graph
  // cout << "Loading the graph!" << endl;
  // Graph *g = new Graph(argv[1], argv[2], 0, 20, 10, 40);

  // g->setS(0);
  // DeterministicModel *dm = new DeterministicModel(g);
  // dm->createVariables();
  // dm->initModelCompact(true);
  // dm->solveCompact("3600");
  // // sm->check_solution(30, 10);
  // dm->writeSolution("output.txt");
  return 0;
}
