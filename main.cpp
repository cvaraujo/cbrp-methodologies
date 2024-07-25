#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"
#include "headers/GreedyHeuristic.h"
#include "headers/StochasticModel.h"

int main(int argc, const char *argv[])
{
  int T = 10000;
  float alpha = 0.8;
  Graph *graph = new Graph(argv[1], argv[2], stoi(argv[3]), 20, 10, T, 99999, alpha);
  GreedyHeuristic gh(graph);

  // for (int b = 0; b < graph->getB(); b++)
  // {
  //   cout << "B" << b << ": ";
  //   for (auto i : graph->nodes_per_block[b])
  //     cout << "N" << i << " ";
  //   cout << endl;
  // }
  // cout << "----------------------" << endl
  // TODO: fix OF
  vector<vector<pair<int, int>>> sol_x;
  vector<vector<pair<int, int>>> sol_y;
  float of = gh.Run(0.05, 5, sol_x, sol_y);
  cout << "Final objective: " << of << endl;

  // StochasticModel sm(graph);
  // sm.createVariables();
  // sm.initModelCompact(false);

  // for (int s = 0; s <= graph->getS(); s++)
  //   sm.setStartSolution(s, sol_x[s], sol_y[s]);

  // sm.solveCompact("600");
  // sm.writeSolution("result_sm.txt");

  return 0;
}
