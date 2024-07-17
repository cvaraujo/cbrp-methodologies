#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"
#include "headers/StochasticModel.h"

int main(int argc, const char *argv[])
{
  // Create Graph
  cout << "Loading the graph!" << endl;
  Graph *g = new Graph(argv[1], argv[2], 0, 20, 10, 120);
  g->setS(0);
  StochasticModel *sm = new StochasticModel(g);
  sm->createVariables();
  sm->initModelCompact(false);
  sm->solveCompact("3600");
  // sm->check_solution(30, 10);
  sm->writeSolution("output.txt");
  return 0;
}
