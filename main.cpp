#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"

int main(int argc, const char *argv[])
{
  // Create Graph
  cout << "Loading the graph!" << endl;
  Graph *g = new Graph(argv[1], 0, 20, 10, 120);
  // g->showGraph();
  g->run_spprc();

  // cout << "Instantiating Gurobi model!" << endl;
  // Lagrangean *lg = new Lagrangean(g, default_vel, spraying_vel, insecticide_ml_min);
  // lg->lagrangean_relax();

  // cout << lg->timeBlock(1, spraying_vel) + lg->timeBlock(3, spraying_vel) + lg->timeBlock(4, spraying_vel) + lg->timeBlock(5, spraying_vel) << endl;
  // cout << lg->timeArc(100, default_vel) << endl;
  return 0;
}
