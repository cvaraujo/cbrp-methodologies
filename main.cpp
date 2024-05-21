#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"
#include "headers/Lagrangean.h"

int main(int argc, const char *argv[])
{
  // Create Graph
  cout << "Loading the graph!" << endl;
  Graph *g = new Graph(argv[1], 0, 20, 10, 1200);
  Lagrangean *l = new Lagrangean(g);
  l->lagrangean_relax();

  return 0;
}
