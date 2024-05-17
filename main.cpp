#include <sys/stat.h>
#include <string>
#include <chrono>
#include "headers/Graph.h"

int main(int argc, const char *argv[])
{
  // Create Graph
  cout << "Loading the graph!" << endl;
  Graph *g = new Graph(argv[1], 0, 20, 10, 65);
  // g->run_spprc();

  vector<pair<int, int>> route;
  route.push_back(make_pair(10, 5));
  route.push_back(make_pair(5, 4));
  route.push_back(make_pair(7, 11));
  route.push_back(make_pair(4, 7));

  cout << "Blocks in Route" << endl;
  auto blocks = g->getBlocksFromRoute(route);
  for (auto b : blocks)
    cout << b << " ";
  cout << endl;

  cout << "Knapsack Vectors" << endl;
  vector<int> cases, time;
  g->populateKnapsackVectors(blocks, cases, time);
  for (int i = 0; i < blocks.size(); i++)
    cout << cases[i] << " = " << time[i] << ", ";
  cout << endl;

  cout << "Running Knapsack" << endl;
  vector<int> y;
  cout << "Profit: " << g->knapsack(y, cases, time, 17) << endl;
  for (auto b : y)
    cout << "Selected Block: " << *blocks.begin() + b << endl;

  // vector<int> selected;
  // cout << "Cost: " << g->knapsack(selected, 66) << endl;
  // for (auto item : selected)
  //   cout << "Item: " << item << endl;

  // cout << "Instantiating Gurobi model!" << endl;
  // Lagrangean *lg = new Lagrangean(g, default_vel, spraying_vel, insecticide_ml_min);
  // lg->lagrangean_relax();

  // cout << lg->timeBlock(1, spraying_vel) + lg->timeBlock(3, spraying_vel) + lg->timeBlock(4, spraying_vel) + lg->timeBlock(5, spraying_vel) << endl;
  // cout << lg->timeArc(100, default_vel) << endl;
  return 0;
}
