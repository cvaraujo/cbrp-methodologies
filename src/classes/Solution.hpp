//
// Created by Carlos on 26/07/2024.
//
#include "Parameters.hpp"
#include "Route.hpp"

#ifndef DPARP_SOLUTION_H
#define DPARP_SOLUTION_H

class Solution
{

private:
  double of = 0.0, UB = INF, runtime = 0.0;
  int time_used = 0, route_time = 0, num_lazy_cuts = 0, num_frac_cuts = 0, solver_nodes = 0;
  Input *input;
  vector<vector<int>> preds, y;
  vector<vector<int_pair>> x;
  vector<Route *> routes;

public:
  Solution(double of, vector<vector<int>> y, vector<vector<int_pair>> x, Input *input)
  {
    this->of = of;
    this->y = y;
    this->x = x;
    this->input = input;
  }

  Solution(Input *input)
  {
    this->input = input;
  };

  Solution()
  {
    this->of = 0.0;
    this->y = vector<vector<int>>();
    this->x = vector<vector<int_pair>>();
  }

  Solution(double of, double UB, double runtime, int time_used, int num_lazy_cuts, int num_frac_cuts, int solver_nodes, vector<vector<int>> y, vector<vector<int_pair>> x)
  {
    this->of = of;
    this->UB = UB;
    this->runtime = runtime;
    this->time_used = time_used;
    this->num_lazy_cuts = num_lazy_cuts;
    this->num_frac_cuts = num_frac_cuts;
    this->solver_nodes = solver_nodes;
    this->y = y;
    this->x = x;
  }

  ~Solution() { y.clear(), x.clear(); }

  void WriteSolution(string output_file)
  {
    ofstream output;
    output.open(output_file);

    output << "LB: " << this->of << endl;
    output << "UB: " << this->UB << endl;
    output << "Gurobi Nodes: " << this->solver_nodes << endl;
    output << "LAZY_CUTS: " << this->num_lazy_cuts << endl;
    output << "FRAC_CUTS: " << this->num_frac_cuts << endl;
    output << "Runtime: " << this->runtime << endl;

    for (int s = 0; s <= getS(); s++)
    {
      output << "Scenario: " << s << endl;
      for (auto arc : this->x[s])
        output << "X: " << arc.first << " " << arc.second << endl;
      for (auto b : this->y[s])
        output << "Y: " << b << endl;
      output << "Route Time: " << this->time_used << endl;
    }
    output.close();
#ifndef Silence
    cout << "[*] Solution writed!" << endl;
#endif
  };

  void AddScenarioSolution(int s, vector<int_pair> x, vector<int> y)
  {
    if (routes.empty())
    {
      int S = this->input->getS();
      this->y = vector<vector<int>>(S + 1, vector<int>());
      this->x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
      this->routes = vector<Route *>(S + 1);
    }

    Graph *graph = this->input->getGraph();
    this->routes[s] = new Route(this->input, x, y);
    this->x[s] = x, this->y[s] = y;
  };

  void ReplaceScenarioSolution(int s, vector<int_pair> x, vector<int> y, Route *route)
  {
    this->routes[s] = route;
    this->x[s] = x, this->y[s] = y;
  };

  int getS() { return input->getS(); }

  Graph *getGraph() { return input->getGraph(); }

  vector<Route *> getRoutes() { return this->routes; }

  void setRoute(Route *route, int s) { this->routes[s] = route; }

  Route *getRouteFromScenario(int s) { return this->routes[s]; }

  double getOf() { return of; }

  void setOf(double of) { this->of = of; }

  double getUB() { return UB; }

  void setUB(double UB) { this->UB = UB; }

  double getRuntime() { return runtime; }

  void setRuntime(double runtime) { this->runtime = runtime; }

  vector<int> getYFromScenario(int s) { return y[s]; }

  vector<int_pair> getXFromScenario(int s) { return x[s]; }

  vector<vector<int>> getY() { return y; }

  void setY(vector<vector<int>> y) { this->y = y; }

  vector<vector<int_pair>> getX() { return x; }

  void setX(vector<vector<int_pair>> x) { this->x = x; }

  void addAttendedToY(int s, int block) { this->y[s].push_back(block); }

  void addArcToX(int s, int o, int d) { this->x[s].push_back(make_pair(o, d)); }
};

#endif