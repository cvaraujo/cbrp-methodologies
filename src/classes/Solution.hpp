//
// Created by Carlos on 26/07/2024.
//
#include "Parameters.hpp"

#ifndef DPARP_SOLUTION_H
#define DPARP_SOLUTION_H

class Solution
{

private:
  double of = 0.0, UB = INF, runtime = 0.0;
  int time_used = 0, num_lazy_cuts = 0, num_frac_cuts = 0, solver_nodes = 0;
  vector<int> y;
  vector<int_pair> x;

public:
  Solution(double of, vector<int> y, vector<int_pair> x)
  {
    this->of = of;
    this->y = y;
    this->x = x;
  }

  Solution()
  {
    this->of = 0.0;
    this->y = vector<int>();
    this->x = vector<int_pair>();
  }

  Solution(double of, double UB, double runtime, int time_used, int num_lazy_cuts, int num_frac_cuts, int solver_nodes, vector<int> y, vector<int_pair> x)
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

    for (auto arc : this->x)
      output << "X: " << arc.first << " " << arc.second << endl;
    for (auto b : this->y)
      output << "Y: " << b << endl;

    output << "Route Time: " << this->time_used << endl;
    output.close();
  };

  double getOf() { return of; }

  void setOf(double of) { this->of = of; }

  double getUB() { return UB; }

  void setUB(double UB) { this->UB = UB; }

  double getRuntime() { return runtime; }

  void setRuntime(double runtime) { this->runtime = runtime; }

  vector<int> getY() { return y; }

  void setY(vector<int> y) { this->y = y; }

  vector<int_pair> getX() { return x; }

  void setX(vector<int_pair> x) { this->x = x; }

  void addAttendedToY(int block) { this->y.push_back(block); }

  void addArcToX(int o, int d) { this->x.push_back(make_pair(o, d)); }
};

#endif