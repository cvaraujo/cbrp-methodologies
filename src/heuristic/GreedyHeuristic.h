//
// Created by carlos on 24/07/24.
//

#ifndef DPARP_GREEDYHEURISTIC_H
#define DPARP_GREEDYHEURISTIC_H

#include "Include.h"
#include "Graph.h"

class GreedyHeuristic
{
    Graph *graph;
    vector<vector<pair<int, int>>> y;
    vector<pair<int, int>> x;
    float objective_value;

public:
    GreedyHeuristic(Graph *graph);

    float SolveScenario(vector<double> cases, vector<int> time, float route_time_increase, int max_tries, vector<int> &y, vector<dpair> &x);

    float Run(float route_time_increase, int max_tries, vector<vector<pair<int, int>>> &sol_x, vector<vector<pair<int, int>>> &sol_y);
};

#endif
