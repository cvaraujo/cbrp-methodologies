//
// Created by carlos on 24/07/24.
//

#ifndef DPARP_GREEDYHEURISTIC_H
#define DPARP_GREEDYHEURISTIC_H

#include "../classes/Parameters.hpp"
#include "../classes/Input.hpp"
#include "../common/Knapsack.hpp"
#include "../common/BlockConnection.hpp"

class GreedyHeuristic
{
    Input *input;
    vector<vector<pair<int, int>>> y;
    vector<pair<int, int>> x;
    float objective_value;

public:
    GreedyHeuristic(Input *input)
    {
        this->input = input;
    };

    float SolveScenario(vector<double> cases, vector<int> time, float route_time_increase, int max_tries, vector<int> &y, vector<int_pair> &x);

    float Run(float route_time_increase, int max_tries, vector<vector<pair<int, int>>> &sol_x, vector<vector<pair<int, int>>> &sol_y);
};

#endif
