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
    double objective_value;

    pair<double, double> getBlockSecondStageProfitAvg(vector<Scenario> scenarios, int b)
    {
        double profit = 0;
        for (int s = 0; s < this->input->getS(); s++)
            profit += (input->getAlpha() * input->getScenario(s).getProbability() * input->getScenario(s).getCasesPerBlock(b));
        return make_pair((profit / this->input->getS()), profit);
    };

    double getBlockSecondStageProfitSum(vector<Scenario> scenarios, int b)
    {
        double profit = 0;
        for (int s = 0; s < this->input->getS(); s++)
            profit += (input->getAlpha() * input->getScenario(s).getProbability() * input->getScenario(s).getCasesPerBlock(b));
        return profit;
    };

    double getRealValueOfFirstStageSolution(vector<int> y, vector<double> profit)
    {
        double of = 0;
        for (auto b : y)
            of += profit[b];
        return of;
    };

public:
    GreedyHeuristic(Input *input)
    {
        this->input = input;
    };

    double SolveScenario(vector<double> cases, vector<int> time, double route_time_increase, int max_tries, vector<int> &y, vector<int_pair> &x);

    double Run(double route_time_increase, int max_tries, bool use_avg, vector<vector<pair<int, int>>> &sol_x, vector<vector<pair<int, int>>> &sol_y);
};

#endif
