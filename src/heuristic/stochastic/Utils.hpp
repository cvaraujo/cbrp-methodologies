//
// Created by carlos on 07/12/24.
//

#ifndef DPARP_STOCHASTIC_UTILS_H
#define DPARP_STOCHASTIC_UTILS_H

#include "../../classes/Parameters.hpp"
#include "../../classes/Input.hpp"

class Utils
{

public:
    static void UpdateFirstStageCosts(Input *input, vector<vector<double>> &cases_per_block)
    {
        double alpha = input->getAlpha();
        Graph *graph = input->getGraph();
        int B = graph->getB();
        double cost;

        for (int b = 0; b < B; b++)
        {
            cost = cases_per_block[0][b];
            for (int s = 0; s < input->getS(); s++)
                cost += alpha * input->getScenario(s).getProbability() * input->getScenario(s).getCasesPerBlock(b);
            cases_per_block[0][b] = cost;
        }
    };

    static void UpdateSecondStageCosts(Input *input, vector<int> first_stage_solution, vector<vector<double>> &cases_per_block, int s)
    {
        double alpha = input->getAlpha(), cost = 0;
        for (int b : first_stage_solution)
            cases_per_block[s][b] = (1 - alpha) * cases_per_block[s][b];
    };
};

#endif
