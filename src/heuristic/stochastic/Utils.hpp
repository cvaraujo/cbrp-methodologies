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
                cost += alpha * input->getScenario(s)->getProbability() * input->getScenario(s)->getCasesPerBlock(b);
            cases_per_block[0][b] = cost;
        }
    };

    static void UpdateSecondStageCosts(Input *input, vector<int> first_stage_solution, vector<vector<double>> &cases_per_block, int s)
    {
        double alpha = input->getAlpha();
        for (int b : first_stage_solution)
            cases_per_block[s][b] = (1.0 - alpha) * cases_per_block[s][b];
    };

    // static double ComputeOFFromSolution(Input *input, Solution *solution)
    // {
    //     double of = 0;
    //     Graph *graph = input->getGraph();
    //     vector<vector<double>> cases_per_block = vector<vector<double>>(input->getS() + 1, vector<double>());
    //     cases_per_block[0] = graph->getCasesPerBlock();

    //     for (int s = 0; s < input->getS(); s++)
    //         cases_per_block[s + 1] = input->getScenario(s)->getCases();

    //     Utils::UpdateFirstStageCosts(input, cases_per_block);

    //     for (int s = 0; s <= input->getS(); s++)
    //     {
    //         Route *route = solution->getRouteFromScenario(s);
    //         for (int b : route->getBlocks())
    //             of += input->getScenario(s)->getProbability() * cases_per_block[s][b];
    //     }

    //     return of;
    // };
};

#endif
