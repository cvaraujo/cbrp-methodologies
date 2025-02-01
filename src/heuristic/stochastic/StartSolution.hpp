//
// Created by carlos on 07/12/24.
//

#ifndef DPARP_STOCHASTIC_STARTSOLUTION_H
#define DPARP_STOCHASTIC_STARTSOLUTION_H

#include "../../classes/Solution.hpp"
#include "../../classes/Input.hpp"
#include "../GreedyHeuristic.hpp"
#include "Utils.hpp"

class StartSolution
{

public:
    static Solution CreateStartSolution(Input *input)
    {
        Graph *graph = input->getGraph();
        int S = input->getS(), T = input->getT(), B = graph->getB();
        double alpha = input->getAlpha();
        vector<int> y_0 = vector<int>(), y;
        vector<int_pair> x = vector<int_pair>();
        Solution solution = Solution(input);

        // Getting all cases in a matrix
        vector<vector<double>> cases_per_block = vector<vector<double>>(S + 1, vector<double>());
        cases_per_block[0] = graph->getCasesPerBlock();

        for (int s = 0; s < S; s++)
            cases_per_block[s + 1] = input->getScenario(s)->getCases();

        // Solving the first stage problem
        GreedyHeuristic greedy_heuristic = GreedyHeuristic(input);

        // Update first stage costs
        Utils::UpdateFirstStageCosts(input, cases_per_block);

        auto time_per_block = graph->getTimePerBlock();
        double of = greedy_heuristic.SolveScenario(cases_per_block[0], time_per_block, 0.1, 10, T, y_0, x);
        solution.AddScenarioSolution(0, x, y_0);

        // Solve second stage problems
        for (int s = 1; s <= S; s++)
        {
            // Update second stage costs
            y = vector<int>(), x = vector<int_pair>();
            Utils::UpdateSecondStageCosts(input, y_0, cases_per_block, s);
            of += input->getScenario(s - 1)->getProbability() * greedy_heuristic.SolveScenario(cases_per_block[s], time_per_block, 0.1, 10, T, y, x);
            // cout << "OF from Scenario[" << s << "]: " << of << endl;
        }

        // Update OF
        solution.setOf(of);

        return solution;
    };
};

#endif
