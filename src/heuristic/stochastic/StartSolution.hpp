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
        vector<vector<int>> y = vector<vector<int>>(S + 1, vector<int>());
        vector<vector<int_pair>> x = vector<vector<int_pair>>(S + 1, vector<int_pair>());

        // Getting all cases in a matrix
        vector<vector<double>> cases_per_block = vector<vector<double>>(S + 1, vector<double>());
        cases_per_block[0] = graph->getCasesPerBlock();

        for (int s = 0; s < S; s++)
            cases_per_block[s + 1] = input->getScenario(s).getCases();

        // Solving the first stage problem
        GreedyHeuristic greedy_heuristic = GreedyHeuristic(input);

        // Update first stage costs
        Utils::UpdateFirstStageCosts(input, cases_per_block);

        auto time_per_block = graph->getTimePerBlock();
        double of = greedy_heuristic.SolveScenario(cases_per_block[0], time_per_block, 0.05, 10, T, y[0], x[0]);
        cout << "OF from Scenario: " << of << endl;

        // Solve second stage problems
        for (int s = 0; s < S; s++)
        {
            // Update second stage costs
            Utils::UpdateSecondStageCosts(input, y[0], cases_per_block, s + 1);
            of += input->getScenario(s).getProbability() * greedy_heuristic.SolveScenario(cases_per_block[s + 1], time_per_block, 0.05, 10, T, y[s + 1], x[s + 1]);
        }

        return Solution(of, y, x);
    };
};

#endif
