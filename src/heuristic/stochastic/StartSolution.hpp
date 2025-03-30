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

private:
    static Route *BuildRouteFromGreedyHeuristic(Input *input, vector<int> y, double of)
    {
#ifndef Silence
        cout << "[*] BuildRouteGromGH" << endl;
        cout << "\t[*] OF: " << of << endl;
        cout << "\t[*] Y: ";
        for (auto i : y)
            cout << i << ", ";
        cout << endl;
#endif
        string key = input->getBlockConnection()->GenerateStringFromIntVector(y);
        vector<int> path = input->getBlockConnectionRoute(key);
        int route_time = input->getBlockConnectionTime(key);

        int attend_time = 0;
        for (auto b : y)
            attend_time += input->getGraph()->getTimePerBlock(b);

        Route *route = new Route(input, y, path, attend_time, route_time, of);
        return route;
    }

public:
    static Solution CreateStartSolution(Input *input)
    {
#ifndef Silence
        cout << "[*] Create Start Solution!" << endl;
#endif
        Graph *graph = input->getGraph();
        int S = input->getS(), T = input->getT(), B = graph->getB();
        double alpha = input->getAlpha();

        vector<int> y_0 = vector<int>(), y = vector<int>();
        Solution solution = Solution(input);

        // Solving the first stage problem
        GreedyHeuristic greedy_heuristic = GreedyHeuristic(input);

        vector<double> cases_per_block = vector<double>(B, 0);
        vector<int> time_per_block = graph->getTimePerBlock();

        for (int b = 0; b < B; b++)
            cases_per_block[b] = input->getFirstStageProfit(b);

        double of = greedy_heuristic.SolveScenario(cases_per_block, time_per_block, T, y_0);

#ifndef Silence
        cout << "[**] First Stage OF = " << of << endl;
#endif

        Route *route = BuildRouteFromGreedyHeuristic(input, y_0, of);
        solution.AddScenarioSolution(0, route, of);

        // Solve second stage problems
        for (int s = 1; s <= S; s++)
        {
            // Update second stage costs
            y = vector<int>();
            Utils::GetSecondStageCosts(input, s - 1, y_0, cases_per_block);
            double scenario_of = input->getScenarioProbability(s - 1) * greedy_heuristic.SolveScenario(cases_per_block, time_per_block, T, y);
            Route *route = BuildRouteFromGreedyHeuristic(input, y, scenario_of);
            solution.AddScenarioSolution(s, route, scenario_of);
        }

        // Update OF
        cout << "[!!!] Final Stochastic Start Solution OF: " << of << endl;
        return solution;
    };
};

#endif
