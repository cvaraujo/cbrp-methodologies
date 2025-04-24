//
// Created by carlos on 23/04/25
//

#ifndef DPARP_STOCHASTIC_SA_H
#define DPARP_STOCHASTIC_SA_H

#include "../../classes/Parameters.hpp"
#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "../stochastic/Utils.hpp"
#include "../stochastic/LocalSearch.hpp"

class SimulatedAnnealing
{

private:
    int temperature = 1;
    float temperature_min = 0.0001;
    float alpha = 0.9;
    int max_iterations = 100;
    string delta_type = "moderate";
    bool first_improve = true;

public:
    void Run(Input *input, Solution solution, random_device &rd)
    {
        mt19937 gen(rd()); // Mersenne Twister RNG
        uniform_real_distribution<> dis(0.0, 1.0);

        Solution *best_solution = new Solution(solution);
        Solution *current_solution = new Solution(solution);
        LocalSearch *ls = new LocalSearch(input, current_solution);

        double delta, ap;
        vector<pair<int, int_pair>> curr_swaps, best_swaps;
        while (temperature > temperature_min)
        {
            for (int i = 0; i < max_iterations; i++)
            {
                if (best_solution->getOf() < current_solution->getOf())
                    best_solution = current_solution;

                // TODO: improve this to use all types of swap
                delta = ls->ComputeInRouteSwapBlocksStartScenario(delta_type, curr_swaps, first_improve);
                ap = exp(delta / temperature);

                if (ap > dis(gen))
                {
                    best_swaps = curr_swaps;
                    current_solution->ApplySwaps(best_swaps, delta);
                }
            }
            temperature *= alpha;
        }
    };
};

#endif
