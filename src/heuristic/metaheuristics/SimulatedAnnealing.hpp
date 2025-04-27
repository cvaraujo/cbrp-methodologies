//
// Created by carlos on 23/04/25
//

#ifndef DPARP_STOCHASTIC_SA_H
#define DPARP_STOCHASTIC_SA_H

#include "../../classes/Input.hpp"
#include "../../classes/Parameters.hpp"
#include "../../classes/Solution.hpp"
#include "../stochastic/LocalSearch.hpp"
#include "../stochastic/Utils.hpp"

class SimulatedAnnealing {

  private:
    float temperature = 1;
    float temperature_min = 0.0001;
    float alpha = 0.9;
    int max_iterations = 100;
    string delta_type = "moderate";
    bool first_improve = true;

  public:
    void Run(Input *input, const Solution &solution, random_device &rd) {
        mt19937 gen(rd()); // Mersenne Twister RNG
        uniform_real_distribution<> dis(0.0, 1.0);

        auto *best_solution = new Solution(solution);
        auto *current_solution = new Solution(solution);
        auto *ls = new LocalSearch(input, current_solution);

        double delta, ap;
        vector<pair<int, int_pair>> curr_swaps, best_swaps;
        while (temperature > temperature_min) {
            for (int i = 0; i < max_iterations; i++) {
                if (best_solution->getOf() < current_solution->getOf())
                    best_solution = current_solution;

                // TODO: improve this to use all types of swap
                delta = ls->RunDefaultPerturbation(curr_swaps);
                ap = exp(delta / temperature);

                if (ap > dis(gen)) {
                    best_swaps = curr_swaps;
                    current_solution->ApplySwaps(best_swaps, delta);
                }
            }
            temperature *= alpha;
        }
    };
};

#endif
