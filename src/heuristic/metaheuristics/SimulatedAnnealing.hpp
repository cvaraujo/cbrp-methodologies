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

        Change best_change, curr_change;
        double delta, ap;
        vector<pair<int, int_pair>> curr_swaps, best_swaps;

        while (temperature > temperature_min) {
            cout << "[*] Temperature: " << temperature << ", Temp. Min: " << temperature_min << endl;
            for (int i = 0; i < max_iterations; i++) {
                cout << "\t[*] Iteration: " << i << ", BestSol: " << best_solution->getOf() << ", CurrSol: " << current_solution->getOf() << endl;
                // TODO: Check changes
                cout << "\t[*] Checking the OF from CurrSol: " << current_solution->ComputeCurrentSolutionOF() << endl;
                getchar();

                if (best_solution->getOf() < current_solution->getOf()) {
                    best_solution = current_solution;
                }

                curr_change = ls->RunDefaultPerturbation(true);
                if (ChangeUtils::isEmpty(curr_change))
                    continue;

                delta = curr_change.delta;
                ap = exp(delta / temperature);
                cout << "[*] Delta: " << delta << ", AP: " << ap << endl;

                if (delta > 0 || ap > dis(gen)) {
                    current_solution->ApplyChanges(curr_change);
                }
            }
            temperature *= alpha;
        }
    };
};

#endif
