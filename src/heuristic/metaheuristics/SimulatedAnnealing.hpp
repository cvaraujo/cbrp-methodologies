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
    double temperature = 1.0, temperature_max = 100, alpha = 1.15;
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
        int acc_change = 0, refuse_change = 0;
        double best_of = best_solution->getOf();

        while (temperature < temperature_max) {
            for (int i = 0; i < max_iterations; i++) {
                curr_change = ls->RunDefaultPerturbation(true);
                if (ChangeUtils::isEmpty(curr_change))
                    continue;

                delta = curr_change.delta;
                ap = delta != 0 ? exp((delta * temperature) / best_of) : 0.0;

                cout << "[*] Delta: " << delta << ", AP: " << ap << endl;
                if (delta > 0 || dis(gen) < ap) {
                    acc_change++;
                    cout << "\t[*] Apply Changes!" << endl;
                    current_solution->ApplyChanges(curr_change);
                } else
                    refuse_change++;

                if (best_solution->getOf() < current_solution->getOf()) {
                    best_solution = new Solution(*current_solution);
                    best_of = best_solution->getOf();
                }

                cout << "[*] Temperature: " << temperature << ", Temp. Max: " << temperature_max << endl;
                cout << "\t[*] Iteration: " << i << ", BestSol: " << best_solution->getOf() << ", CurrSol: " << current_solution->getOf() << endl;
                cout << "\t[*] Checking the OF from CurrSol:" << endl;
                current_solution->ComputeCurrentSolutionOF();
                // getchar();
            }
            temperature *= alpha;
            // getchar();
        }
        cout << "[!] Acepted Changes: " << acc_change << ", Refused: " << refuse_change << endl;
        ls->PostProcessing(*best_solution);
    };
};

#endif
