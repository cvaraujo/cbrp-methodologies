#ifndef MULTISTART_HPP
#define MULTISTART_HPP

#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "../../heuristic/GreedyHeuristic.hpp"
#include "LocalSearch.hpp"

class MultiStart {
    Input *input;
    vector<double> cases_per_block_prop, sim_acc_cases_prop, sim_incidence_prop;

  private:
    void SaveCasesPerBlockProportions() {
        Graph *graph = input->getGraph();
        int B = graph->getB();
        double total_cases = accumulate(graph->getCasesPerBlock().begin(), graph->getCasesPerBlock().end(), 0.0);

        for (int b = 0; b < B; b++) {
            this->cases_per_block_prop[b] = graph->getCasesPerBlock(b) / total_cases;
        }
    }

    void SaveSimheuristicProportions() {
        Graph *graph = input->getGraph();
        int B = graph->getB();
        double total_acc_cases = 0.0, total_incidence = 0.0;

        for (int b = 0; b < B; b++) {
            total_acc_cases += input->getSimheuristicBlockAccCases(b);
            total_incidence += input->getSimheuristicBlockIncidence(b);
        }

        for (int b = 0; b < B; b++) {
            this->sim_acc_cases_prop[b] = input->getSimheuristicBlockAccCases(b) / total_acc_cases;
            this->sim_incidence_prop[b] = input->getSimheuristicBlockIncidence(b) / total_incidence;
        }
    }

  public:
    explicit MultiStart(Input *input) {
        this->input = input;
        int B = input->getGraph()->getB();
        this->cases_per_block_prop = vector<double>(B, 0.0);
        this->sim_acc_cases_prop = vector<double>(B, 0.0);
        this->sim_incidence_prop = vector<double>(B, 0.0);
    }

    ~MultiStart() = default;

    Solution *GenerateNewSolution(const string &objective_type) {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis(0.0, 1.0);

        Graph *graph = input->getGraph();
        int T = input->getT(), B = graph->getB();
        auto *solution = new Solution(input);
        GreedyHeuristic greedy_heuristic = GreedyHeuristic(input);

        vector<int> y = vector<int>(), time_per_block = graph->getTimePerBlock();
        y.reserve(B);
        vector<double> profit_per_block = vector<double>(B, 0.0);

        if (objective_type == "FIRST_STAGE") {
            for (int b = 0; b < B; b++) {
                profit_per_block[b] = cases_per_block_prop[b] + dis(gen);
            }
        } else if (objective_type == "SIM_ACC_CASES") {
            for (int b = 0; b < B; b++) {
                profit_per_block[b] = sim_acc_cases_prop[b] + dis(gen);
            }
        } else if (objective_type == "SIM_INCIDENCE") {
            for (int b = 0; b < B; b++) {
                profit_per_block[b] = sim_incidence_prop[b] + dis(gen);
            }
        } else if (objective_type == "FULL") {
            for (int b = 0; b < B; b++) {
                profit_per_block[b] = cases_per_block_prop[b] + sim_acc_cases_prop[b] + sim_incidence_prop[b] + dis(gen);
            }
        } else {
            for (int b = 0; b < B; b++) {
                profit_per_block[b] = dis(gen);
            }
        }

        greedy_heuristic.SolveScenario(profit_per_block, time_per_block, T, y);
        double of = 0.0;

        for (int b : y) {
            of += graph->getCasesPerBlock(b);
        }

        auto *route = new Route(input, y);
        of += LocalSearch::RunLocalSearch(input, route, profit_per_block);

        solution->AddScenarioSolution(0, route, of);
        return solution;
    };
};

#endif