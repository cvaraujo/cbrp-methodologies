#ifndef MULTISTART_HPP
#define MULTISTART_HPP

#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "../../heuristic/GreedyHeuristic.hpp"

class MultiStart {
    Input *input;

  public:
    explicit MultiStart(Input *input) {
        this->input = input;
    }

    ~MultiStart() = default;

    Solution *GenerateNewSolution() {
        random_device rd;
        mt19937 gen(rd());
        uniform_real_distribution<> dis(0.0, 1.0);

        Graph *graph = input->getGraph();
        int T = input->getT(), B = graph->getB();
        auto *solution = new Solution(input);
        GreedyHeuristic greedy_heuristic = GreedyHeuristic(input);

        vector<int> y = vector<int>(), time_per_block = graph->getTimePerBlock();
        y.reserve(B);
        // this value gives a chance for all blocks to be selected
        vector<double> profit_per_block = vector<double>(B, 0.1);

        for (int b = 0; b < B; b++) {
            int cases = input->getSimheuristicBlockAccCases(b);
            int incidence = input->getSimheuristicBlockIncidence(b);
            profit_per_block[b] = graph->getCasesPerBlock(b) + 10.0 * dis(gen);
            if (incidence > 0)
                profit_per_block[b] += double(cases) / double(incidence);
        }

        // Solve first stage
        greedy_heuristic.SolveScenario(profit_per_block, time_per_block, T, y);
        double of = 0.0;

        for (int b : y) {
            of += graph->getCasesPerBlock(b);
        }

        Route *route = new Route(input, y);
        solution->AddScenarioSolution(0, route, of);
        return solution;
    };
};

#endif