#ifndef MULTISTART_HPP
#define MULTISTART_HPP

#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "../../heuristic/GreedyHeuristic.hpp"
#include "LocalSearch.hpp"
#include <chrono>
#include <random>

enum StrategyType {
    PURE_CURRENT,
    SIM_ACC_CASES,
    SIM_INCIDENCE,
    FULL_COMBINED,
    BIASED_CURRENT,
    BIASED_FULL
};

struct CandidateResult {
    vector<int> blocks;
    double deterministic_of = 0.0;
    double tiebreak_of = 0.0;
};

class MultiStart {
    Input *input;
    vector<double> cases_per_block_prop, sim_acc_cases_prop, sim_incidence_prop;

  private:
    void SaveCasesPerBlockProportions() {
        Graph *graph = input->getGraph();
        int B = graph->getB();
        vector<double> cases = graph->getCasesPerBlock();
        double total_cases = accumulate(cases.begin(), cases.end(), 0.0);
        if (total_cases <= 0.0)
            return;

        for (int b = 0; b < B; b++)
            cases_per_block_prop[b] = cases[b] / total_cases;
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
            sim_acc_cases_prop[b] = (total_acc_cases > 0.0)
                                        ? input->getSimheuristicBlockAccCases(b) / total_acc_cases
                                        : 0.0;
            sim_incidence_prop[b] = (total_incidence > 0.0)
                                        ? input->getSimheuristicBlockIncidence(b) / total_incidence
                                        : 0.0;
        }
    }

    vector<StrategyType> GetActiveStrategies() {
        vector<StrategyType> strategies = {PURE_CURRENT};
        bool has_scenarios = input->getS() > 0;

        if (has_scenarios) {
            strategies.push_back(SIM_ACC_CASES);
            strategies.push_back(SIM_INCIDENCE);
            strategies.push_back(FULL_COMBINED);
        }

        strategies.push_back(BIASED_CURRENT);
        if (has_scenarios)
            strategies.push_back(BIASED_FULL);

        return strategies;
    }

    vector<double> BuildProfitVector(StrategyType strategy, int B, Graph *graph,
                                     mt19937 &gen) {
        vector<double> profit(B, 0.0);
        gamma_distribution<double> gamma_a(2.0, 1.0);
        gamma_distribution<double> gamma_b(5.0, 1.0);
        constexpr double PERTURBATION = 0.3;

        switch (strategy) {

        case PURE_CURRENT: {
            for (int b = 0; b < B; b++) {
                profit[b] = graph->getCasesPerBlock(b);
                if (profit[b] <= 0.0 && input->getS() > 0)
                    profit[b] = sim_acc_cases_prop[b] * 0.001;
            }
            break;
        }

        case SIM_ACC_CASES: {
            for (int b = 0; b < B; b++)
                profit[b] = sim_acc_cases_prop[b];
            break;
        }

        case SIM_INCIDENCE: {
            for (int b = 0; b < B; b++)
                profit[b] = sim_incidence_prop[b];
            break;
        }

        case FULL_COMBINED: {
            for (int b = 0; b < B; b++)
                profit[b] = graph->getCasesPerBlock(b) + sim_incidence_prop[b];
            break;
        }

        case BIASED_CURRENT: {
            vector<double> cpb = graph->getCasesPerBlock();
            double max_val = *max_element(cpb.begin(), cpb.end());
            double scale = PERTURBATION * max(max_val, 1.0);
            for (int b = 0; b < B; b++) {
                double xa = gamma_a(gen), xb = gamma_b(gen);
                double beta_sample = xa / (xa + xb);
                profit[b] = graph->getCasesPerBlock(b) + beta_sample * scale;
            }
            break;
        }

        case BIASED_FULL: {
            double max_val = 0.0;
            for (int b = 0; b < B; b++)
                max_val = max(max_val,
                              graph->getCasesPerBlock(b) + sim_incidence_prop[b]);
            double scale = PERTURBATION * max(max_val, 1.0);
            for (int b = 0; b < B; b++) {
                double xa = gamma_a(gen), xb = gamma_b(gen);
                double beta_sample = xa / (xa + xb);
                profit[b] = graph->getCasesPerBlock(b) + sim_incidence_prop[b] +
                            beta_sample * scale;
            }
            break;
        }
        }

        return profit;
    }

    static CandidateResult BuildAndEvaluateSolution(Input *input,
                                                    vector<double> &profit,
                                                    const vector<int> &time_per_block,
                                                    int T, int B) {
        GreedyHeuristic greedy(input);
        vector<int> y;
        y.reserve(B);

        greedy.SolveScenario(profit, time_per_block, T, y);

        if (y.empty())
            return {};

        auto *route = new Route(input, y);
        LocalSearch::RunLocalSearch(input, route, profit);
        vector<int> final_blocks = route->getSequenceOfAttendingBlocks();
        Graph *graph = input->getGraph();
        double det_of = 0.0, tiebreak_of = 0.0;

        for (int b : final_blocks) {
            det_of += graph->getCasesPerBlock(b);
            if (graph->getCasesPerBlock(b) <= 0.0) {
                for (int s = 0; s < input->getS(); s++)
                    tiebreak_of += input->getCasesFromScenarioBlock(s, b);
            }
        }

        delete route;
        return {final_blocks, det_of, tiebreak_of};
    }

  public:
    explicit MultiStart(Input *input) {
        this->input = input;
        int B = input->getGraph()->getB();
        cases_per_block_prop = vector<double>(B, 0.0);
        sim_acc_cases_prop = vector<double>(B, 0.0);
        sim_incidence_prop = vector<double>(B, 0.0);
    }

    ~MultiStart() = default;

    Solution *GenerateNewSolution(const string &objective_type) {
        Graph *graph = input->getGraph();
        int T = input->getT(), B = graph->getB();
        vector<int> time_per_block = graph->getTimePerBlock();

        SaveCasesPerBlockProportions();
        if (input->getS() > 0)
            SaveSimheuristicProportions();

        vector<StrategyType> strategies = GetActiveStrategies();
        random_device rd;
        mt19937 gen(rd());

        int best_idx = -1;
        vector<CandidateResult> candidates(strategies.size());

        for (size_t i = 0; i < strategies.size(); i++) {
            vector<double> profit = BuildProfitVector(strategies[i], B, graph, gen);
            candidates[i] = BuildAndEvaluateSolution(input, profit, time_per_block, T, B);

            if (candidates[i].blocks.empty())
                continue;

            if (best_idx == -1 ||
                candidates[i].deterministic_of > candidates[best_idx].deterministic_of ||
                (candidates[i].deterministic_of == candidates[best_idx].deterministic_of &&
                 candidates[i].tiebreak_of > candidates[best_idx].tiebreak_of)) {
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx == -1)
            return new Solution(input);

        auto *solution = new Solution(input);
        auto *route = new Route(input, candidates[best_idx].blocks);
        solution->AddScenarioSolution(0, route,
                                      candidates[best_idx].deterministic_of);
        return solution;
    };
};

#endif
