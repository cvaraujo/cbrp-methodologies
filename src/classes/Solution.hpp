//
// Created by Carlos on 26/07/2024.
//
#include "Changes.hpp"
#include "Parameters.hpp"
#include "Route.hpp"
#include <utility>

#ifndef DPARP_SOLUTION_H
#define DPARP_SOLUTION_H

class Solution {

  private:
    double of = 0.0, UB = INF, runtime = 0.0;
    int time_used = 0, route_time = 0, num_lazy_cuts = 0, num_frac_cuts = 0, solver_nodes = 0;
    Input *input = nullptr;
    vector<vector<int>> y;
    vector<vector<int_pair>> x;
    vector<Route *> routes;
    vector<double> scenario_profit;

  public:
    Solution(double of, vector<vector<int>> y, vector<vector<int_pair>> x, Input *input) {
        this->of = of;
        this->y = std::move(y);
        this->x = std::move(x);
        this->input = input;
    }

    explicit Solution(Input *input) {
        this->input = input;
        int S = this->input->getS();
        this->y = vector<vector<int>>(S + 1, vector<int>());
        this->x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
        this->routes = vector<Route *>(S + 1);
        this->scenario_profit = vector<double>(S + 1, 0.0);
    };

    Solution() {
        this->of = 0.0;
        this->y = vector<vector<int>>();
        this->x = vector<vector<int_pair>>();
    }

    Solution(Input *input, double of, double UB, double runtime, int time_used, int num_lazy_cuts, int num_frac_cuts, int solver_nodes, vector<vector<int>> y, vector<vector<int_pair>> x) {
        this->input = input;
        this->of = of;
        this->UB = UB;
        this->runtime = runtime;
        this->time_used = time_used;
        this->num_lazy_cuts = num_lazy_cuts;
        this->num_frac_cuts = num_frac_cuts;
        this->solver_nodes = solver_nodes;
        this->y = y;
        this->x = x;
    }

    ~Solution() {
        scenario_profit.clear();
        y.clear(), x.clear();
        for (Route *r : this->routes)
            delete r;
        routes.clear();
    }

    // Construtor de cópia (deep copy)
    Solution(const Solution &other) {
        if (this != &other) {
            this->of = other.of;
            this->UB = other.UB;
            this->runtime = other.runtime;
            this->time_used = other.time_used;
            this->num_lazy_cuts = other.num_lazy_cuts;
            this->num_frac_cuts = other.num_frac_cuts;
            this->solver_nodes = other.solver_nodes;
            this->y = other.y;
            this->x = other.x;
            this->scenario_profit = other.scenario_profit;

            for (Route *r : other.routes)
                routes.push_back(new Route(*r)); // Copia profunda
        }
    }

    // Operador de atribuição (deep copy)
    Solution &operator=(const Solution &other) {
        if (this != &other) {
            // Libera as rotas existentes
            for (Route *r : routes)
                delete r;
            routes.clear();

            this->of = other.of;
            this->UB = other.UB;
            this->runtime = other.runtime;
            this->time_used = other.time_used;
            this->num_lazy_cuts = other.num_lazy_cuts;
            this->num_frac_cuts = other.num_frac_cuts;
            this->solver_nodes = other.solver_nodes;
            this->y = other.y;
            this->x = other.x;
            this->scenario_profit = other.scenario_profit;

            // Copia profunda das rotas
            for (Route *r : other.routes)
                routes.push_back(new Route(*r));
        }
        return *this;
    }

    void WriteSolution(const string &output_file) {
        ofstream output;
        output.open(output_file);
        Graph *graph = this->input->getGraph();

        output << "N: " << graph->getN() << endl;
        output << "M: " << graph->getM() << endl;
        output << "B: " << graph->getB() << endl;
        output << "S: " << this->input->getS() << endl;
        output << "Alpha: " << this->input->getAlpha() << endl;
        output << "LB: " << this->of << endl;
        output << "UB: " << this->UB << endl;
        output << "Gurobi_Nodes: " << this->solver_nodes << endl;
        output << "Lazy_cuts: " << this->num_lazy_cuts << endl;
        output << "Frac_cuts: " << this->num_frac_cuts << endl;
        output << "Runtime: " << this->runtime << endl;

        for (int s = 0; s <= getS(); s++) {
            output << "Scenario: " << s << endl;
            for (auto arc : this->x[s])
                output << "X: " << arc.first << " " << arc.second << endl;

            for (auto b : this->y[s])
                output << "Y: " << b << endl;
            output << "Route_Time: " << this->time_used << endl;
        }
        output.close();
#ifndef Silence
        cout << "[*] Solution writed!" << endl;
#endif
    };

    void AddScenarioSolution(int s, vector<int_pair> x, vector<int> y, double profit) {
        if (routes.empty()) {
            int S = this->input->getS();
            this->y = vector<vector<int>>(S + 1, vector<int>());
            this->x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
            this->routes = vector<Route *>(S + 1);
            this->scenario_profit = vector<double>(S + 1, 0.0);
        }

        this->routes[s] = new Route(this->input, x, y);
        this->x[s] = x, this->y[s] = y;
        this->scenario_profit[s] = profit;
    };

    void AddScenarioSolution(int s, Route *route, double profit) {
        this->routes[s] = route;
        this->y[s] = route->getSequenceOfAttendingBlocks();
        this->scenario_profit[s] = profit;
        this->of += profit;
    };

    void ReplaceScenarioSolution(int s, vector<int_pair> x, vector<int> y, Route *route) {
        this->routes[s] = route;
        this->x[s] = std::move(x), this->y[s] = std::move(y);
    };

    void ScenarioBlockSwapWithoutOF(int s, int b1, int b2) {
        this->y[s].erase(find(this->y[s].begin(), this->y[s].end(), b1));
        this->y[s].push_back(b2);
        this->routes[s]->SwapInRouteBlocks(b1, b2);
    };

    void ScenarioBlockSwap(int s, int b1, int b2, double delta) {
        this->y[s].erase(find(this->y[s].begin(), this->y[s].end(), b1));
        this->y[s].push_back(b2);
        this->routes[s]->SwapInRouteBlocks(b1, b2);
        this->of += delta;
    };

    void ApplySwaps(vector<pair<int, int_pair>> &swaps, double delta) {
        for (auto swap : swaps) {
            int s = swap.first;
            int_pair blocks = swap.second;
            int b1 = blocks.first, b2 = blocks.second;
            this->routes[s]->SwapInRouteBlocks(b1, b2);
        }
        this->of += delta;
    }

    void ApplyChanges(const Change &change) {
        if (!ChangeUtils::isEmpty(change)) {
            if (ChangeUtils::hasDeletions(change)) {
                ApplyRemovals(change.deletions);
            }
            if (ChangeUtils::hasInsertions(change)) {
                ApplyInsertions(change.insertions);
            }
            if (ChangeUtils::hasSwaps(change)) {
                ApplySwaps(change.swaps);
            }

            this->of += change.delta;
        }
    }

    void ApplyRemovals(const vector<int_pair> &removals) {
        for (auto &[scenario, block] : removals) {
            this->routes[scenario]->RemoveBlockFromRoute(block);
        }
    }

    void ApplyInsertions(const vector<int_pair> &insertions) {
        for (auto &[scenario, block] : insertions) {
            this->routes[scenario]->AddBlockToRoute(block, true);
        }
    }

    void ApplySwaps(const vector<pair<int, int_pair>> &swaps) {
        for (auto &[scenario, blocks] : swaps) {
            int to_remove = blocks.first, to_insert = blocks.second;
            this->routes[scenario]->GeneralSwapBlocks(to_remove, to_insert);
        }
    }

    double ComputeCurrentSolutionOF() {
        double of = 0;
        Graph *graph = input->getGraph();
        vector<double> cases_per_block = graph->getCasesPerBlock();
        vector<bool> attended_first_stage = vector<bool>(graph->getB(), false);

        for (auto b : this->y[0]) {
            attended_first_stage[b] = true;
            of += cases_per_block[b];

            for (int s = 0; s < input->getS(); s++) {
                auto *scn = input->getScenario(s);
                of += input->getAlpha() * input->getScenario(s)->getProbability() * scn->getCasesPerBlock(b);
            }
        }

        // cout << "[!] First Stage OF: " << of << endl;

        for (int s = 0; s < input->getS(); s++) {
            Scenario *scn = input->getScenario(s);
            for (auto b : this->y[s + 1]) {
                if (attended_first_stage[b])
                    of += scn->getProbability() * (1.0 - input->getAlpha()) * scn->getCasesPerBlock(b);
                else
                    of += scn->getProbability() * scn->getCasesPerBlock(b);
            }
        }
        cout << "[!] Second Stage OF: " << of << endl;
        getchar();
        return of;
    }

    double getScenarioProfit(int s) { return this->scenario_profit[s]; }

    void setScenarioProfit(int s, double profit) { this->scenario_profit[s] = profit; }

    void updateScenarioProfit(int s, double profit) { this->scenario_profit[s] += profit; }

    int getS() { return (input != nullptr) ? input->getS() : 0; }

    Graph *getGraph() { return input->getGraph(); }

    vector<Route *> getRoutes() { return this->routes; }

    void setRoute(Route *route, int s) { this->routes[s] = route; }

    Route *getRouteFromScenario(int s) { return this->routes[s]; }

    double getOf() { return of; }

    void setOf(double of) { this->of = of; }

    double getUB() { return UB; }

    void setUB(double UB) { this->UB = UB; }

    double getRuntime() { return runtime; }

    void setRuntime(double runtime) { this->runtime = runtime; }

    vector<int> getYFromScenario(int s) { return y[s]; }

    vector<int_pair> getXFromScenario(int s) { return x[s]; }

    vector<vector<int>> getY() { return y; }

    void setY(vector<vector<int>> y) { this->y = y; }

    vector<vector<int_pair>> getX() { return x; }

    void setX(vector<vector<int_pair>> x) { this->x = x; }

    void addAttendedToY(int s, int block) { this->y[s].push_back(block); }

    void addArcToX(int s, int o, int d) { this->x[s].push_back(make_pair(o, d)); }
};

#endif
