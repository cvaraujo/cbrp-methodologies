#ifndef DPARP_INPUT_H
#define DPARP_INPUT_H

#include "../common/BlockConnection.hpp"
#include "../common/ShortestPath.hpp"
#include "Graph.hpp"
#include "Parameters.hpp"
#include "Scenario.hpp"
#include <unordered_map>

class Input {
  private:
    int S = 0, T = 1200, default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;
    bool preprocessing = false, is_trail = false, walk_mtz_model = false;
    Graph *graph;
    ShortestPath *sp = nullptr;
    BlockConnection *bc = nullptr;
    vector<Scenario> scenarios;
    vector<double> first_stage_profit, time_profit_proportion;
    vector<vector<vector<Arc *>>> arcs_in_path;
    vector<vector<int>> arc_length;
    // Simheuristic only
    vector<int> simheuristic_scenario_sequence;
    unordered_map<int, int> simheuristic_block_incidences;
    unordered_map<int, int> simheuristic_block_acc_cases;

  public:
    Input(Graph *graph, vector<Scenario> scenarios, ShortestPath *sp)
        : graph(graph)
        , scenarios(std::move(scenarios))
        , sp(sp) {}

    Input(string file_graph, string scenarios_graph, bool preprocessing,
          bool is_trail, bool walk_mtz_model, int default_vel, int neblize_vel,
          int T, double alpha);

    Input(string file_graph, string scenarios_graph, int default_vel,
          int nebulize_vel, int T, double alpha);

    explicit Input(Input *input) {
        this->S = input->S;
        this->T = input->T;
        this->default_vel = input->default_vel;
        this->neblize_vel = input->neblize_vel;
        this->alpha = input->alpha;
        this->preprocessing = input->preprocessing;
        this->is_trail = input->is_trail;
        this->graph = new Graph(*input->graph);
        this->scenarios = input->scenarios;
        this->sp = new ShortestPath(*input->sp);
        this->bc = new BlockConnection(*input->bc);
    };

    ~Input() = default;

    void updateFirstStageCases() {
        int B = graph->getB();
        this->first_stage_profit = vector<double>(B);
        this->time_profit_proportion = vector<double>(B);

        for (int b = 0; b < B; b++) {
            this->first_stage_profit[b] = graph->getCasesPerBlock(b);
            for (int s = 0; s < S; s++) {
                this->first_stage_profit[b] += alpha * scenarios[s].getProbability() *
                                               scenarios[s].getCasesPerBlock(b);
            }
            this->time_profit_proportion[b] = first_stage_profit[b] > 0.0 ? first_stage_profit[b] / double(this->getBlockTime(b)) : 0.0;
        }
    };

    double getFirstStageProfit(int b) { return this->first_stage_profit[b]; }

    double getCasesFromScenarioBlock(int s, int b) {
        return this->scenarios[s].getCasesPerBlock(b);
    }

    vector<double> getCasesFromScenario(int s) {
        return this->scenarios[s].getCases();
    }

    double getSecondStageProfit(int s, int b) {
        if (scenarios[s].getCasesPerBlock(b) <= 0.0)
            return 0.0;
        return alpha * scenarios[s].getProbability() *
               scenarios[s].getCasesPerBlock(b);
    }

    double getScenarioProbability(int s) {
        return this->scenarios[s].getProbability();
    }

    vector<int> getBlockConnectionRoute(const string &key) {
        return this->bc->getBlocksAttendPath(key);
    };

    int getBlockConnectionTime(const string &key) {
        return this->bc->getBlocksAttendCost(key);
    };

    vector<int> getBestOrderToAttendBlocks(const string &key) {
        return this->bc->getBestOrderToAttendBlocks(key);
    };

    bool isArcRoute(int i, int j) {
        if (this->arcs_in_path[i][j].size() <= 0)
            this->getArcTime(i, j);

        return this->arcs_in_path[i][j].size() > 1;
    }

    int getArcTime(int i, int j) {
        int N = graph->getN();
        if (i >= N || j >= N)
            return 0;

        if (this->arc_length[i][j] != -1)
            return this->arc_length[i][j];

        Arc *arc = graph->getArc(i, j);
        int length = 0;
        if (arc == nullptr) {
            vector<int> path;
            length = this->sp->ShortestPathST(i, j, path);

            // Validate path
            for (int k = 0; k < path.size() - 1; k++)
                this->arcs_in_path[i][j].push_back(graph->getArc(path[k], path[k + 1]));
        } else
            length = arc->getLength();

        this->arc_length[i][j] = length;

        return this->arc_length[i][j];
    }

    int getBlockTime(int b) { return graph->getTimePerBlock(b); }

    void updateBlocksInGraph(map<int, int> positive_block_to_block,
                             set<int> set_of_used_nodes,
                             vector<vector<bool>> used_arcs);

    void reduceGraphToPositiveCases();

    void loadScenarios(string instance);

    void getSetOfNodesPreprocessing(set<int> &used_nodes,
                                    vector<vector<bool>> &used_arcs);

    void walkAdaptMTZModel();

    void filterMostDifferentScenarios(int new_s);

    void showScenarios() {
        for (int i = 0; i < S; i++) {
            cout << "Scenario i: " << i << ": " << scenarios[i].getProbability()
                 << endl;
            for (int b = 0; b < graph->getB(); b++) {
                if (scenarios[i].getCasesPerBlock(b) > 0)
                    cout << b << ": " << scenarios[i].getCasesPerBlock(b) << endl;
            }
        }
    }

    double getTimeProfitProportion(int b) {
        return time_profit_proportion[b];
    }

    void appendNewScenario(Scenario &scenario) {
        this->scenarios.push_back(scenario);

        // Update feedback from simulation
        for (int b = 0; b < graph->getB(); b++) {
            int cases = int(scenario.getCasesPerBlock(b));

            if (cases > 0) {
                if (this->simheuristic_block_acc_cases.find(b) == this->simheuristic_block_acc_cases.end()) {
                    this->simheuristic_block_acc_cases[b] = 0;
                    this->simheuristic_block_incidences[b] = 0;
                }

                this->simheuristic_block_acc_cases[b] += cases;
                this->simheuristic_block_incidences[b]++;
            }
        }

        // Update vector to shuffle scenarios
        this->simheuristic_scenario_sequence.push_back(this->S);
        this->S++;
    }

    vector<Scenario> getScenarios() { return this->scenarios; }

    ShortestPath *getShortestPath() { return this->sp; }

    void setShortestPath(ShortestPath *sp) { this->sp = sp; }

    [[nodiscard]] double getAlpha() const { return this->alpha; }

    void setAlpha(double alpha) { this->alpha = alpha; }

    [[nodiscard]] Graph *getGraph() const { return this->graph; }

    void setGraph(Graph *graph) { this->graph = graph; }

    void setScenarios(vector<Scenario> scenarios) { this->scenarios = std::move(scenarios); }

    Scenario *getScenario(int i) { return &this->scenarios[i]; }

    void setScenario(int i, Scenario scenario) { this->scenarios[i] = std::move(scenario); }

    [[nodiscard]] int getS() const { return this->S; }

    void setS(int s) { this->S = s; }

    [[nodiscard]] int getT() const { return this->T; }

    void setT(int t) { this->T = t; }

    [[nodiscard]] bool isPreprocessing() const { return this->preprocessing; }

    [[nodiscard]] bool isTrail() const { return this->is_trail; }

    [[nodiscard]] bool isWalkMtzGraph() const { return this->walk_mtz_model; }

    void setBlockConnection(BlockConnection *bc) { this->bc = bc; }

    BlockConnection *getBlockConnection() { return this->bc; }

    bool isNodeInPositiveValidBlock(int node);
};

#endif
