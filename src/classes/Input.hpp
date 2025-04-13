#ifndef DPARP_INPUT_H
#define DPARP_INPUT_H

#include "Parameters.hpp"
#include "Graph.hpp"
#include "Scenario.hpp"
#include "../common/ShortestPath.hpp"
#include "../common/BlockConnection.hpp"

class Input
{
private:
    int S = 0, T = 1200, default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;
    bool preprocessing = false, is_trail = false, walk_mtz_model = false;
    Graph *graph;
    ShortestPath *sp = nullptr;
    BlockConnection *bc = nullptr;
    vector<Scenario> scenarios;
    vector<double> first_stage_profit;
    vector<vector<vector<Arc *>>> arcs_in_path;
    vector<vector<int>> arc_length;

public:
    Input(Graph *graph, vector<Scenario> scenarios, ShortestPath *sp) : graph(graph), scenarios(scenarios), sp(sp) {}

    Input(string file_graph, string scenarios_graph, bool preprocessing, bool is_trail, bool walk_mtz_model, int default_vel, int neblize_vel, int T, double alpha);

    Input(string file_graph, string scenarios_graph, int default_vel, int nebulize_vel, int T, double alpha);

    Input(Input *input)
    {
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

    ~Input() {}

    void updateFirstStageCases()
    {
        int B = graph->getB();
        this->first_stage_profit = vector<double>(B);

        for (int b = 0; b < B; b++)
        {
            this->first_stage_profit[b] = graph->getCasesPerBlock(b);
            for (int s = 0; s < S; s++)
                this->first_stage_profit[b] += alpha * scenarios[s].getProbability() * scenarios[s].getCasesPerBlock(b);
        }
    };

    double getFirstStageProfit(int b) { return this->first_stage_profit[b]; }

    double getCasesFromScenarioBlock(int s, int b) { return this->scenarios[s].getCasesPerBlock(b); }

    vector<double> getCasesFromScenario(int s) { return this->scenarios[s].getCases(); }

    double getSecondStageProfit(int s, int b) { return alpha * scenarios[s].getProbability() * scenarios[s].getCasesPerBlock(b); }

    double getScenarioProbability(int s) { return this->scenarios[s].getProbability(); }

    vector<int> getBlockConnectionRoute(string key) { return this->bc->getBlocksAttendPath(key); };

    int getBlockConnectionTime(string key) { return this->bc->getBlocksAttendCost(key); };

    vector<int> getBestOrderToAttendBlocks(string key) { return this->bc->getBestOrderToAttendBlocks(key); };

    bool isArcRoute(int i, int j)
    {
        if (this->arcs_in_path[i][j].size() <= 0)
            this->getArcTime(i, j);

        return this->arcs_in_path[i][j].size() > 1;
    }

    int getArcTime(int i, int j)
    {
        int N = graph->getN();
        if (i >= N || j >= N)
            return 0;

        if (this->arc_length[i][j] != -1)
            return this->arc_length[i][j];

        Arc *arc = graph->getArc(i, j);
        int length = 0;
        if (arc == nullptr)
        {
            vector<int> path;
            length = this->sp->ShortestPathST(i, j, path);

            // Validate path
            for (int k = 0; k < path.size() - 1; k++)
                this->arcs_in_path[i][j].push_back(graph->getArc(path[k], path[k + 1]));
        }
        else
            length = arc->getLength();

        this->arc_length[i][j] = length;

        return this->arc_length[i][j];
    }

    int getBlockTime(int b) { return graph->getTimePerBlock(b); }

    void updateBlocksInGraph(map<int, int> positive_block_to_block, set<int> set_of_used_nodes, vector<vector<bool>> used_arcs);

    void reduceGraphToPositiveCases();

    void loadScenarios(string instance);

    void getSetOfNodesPreprocessing(set<int> &used_nodes, vector<vector<bool>> &used_arcs);

    void walkAdaptMTZModel();

    void filterMostDifferentScenarios(int new_s);

    void showScenarios()
    {
        for (int i = 0; i < S; i++)
        {
            cout << "Scenario i: " << i << ": " << scenarios[i].getProbability() << endl;
            for (int b = 0; b < graph->getB(); b++)
            {
                if (scenarios[i].getCasesPerBlock(b) > 0)
                    cout << b << ": " << scenarios[i].getCasesPerBlock(b) << endl;
            }
        }
    }

    vector<Scenario> getScenarios() { return this->scenarios; }

    ShortestPath *getShortestPath() { return this->sp; }

    void setShortestPath(ShortestPath *sp) { this->sp = sp; }

    double getAlpha() { return this->alpha; }

    void setAlpha(double alpha) { this->alpha = alpha; }

    Graph *getGraph() { return this->graph; }

    void setGraph(Graph *graph) { this->graph = graph; }

    void setScenarios(vector<Scenario> scenarios) { this->scenarios = scenarios; }

    Scenario *getScenario(int i) { return &this->scenarios[i]; }

    void setScenario(int i, Scenario scenario) { this->scenarios[i] = scenario; }

    int getS() { return this->S; }

    void setS(int s) { this->S = s; }

    int getT() { return this->T; }

    void setT(int t) { this->T = t; }

    bool isPreprocessing() { return this->preprocessing; }

    bool isTrail() { return this->is_trail; }

    bool isWalkMtzGraph() { return this->walk_mtz_model; }

    void setBlockConnection(BlockConnection *bc) { this->bc = bc; }

    BlockConnection *getBlockConnection() { return this->bc; }

    bool isNodeInPositiveValidBlock(int node);
};

#endif