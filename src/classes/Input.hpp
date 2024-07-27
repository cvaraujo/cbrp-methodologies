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
    int S = 0, T = 0, graph_adapt = 0, default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;
    Graph *graph;
    ShortestPath *sp;
    BlockConnection *bc;
    vector<Scenario> scenarios;

public:
    Input(Graph *graph, vector<Scenario> scenarios, ShortestPath *sp) : graph(graph), scenarios(scenarios), sp(sp) {}

    Input(string file_graph, string scenarios_graph, int graph_adapt, int default_vel, int neblize_vel, int T, double alpha);

    ~Input() {}

    void reduceGraphToPositiveCases();

    void loadScenarios(string instance);

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

    double getAlpha() { return this->alpha; }

    void setAlpha(double alpha) { this->alpha = alpha; }

    Graph *getGraph() { return this->graph; }

    void setGraph(Graph *graph) { this->graph = graph; }

    void setScenarios(vector<Scenario> scenarios) { this->scenarios = scenarios; }

    Scenario getScenario(int i) { return this->scenarios[i]; }

    void setScenario(int i, Scenario scenario) { this->scenarios[i] = scenario; }

    int getS() { return this->S; }

    void setS(int s) { this->S = s; }

    int getT() { return this->T; }

    void setT(int t) { this->T = t; }

    void setBlockConnection(BlockConnection *bc) { this->bc = bc; }

    BlockConnection *getBlockConnection() { return this->bc; }
};

#endif