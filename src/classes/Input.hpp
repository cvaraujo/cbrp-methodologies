#ifndef DPARP_INPUT_H
#define DPARP_INPUT_H

#include "Parameters.hpp"
#include "Graph.hpp"
#include "Scenario.hpp"
#include "../common/ShortestPath.hpp"

class Input
{
private:
    int S, T, graph_adapt, default_vel, neblize_vel;
    double alpha;
    Graph *graph;
    vector<Scenario> scenarios;
    ShortestPath *sp;

public:
    Input(Graph *graph, vector<Scenario> scenarios, ShortestPath *sp) : graph(graph), scenarios(scenarios), sp(sp) {}

    Input(string file_graph, string scenarios_graph, int graph_adapt, int default_vel, int neblize_vel, int T, double alpha);

    ~Input() {}

    Graph *getGraph() { return this->graph; }

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
};

#endif