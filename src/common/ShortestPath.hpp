
//
// Created by Carlos on 20/04/2024.
//

#ifndef SCBRP_SHP_H
#define SCBRP_SHP_H

#include "../classes/Parameters.hpp"
#include "../classes/Graph.hpp"

class ShortestPath
{

private:
    Graph *graph;
    vector<vector<double>> block_2_block_shp;
    vector<vector<int>> dist, next;
    vector<vector<vector<int>>> ij_path;

public:
    ShortestPath(Graph *graph)
    {
        this->graph = graph;
        allPairsShortestPath();
    };

    ShortestPath() {};

    ~ShortestPath() {}

    void allPairsShortestPath();

    static int DijkstraLayeredDAG(vector<vector<Arc>> dag, int n, int s, int t, vector<int> &pred);

    int ShortestPathST(int s, int t, vector<int> &path);

    int SHPBetweenBlocks(int b1, int b2, set<int> &nodes, map<int, map<int, bool>> &arcs);
};

#endif // SCBRP_SHP_H
