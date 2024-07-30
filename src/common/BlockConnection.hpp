//
// Created by Carlos on 20/04/2024.
//

#ifndef SCBRP_BC_H
#define SCBRP_BC_H

#include "ShortestPath.hpp"
#include "../classes/Parameters.hpp"
#include "../classes/Graph.hpp"
#include "../classes/Arc.hpp"

using namespace std;

class BlockConnection
{

private:
    Graph *graph;
    ShortestPath *sp;
    map<string, double> blocks_attend_cost;
    map<string, vector<int>> blocks_attend_path;
    vector<vector<int>> block_2_block_cost;

public:
    BlockConnection() {};

    BlockConnection(Graph *graph, ShortestPath *sp)
    {
        this->graph = graph;
        this->sp = sp;
    };

    ~BlockConnection() {};

    int HeuristicBlockConnection(Graph *graph, ShortestPath *sp, vector<int> blocks, string key);

    static string GenerateStringFromIntVector(vector<int> blocks)
    {
        stringstream result;
        copy(blocks.begin(), blocks.end(), ostream_iterator<int>(result, ""));

        return result.str();
    }

    vector<vector<Arc>> createLayeredDag(vector<int> nodes, map<int, int> &dag_2_graph, int &V);

    vector<int> getBestOrderToAttendBlocks(vector<int> blocks);

    void setBlocksAttendCost(string key, double cost) { this->blocks_attend_cost[key] = cost; }

    void setBlocksAttendPath(string key, vector<int> path) { this->blocks_attend_path[key] = path; }

    double getBlocksAttendCost(string key) { return this->blocks_attend_cost[key]; }

    vector<int> getBlocksAttendPath(string key) { return this->blocks_attend_path[key]; }

    void setBlock2BlockCost(vector<vector<int>> block_2_block_cost) { this->block_2_block_cost = block_2_block_cost; }

    vector<vector<int>> getBlock2BlockCost() { return this->block_2_block_cost; }

    bool keyExists(string key) { return this->blocks_attend_cost.find(key) != this->blocks_attend_cost.end(); }

    void updateBlock2BlockCost(int i, int j, int cost) { this->block_2_block_cost[i][j] = cost; }
};

#endif // SCBRP_SHP_H
