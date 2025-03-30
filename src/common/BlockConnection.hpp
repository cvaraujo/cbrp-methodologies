//
// Created by Carlos on 20/04/2024.
//

#ifndef SCBRP_BC_H
#define SCBRP_BC_H

#include "ShortestPath.hpp"
#include "../classes/Parameters.hpp"
#include "../classes/Graph.hpp"
#include "../classes/Arc.hpp"

class BlockConnection
{

private:
    Graph *graph;
    ShortestPath *sp;
    map<string, double> blocks_attend_cost;
    map<string, vector<int>> blocks_attend_path, best_order_attend_blocks;
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

    set<int> computeBlock2BlockCost()
    {
        // Remove unecessary arcs between blocks
        int B = graph->getB();
        this->block_2_block_cost = vector<vector<int>>(B, vector<int>(B, INF));
        set<int> set_of_used_nodes, nodes_in_path, union_set;

        for (int b = 0; b < B - 1; b++)
        {
            for (int b2 = b + 1; b2 < B; b2++)
            {
                nodes_in_path = set<int>();
                int cost = this->sp->SHPBetweenBlocks(b, b2, nodes_in_path);

                if (cost < INF)
                {
                    union_set = set<int>();
                    set_union(nodes_in_path.begin(), nodes_in_path.end(), set_of_used_nodes.begin(), set_of_used_nodes.end(), inserter(union_set, union_set.begin()));
                    set_of_used_nodes = union_set;
                    this->block_2_block_cost[b][b2] = this->block_2_block_cost[b2][b] = cost;
                }
            }
        }

        return set_of_used_nodes;
    };

    static string GenerateStringFromIntVector(vector<int> blocks)
    {
        stringstream result;
        vector<int> keys(blocks);
        sort(keys.begin(), keys.end());
        copy(keys.begin(), keys.end(), ostream_iterator<int>(result, "_"));

        return result.str();
    };

    vector<vector<Arc>> createLayeredDag(vector<int> nodes, map<int, int> &dag_2_graph, int &V);

    vector<int> getBestOrderToAttendBlocks(vector<int> blocks);

    void setBlocksAttendCost(string key, double cost) { this->blocks_attend_cost[key] = cost; }

    void setBlocksAttendPath(string key, vector<int> path) { this->blocks_attend_path[key] = path; }

    double getBlocksAttendCost(string key) { return this->blocks_attend_cost[key]; }

    vector<int> getBlocksAttendPath(string key) { return this->blocks_attend_path[key]; }

    void setBlock2BlockCost(vector<vector<int>> block_2_block_cost) { this->block_2_block_cost = block_2_block_cost; }

    vector<vector<int>> getBlock2BlockCost() { return this->block_2_block_cost; }

    void setBestOrderToAttendBlocks(string key, vector<int> order) { this->best_order_attend_blocks[key] = order; }

    vector<int> getBestOrderToAttendBlocks(string key) { return this->best_order_attend_blocks[key]; }

    int getBlock2BlockCost(int i, int j) { return this->block_2_block_cost[i][j]; }

    bool keyExists(string key) { return this->blocks_attend_cost.find(key) != this->blocks_attend_cost.end(); }

    void updateBlock2BlockCost(int i, int j, int cost) { this->block_2_block_cost[i][j] = cost; }
};

#endif // SCBRP_SHP_H
