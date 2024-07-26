
//
// Created by Carlos on 20/04/2024.
//

#ifndef SCBRP_BC_H
#define SCBRP_BC_H

#include "../classes/Input.hpp"
#include "../classes/Parameters.hpp"
#include "../classes/Graph.hpp"
#include "../classes/Arc.hpp"

using namespace std;

class BlockConnection
{

private:
    Graph *graph;
    map<string, double> blocks_attend_cost;
    map<string, vector<int>> blocks_attend_path;
    vector<vector<int>> block_2_block_shp;

public:
    BlockConnection() {};

    BlockConnection(Graph *graph) { this->graph = graph; };

    ~BlockConnection() {};

    int HeuristicBlockConnection(Input *input, vector<int> blocks, string key);

    static string GenerateStringFromIntVector(vector<int> blocks)
    {
        stringstream result;
        copy(blocks.begin(), blocks.end(), ostream_iterator<int>(result, ""));

        return result.str();
    };

    void setBlocksAttendCost(string key, double cost) { this->blocks_attend_cost[key] = cost; }

    void setBlocksAttendPath(string key, vector<int> path) { this->blocks_attend_path[key] = path; }

    double getBlocksAttendCost(string key) { return this->blocks_attend_cost[key]; }

    vector<int> getBlocksAttendPath(string key) { return this->blocks_attend_path[key]; }
};

#endif // SCBRP_SHP_H
