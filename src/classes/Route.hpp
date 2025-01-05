//
// Created by carlos on 04/01/25.
//

#ifndef DPARP_STOCHASTIC_ROUTE_H
#define DPARP_STOCHASTIC_ROUTE_H

#include "Parameters.hpp"
#include "Graph.hpp"

class Route
{
private:
    Graph *graph;
    map<int, int> route, used_node_attended_block;
    map<int, vector<int>> att_blocks_per_node;
    vector<int> preds, route_blocks;
    vector<bool> blocks_attended;
    int time_blocks = 0, time_route = 0;
    vector<pair<int, int>> x;

public:
    Route(Graph *graph, vector<pair<int, int>> arcs, vector<int> blocks)
    {
        this->graph = graph;
        this->x = arcs;
        this->PopulateRouteDataStructures(arcs);
        this->PopulateBlocksDataStructures(blocks);
    };

    void PopulateRouteDataStructures(vector<pair<int, int>> arcs)
    {
        preds = vector<int>(graph->getN() + 1, -1);

        for (auto arc : arcs)
        {
            int i = arc.first, j = arc.second;
            this->route[i] = j, this->preds[j] = i;
            Arc *g_arc = this->graph->getArc(i, j);

            if (g_arc == nullptr)
                throw std::runtime_error("[!!!] Invalid arc while populate route data structures!");

            this->time_route += g_arc->getLength();
        }
    };

    void PopulateBlocksDataStructures(vector<int> blocks)
    {
        this->route_blocks = blocks;
        this->blocks_attended = vector<bool>(graph->getB(), false);

        for (auto b : blocks)
        {
            this->blocks_attended[b] = true;
            time_blocks += this->graph->getTimePerBlock(b);

            set<int> nodes = this->graph->getNodesFromBlock(b);
            for (auto node : nodes)
            {
                if (preds[node] != -1)
                {
                    if (this->att_blocks_per_node.find(node) == this->att_blocks_per_node.end())
                        this->att_blocks_per_node[node] = vector<int>();

                    this->att_blocks_per_node[node].push_back(b);
                    this->used_node_attended_block[b] = node;
                    break;
                }
            }
        }
    };

    void SwapBlocks(int b1, int b2)
    {
        // Basic checks
        if (!this->blocks_attended[b1])
            throw std::runtime_error("[!!!] Block " + to_string(b1) + " not attended to be swapped!");

        if (this->blocks_attended[b2])
            throw std::runtime_error("[!!!] Block " + to_string(b2) + " already attended!");

        this->RemoveBlockFromAttended(b1);
        this->AddBlockToAttended(b2);
    };

    void RemoveBlockFromAttended(int b)
    {
        // Basic checks
        if (!this->blocks_attended[b])
            throw std::runtime_error("[!!!] Block " + to_string(b) + " not attended to be swapped!");

        // Remove references of b
        int node_b = this->used_node_attended_block[b];
        used_node_attended_block.erase(b);
        blocks_attended[b] = false;
        att_blocks_per_node[node_b].erase(find(att_blocks_per_node[node_b].begin(), att_blocks_per_node[node_b].end(), b));
        this->time_blocks -= graph->getTimePerBlock(b);
    };

    void AddBlockToAttended(int b)
    {
        if (this->blocks_attended[b])
            throw std::runtime_error("[!!!] Block " + to_string(b) + " already attended!");

        set<int> nodes_b = this->graph->getNodesFromBlock(b);
        for (auto node : nodes_b)
        {
            if (preds[node] != -1)
            {
                if (att_blocks_per_node.find(node) == att_blocks_per_node.end())
                    att_blocks_per_node[node] = vector<int>();

                this->att_blocks_per_node[node].push_back(b);
                this->used_node_attended_block[b] = node;
                break;
            }
        }

        blocks_attended[b] = true;
        this->time_blocks += graph->getTimePerBlock(b);
    };
};
#endif
