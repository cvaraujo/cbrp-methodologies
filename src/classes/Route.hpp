//
// Created by carlos on 04/01/25.
//

#ifndef DPARP_STOCHASTIC_ROUTE_H
#define DPARP_STOCHASTIC_ROUTE_H

#include "Parameters.hpp"
#include "Input.hpp"
#include "Graph.hpp"

class Route
{

private:
    Input *input;
    vector<int> route, preds;
    vector<bool> blocks_attended;
    vector<int> used_node_to_attend_block;
    set<int> route_blocks;
    map<int, vector<int>> blocks_attendeds_per_node;
    int time_blocks = 0, time_route = 0;
    vector<pair<int, int>> x; // Maybe not needed
    vector<int> sequence_of_attended_blocks;

public:
    Route(Input *input, vector<pair<int, int>> arcs, vector<int> blocks)
    {
        this->input = input;
        this->x = arcs;
        this->PopulateRouteDataStructures(arcs);
        this->PopulateBlocksDataStructures(blocks);
    };

    Route(Input *input, vector<int> y)
    {
        this->input = input;
        this->sequence_of_attended_blocks = y;
        this->PopulateDataStructures();
    };

    Route(Input *input, vector<pair<int, int>> arcs)
    {
        this->input = input;
        this->x = arcs;
        this->PopulateRouteDataStructures(arcs);
    };

    // Destructor
    ~Route()
    {
        route.clear();
        preds.clear();
        blocks_attended.clear();
        used_node_to_attend_block.clear();
        route_blocks.clear();
        blocks_attendeds_per_node.clear();
        x.clear();
        sequence_of_attended_blocks.clear();
    };

    Route(Input *input) { this->input = input; };

    void setSequenceOfAttendingBlocks(vector<int> sequence_of_attended_blocks) { this->sequence_of_attended_blocks = sequence_of_attended_blocks; };

    void setRoute(vector<int> route) { this->route = route; };

    void setTimeRoute(int time_route) { this->time_route = time_route; };

    void setTimeBlocks(int time_blocks) { this->time_blocks = time_blocks; };

    vector<int> getSequenceOfAttendingBlocks() { return this->sequence_of_attended_blocks; };

    void PopulateDataStructures()
    {
        Graph *graph = input->getGraph();
        BlockConnection *bc = input->getBlockConnection();

        string key = bc->GenerateStringFromIntVector(this->sequence_of_attended_blocks);
        this->route = input->getBlockConnectionRoute(key);
        this->time_route = input->getBlockConnectionTime(key);
        this->sequence_of_attended_blocks = input->getBestOrderToAttendBlocks(key);

        int B = graph->getB();
        preds = vector<int>(graph->getN() + 1, -1);
        route_blocks = set<int>();

        // Attended blocks
        blocks_attended = vector<bool>(B, false);
        this->time_blocks = 0;
        for (int b : this->sequence_of_attended_blocks)
        {
            time_blocks += graph->getTimePerBlock(b);
            blocks_attended[b] = true;
        }

        // Route starts and ends at node N
        used_node_to_attend_block = vector<int>(B, -1);
        blocks_attendeds_per_node = map<int, vector<int>>();
        set<int> node_blocks;
        int i, origin, destination;

        for (i = 1; i < this->route.size(); i++)
        {
            origin = route[i - 1], destination = route[i];
            node_blocks = graph->getNode(destination).second;

            // get preds
            this->preds[destination] = origin;
            // All blocks of the route
            this->route_blocks.insert(node_blocks.begin(), node_blocks.end());

            for (int b : node_blocks)
            {
                if (blocks_attended[b] && used_node_to_attend_block[b] == -1)
                {
                    if (blocks_attendeds_per_node.find(destination) == blocks_attendeds_per_node.end())
                        blocks_attendeds_per_node[destination] = vector<int>();
                    blocks_attendeds_per_node[destination].push_back(b);
                    used_node_to_attend_block[b] = destination;
                }
            }
        }
    };

    void PopulateRouteDataStructures(vector<pair<int, int>> arcs)
    {
        Graph *graph = this->input->getGraph();
        preds = vector<int>(graph->getN() + 1, -1);
        route_blocks = set<int>();

        for (auto arc : arcs)
        {
            int i = arc.first, j = arc.second;
            this->route[i] = j, this->preds[j] = i;
            Arc *g_arc = graph->getArc(i, j);

            if (g_arc == nullptr)
            {
                cout << "i: " << i << ", j: " << j << endl;
                throw std::runtime_error("[!!!] Invalid arc while populate route data structures!");
            }

            this->time_route += g_arc->getLength();
            set<int> blocks_i = graph->getNode(i).second, blocks_j = graph->getNode(j).second;
            route_blocks.insert(blocks_i.begin(), blocks_i.end());
            route_blocks.insert(blocks_j.begin(), blocks_j.end());
        }
    };

    void PopulateBlocksDataStructures(vector<int> blocks)
    {
        Graph *graph = this->input->getGraph();
        this->blocks_attended = vector<bool>(graph->getB(), false);

        for (auto b : blocks)
        {
            this->blocks_attended[b] = true;
            time_blocks += graph->getTimePerBlock(b);

            set<int> nodes = graph->getNodesFromBlock(b);
            for (auto node : nodes)
            {
                if (preds[node] != -1)
                {
                    // if (this->att_blocks_per_node.find(node) == this->att_blocks_per_node.end())
                    //     this->att_blocks_per_node[node] = vector<int>();

                    // this->att_blocks_per_node[node].push_back(b);
                    // this->used_node_attended_block[b] = node;
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
        Graph *graph = this->input->getGraph();
        if (!this->blocks_attended[b])
            throw std::runtime_error("[!!!] Block " + to_string(b) + " not attended to be swapped!");

        // Remove references of b
        this->blocks_attended[b] = false;
        auto it = find(sequence_of_attended_blocks.begin(), sequence_of_attended_blocks.end(), b);
        if (it != sequence_of_attended_blocks.end())
            sequence_of_attended_blocks.erase(it);

        int node_b = this->used_node_to_attend_block[b];
        this->used_node_to_attend_block[b] = -1;
        this->blocks_attendeds_per_node[node_b].erase(find(blocks_attendeds_per_node[node_b].begin(), blocks_attendeds_per_node[node_b].end(), b));
        this->time_blocks -= graph->getTimePerBlock(b);
    };

    bool CanAlocateRemainingBlocksIntoOtherNodes(Graph *graph, int removed_b, int node)
    {
        vector<int> blocks = this->blocks_attendeds_per_node[node];
        for (auto b : blocks)
        {
            if (b == removed_b)
                continue;

            set<int> nodes_b = graph->getNodesFromBlock(b);
            bool can_alocate = false;

            for (auto node_b : nodes_b)
            {
                if (node_b == node)
                    continue;

                if (this->preds[node_b] != -1)
                {
                    can_alocate = true;
                    break;
                }
            }

            if (!can_alocate)
                return false;
        }
        return true;
    };

    void RemoveNodeFromRoute(int node)
    {
        preds[node] = -1;
        auto &it = find(route.begin(), route.end(), node);

        if (it != route.end())
        {
            *it = route.end();
            route.pop_back();
        }

        for (auto block : this->blocks_attendeds_per_node[node])
        {
            this->blocks_attended[block] = false;
            this->used_node_to_attend_block[block] = -1;
            this->route_blocks.erase(block);
        }
        this->blocks_attendeds_per_node.erase(node);
    };

    double RemoveBlockFromRoute(int b)
    {
        if (!this->blocks_attended[b] && !this->isBlockInRoute(b))
            throw std::runtime_error("[!] Block " + to_string(b) + " not in route to be removed!");

        Graph *graph = this->input->getGraph();
        BlockConnection *bc = this->input->getBlockConnection();

        int i, block;
        for (i = 0; i < this->sequence_of_attended_blocks.size(); i++)
        {
            block = this->sequence_of_attended_blocks[i];
            if (block != b)
                continue;

            int node = this->used_node_to_attend_block[block];
            vector<int> blocks_attended = this->blocks_attendeds_per_node[node];

            // Still have blocks attended in this node
            if (blocks_attended.size() - 1 > 0)
            {
                if (this->CanAlocateRemainingBlocksIntoOtherNodes(graph, block, node))
                {
                    RemoveNodeFromRoute(node);
                }

                // if (this->CanAlocateRemainingBlocksIntoOtherNodes(graph, block, node))
                // {
                // }
                if (this->blocks_attended[b])
                    this->RemoveBlockFromAttended(b);
                return 0.0;
            }
            else
            {
                // No more blocks attended in this node
            }
        }
    }

    void AddBlockToAttended(int b)
    {
        // Basic checks
        Graph *graph = this->input->getGraph();
        if (this->blocks_attended[b])
            throw std::runtime_error("[!!!] Block " + to_string(b) + " already attended!");

        set<int> nodes_b = graph->getNodesFromBlock(b);
        for (auto node : nodes_b)
        {
            if (preds[node] != -1)
            {
                if (blocks_attendeds_per_node.find(node) == blocks_attendeds_per_node.end())
                    blocks_attendeds_per_node[node] = vector<int>();

                this->blocks_attendeds_per_node[node].push_back(b);
                this->used_node_to_attend_block[b] = node;
                break;
            }
        }

        this->blocks_attended[b] = true;
        this->sequence_of_attended_blocks.push_back(b);
        this->time_blocks += graph->getTimePerBlock(b);
    };

    bool isBlockInRoute(int b)
    {
        return this->route_blocks.find(b) != this->route_blocks.end();
    };

    bool isBlockAttended(int b)
    {
        return this->blocks_attended[b];
    };

    bool isSwapFeasible(int b1, int b2)
    {
        Graph *graph = this->input->getGraph();
        // if (!this->blocks_attended[b1] || this->blocks_attended[b2] || this->used_node_attended_block.find(b1) == this->used_node_attended_block.end())
        //     return false;

        if (this->time_blocks - graph->getTimePerBlock(b1) + graph->getTimePerBlock(b2) > this->input->getT())
            return false;

        return true;
    };

    set<int> getBlocks() { return this->route_blocks; };

    bool isSwapTimeLowerThanT(int b1, int b2)
    {
        Graph *graph = this->input->getGraph();
        return (this->time_route + this->time_blocks) + (graph->getTimePerBlock(b2) - graph->getTimePerBlock(b1)) <= input->getT();
    };
};
#endif
