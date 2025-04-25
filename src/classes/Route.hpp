//
// Created by carlos on 04/01/25.
//

#ifndef DPARP_STOCHASTIC_ROUTE_H
#define DPARP_STOCHASTIC_ROUTE_H

#include "Parameters.hpp"
#include "Input.hpp"
#include "Graph.hpp"
#include "../heuristic/stochastic/Utils.hpp"
#include <unordered_map>

class Route
{

private:
    Input *input;
    int time_blocks = 0, time_route = 0;
    vector<int> route, preds, used_node_to_attend_block, sequence_of_attended_blocks;
    vector<bool> blocks_attended;
    set<int> route_blocks;
    std::unordered_map<int, vector<int>> blocks_attendeds_per_node;
    vector<pair<int, int>> x; // Maybe not needed

    bool CanAlocateRemainingBlocksIntoOtherNodes(Graph *graph, int removed_b, int node, std::unordered_map<int, int> &new_node_to_block);

    void ChangeNodeAttendingBlock(int block, int old_node, int new_node);

    void RemoveBlockAndNodeIfPossible(Graph *graph, int block, int node);

    std::unordered_map<int, int> GetAttendedRealocatedBlocks(Graph *graph, int node);

public:
    Route(Input *input, vector<pair<int, int>> arcs, vector<int> blocks);

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

    void GeneralSwapBlocks(int b1, int b2);

    Route(Input *input) { this->input = input; };

    void setSequenceOfAttendingBlocks(vector<int> sequence_of_attended_blocks) { this->sequence_of_attended_blocks = sequence_of_attended_blocks; };

    void setRoute(vector<int> route) { this->route = route; };

    vector<int> getRoute() { return this->route; };

    set<int> getRouteBlocks() { return this->route_blocks; };

    int getTimeRoute() { return this->time_route; };

    int getTimeAttBlocks() { return this->time_blocks; };

    void setTimeRoute(int time_route) { this->time_route = time_route; };

    void setTimeBlocks(int time_blocks) { this->time_blocks = time_blocks; };

    vector<int> getSequenceOfAttendingBlocks() { return this->sequence_of_attended_blocks; };

    void PopulateDataStructures();

    void PopulateRouteDataStructures(vector<pair<int, int>> arcs);

    void PopulateBlocksDataStructures(vector<int> blocks);

    void SwapInRouteBlocks(int b1, int b2);

    void RemoveBlockFromAttended(int b);

    void RemoveNodeFromRoute(int node);

    int EvaluateTimeChangeByRemovingNodeAndBlocks(int node_idx);

    int EvaluateTimeChangeByRemovingNodeAndReallocateBlocks(int node_idx);

    void RemoveBlockFromRoute(int b);

    void AddBlockToRoute(int b, bool in_best_order);

    int_pair EvaluateBlockInsertion(int previous_block, int next_block, int new_block);

    int_pair FindBestPositionToInsertBlock(int new_block);

    void AddBlockToAttended(int b);

    bool IsBlockInRoute(int b)
    {
        if (this->blocks_attended[b] || this->route_blocks.find(b) != this->route_blocks.end())
            return true;
        return false;
    };

    bool IsBlockInsertionFactible(int block, int route_time_increase)
    {
        int total_time = this->time_route + route_time_increase + this->input->getBlockTime(block);
        if (total_time > this->input->getT())
            return false;
        return true;
    };

    void InsertNodeInRoute(int node, int position)
    {
        int previous_node = this->route[position - 1], next_node = this->route[position];
        int change_route_time = this->input->getArcTime(previous_node, node) +
                                this->input->getArcTime(node, next_node) -
                                this->input->getArcTime(previous_node, next_node);

        this->route.insert(this->route.begin() + position, node);
        this->preds[node] = previous_node, this->preds[next_node] = node;
        this->time_route += change_route_time;

        set<int> node_blocks = this->input->getGraph()->getNode(node).second;
        this->route_blocks.insert(node_blocks.begin(), node_blocks.end());
    };

    bool IsTimeChangeFeasible(int time_change)
    {
        if (this->time_route + this->time_blocks + time_change > this->input->getT())
            return false;
        return true;
    };

    bool IsBlockAttended(int b) { return this->blocks_attended[b]; };

    bool IsSwapFeasible(int b1, int b2);

    bool IsOutSwapFeasible(int b1, int b2);

    set<int> getBlocks() { return this->route_blocks; };

    bool IsSwapTimeLowerThanT(int b1, int b2);
};
#endif
