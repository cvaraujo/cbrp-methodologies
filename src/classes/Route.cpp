//
// Created by carlos on 04/01/25.
//

#include "Route.hpp"

Route::Route(Input *input, vector<pair<int, int>> arcs, vector<int> blocks)
{
    this->input = input;
    this->x = arcs;
    this->PopulateRouteDataStructures(arcs);
    this->PopulateBlocksDataStructures(blocks);
}

void Route::PopulateDataStructures()
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
    this->blocks_attendeds_per_node = std::unordered_map<int, vector<int>>();
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
            if (blocks_attended[b] && this->used_node_to_attend_block[b] == -1)
            {
                if (this->blocks_attendeds_per_node.find(destination) == this->blocks_attendeds_per_node.end())
                    this->blocks_attendeds_per_node[destination] = vector<int>();
                this->blocks_attendeds_per_node[destination].push_back(b);
                this->used_node_to_attend_block[b] = destination;
            }
        }
    }
}

void Route::PopulateRouteDataStructures(vector<pair<int, int>> arcs)
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
}

void Route::PopulateBlocksDataStructures(vector<int> blocks)
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
}

void Route::SwapInRouteBlocks(int b1, int b2)
{
    // Basic checks
    if (!this->blocks_attended[b1])
        throw std::runtime_error("[!!!] Block " + to_string(b1) + " not attended to be swapped!");

    if (this->blocks_attended[b2])
        throw std::runtime_error("[!!!] Block " + to_string(b2) + " already attended!");

    this->RemoveBlockFromAttended(b1);
    this->AddBlockToAttended(b2);
}

void Route::GeneralSwapBlocks(int b1, int b2)
{
    bool b1_in_route = this->IsBlockInRoute(b1), b2_in_route = this->IsBlockInRoute(b2);
    bool b1_attended = this->IsBlockAttended(b1), b2_attended = this->IsBlockAttended(b2);

    if ((b1_attended && b2_attended) || (!b1_in_route && !b2_in_route))
    {
        cout << "[!] Both blocks are in same situation, the swap is not feasible!" << endl
             << "(" << b1_attended << ", " << b2_attended << ")" << endl;
        return;
    }

    // Swap in route
    if ((b1_attended && b2_in_route) || (b1_in_route && b2_attended))
    {
        int to_remove = b1_attended ? b1 : b2;
        int to_insert = b2_attended ? b1 : b2;
        this->SwapInRouteBlocks(to_remove, to_insert);
    }
    else if (b1_in_route && !b2_in_route)
    {
    }
}

void Route::RemoveBlockFromAttended(int b)
{
    // Basic checks
    if (!this->blocks_attended[b])
        return;

    Graph *graph = this->input->getGraph();
    // Remove references of b
    this->blocks_attended[b] = false;
    auto it = find(sequence_of_attended_blocks.begin(), sequence_of_attended_blocks.end(), b);
    if (it != sequence_of_attended_blocks.end())
        sequence_of_attended_blocks.erase(it);

    int node_b = this->used_node_to_attend_block[b];
    this->used_node_to_attend_block[b] = -1;
    vector<int> &attended_blocks = this->blocks_attendeds_per_node[node_b];
    Utils::IntVectorRemove(attended_blocks, find(attended_blocks.begin(), attended_blocks.end(), b));
    this->blocks_attendeds_per_node[node_b] = attended_blocks;
    this->time_blocks -= graph->getTimePerBlock(b);
}

bool Route::CanAlocateRemainingBlocksIntoOtherNodes(Graph *graph, int removed_b, int node, std::unordered_map<int, int> &new_node_to_block)
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
                new_node_to_block[node_b] = b;
                break;
            }
        }

        if (!can_alocate)
            return false;
    }
    return true;
};

void Route::RemoveNodeFromRoute(int node)
{
    int i, curr_node, prev_node, next_node;
    for (i = 0; i < route.size(); i++)
    {
        curr_node = route[i];
        if (curr_node == node)
        {
            prev_node = route[i - 1], next_node = route[i + 1];
            this->time_route -= this->input->getArcTime(prev_node, curr_node) + this->input->getArcTime(curr_node, next_node);
            this->time_route += this->input->getArcTime(prev_node, next_node);
            route.erase(route.begin() + i);
            break;
        }
    }

    Graph *graph = this->input->getGraph();
    preds[node] = -1;
    for (auto block : this->blocks_attendeds_per_node[node])
    {
        this->blocks_attended[block] = false;
        this->used_node_to_attend_block[block] = -1;
        this->time_blocks -= input->getBlockTime(block);
    }
    this->blocks_attendeds_per_node.erase(node);

    // Reset Route_blocks
    route_blocks.clear();
    for (int route_node : route)
    {
        set<int> blocks = graph->getNode(route_node).second;
        route_blocks.insert(blocks.begin(), blocks.end());
    }
};

void Route::ChangeNodeAttendingBlock(int block, int old_node, int new_node)
{
    this->used_node_to_attend_block[block] = new_node;

    if (this->blocks_attendeds_per_node.find(new_node) == this->blocks_attendeds_per_node.end())
        this->blocks_attendeds_per_node[new_node] = vector<int>();
    this->blocks_attendeds_per_node[new_node].push_back(block);

    vector<int> &attended_blocks = this->blocks_attendeds_per_node[old_node];
    Utils::IntVectorRemove(attended_blocks, find(attended_blocks.begin(), attended_blocks.end(), block));
    this->blocks_attendeds_per_node[old_node] = attended_blocks;
};

void Route::RemoveBlockAndNodeIfPossible(Graph *graph, int block, int node)
{
    std::unordered_map<int, int> new_node_to_block = GetAttendedRealocatedBlocks(graph, node);
    int num_blocks_to_realocate = this->blocks_attendeds_per_node[node].size() - 1;

    if (new_node_to_block.size() >= num_blocks_to_realocate)
    {
        for (auto &[new_node, realoc_block] : new_node_to_block)
        {
            if (realoc_block == block)
                continue;

            this->ChangeNodeAttendingBlock(block, realoc_block, new_node);
        }
    }

    this->RemoveBlockFromAttended(block);
    if (this->blocks_attendeds_per_node[node].size() <= 0)
        this->RemoveNodeFromRoute(node);
}

std::unordered_map<int, int> Route::GetAttendedRealocatedBlocks(Graph *graph, int node)
{
    std::unordered_map<int, int> new_node_to_block = std::unordered_map<int, int>();
    vector<int> blocks = this->blocks_attendeds_per_node[node];

    for (auto b : blocks)
    {
        for (auto node_b : graph->getNodesFromBlock(b))
        {
            if (node_b == node)
                continue;

            if (this->preds[node_b] != -1)
            {
                new_node_to_block[node_b] = b;
                break;
            }
        }
    }
    return new_node_to_block;
}

int Route::EvaluateTimeChangeByRemovingNodeAndBlocks(int node_idx)
{
    int curr_time = this->time_route + this->time_blocks;
    int new_time = curr_time;
    int i, curr_node, prev_node, next_node;

    curr_node = route[node_idx], prev_node = route[i - 1], next_node = route[i + 1];
    new_time += input->getArcTime(prev_node, next_node) - (input->getArcTime(prev_node, curr_node) + input->getArcTime(curr_node, next_node));

    for (auto block : blocks_attendeds_per_node[curr_node])
        new_time -= input->getBlockTime(block);

    return new_time - curr_time;
}

int Route::EvaluateTimeChangeByRemovingNodeAndReallocateBlocks(int node_idx)
{
    Graph *graph = this->input->getGraph();
    int curr_time = this->time_route + this->time_blocks, new_time = curr_time;
    int i, curr_node, prev_node, next_node;

    curr_node = route[node_idx], prev_node = route[i - 1], next_node = route[i + 1];
    new_time += input->getArcTime(prev_node, next_node) - (input->getArcTime(prev_node, curr_node) + input->getArcTime(curr_node, next_node));

    std::unordered_map<int, int> new_node_to_block = this->GetAttendedRealocatedBlocks(graph, curr_node);

    for (auto b : this->blocks_attendeds_per_node[curr_node])
    {
        if (new_node_to_block.find(b) != new_node_to_block.end())
            continue;
        new_time -= graph->getTimePerBlock(b);
    }
    return new_time - curr_time;
}

void Route::RemoveBlockFromRoute(int b)
{
    if (!this->IsBlockInRoute(b))
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
        this->RemoveBlockAndNodeIfPossible(graph, block, node);
        break;
    }
}

int_pair Route::EvaluateBlockInsertion(int previous_node, int next_node, int new_block)
{
    Graph *graph = this->input->getGraph();
    set<int> nodes_from_block = graph->getNodesFromBlock(new_block);

    int insert_time, best_time = INT_MAX, best_node = -1;
    int arc_removed_time = (previous_node != -1 && next_node != -1) ? input->getArcTime(previous_node, next_node) : 0;

    for (auto node : nodes_from_block)
    {
        insert_time = (arc_removed_time != 0) ? -arc_removed_time : 0;

        insert_time += previous_node != -1 ? input->getArcTime(previous_node, node) : 0;
        insert_time += next_node != -1 ? input->getArcTime(node, next_node) : 0;

        if (insert_time < best_time)
            best_time = insert_time, best_node = node;
    }
    return make_pair(best_node, best_time);
}

int_pair Route::FindBestPositionToInsertBlock(int new_block)
{
    Graph *graph = this->input->getGraph();
    int best_time_insert = INF, best_node_insert = -1, best_position = -1;

    int_pair insert_pair;
    for (int i = 0; i < this->route.size() - 1; i++)
    {
        int prev_node = this->route[i], next_node = this->route[i + 1];
        insert_pair = this->EvaluateBlockInsertion(prev_node, next_node, new_block);
        int time_insert = insert_pair.first, node_insert = insert_pair.second;

        if (time_insert < best_time_insert)
        {
            best_time_insert = time_insert;
            best_node_insert = node_insert;
            best_position = i + 1;
        }
    }

    return make_pair(best_node_insert, best_position);
}

void Route::AddBlockToRoute(int b, bool in_best_order)
{
    if (this->IsBlockInRoute(b))
        throw std::runtime_error("[!!!] Block " + to_string(b) + " already in route!");

    int_pair pos_insert_block = this->FindBestPositionToInsertBlock(b);
    int used_node = pos_insert_block.first, position = pos_insert_block.second;

    if (used_node == -1)
        return;

    int previous_node = this->route[position - 1], next_node = this->route[position];
    int change_route_time = this->input->getArcTime(previous_node, used_node) +
                            this->input->getArcTime(used_node, next_node) -
                            this->input->getArcTime(previous_node, next_node);

    this->route.insert(this->route.begin() + position, used_node);
    this->preds[used_node] = previous_node, this->preds[next_node] = used_node;
    this->time_route += change_route_time;
}

void Route::AddBlockToAttended(int b)
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
            if (this->blocks_attendeds_per_node.find(node) == this->blocks_attendeds_per_node.end())
                this->blocks_attendeds_per_node[node] = vector<int>();

            this->blocks_attendeds_per_node[node].push_back(b);
            this->used_node_to_attend_block[b] = node;
            break;
        }
    }

    this->blocks_attended[b] = true;
    this->sequence_of_attended_blocks.push_back(b);
    this->time_blocks += graph->getTimePerBlock(b);
};

bool Route::IsSwapFeasible(int b1, int b2)
{
    Graph *graph = this->input->getGraph();

    if (this->time_blocks - graph->getTimePerBlock(b1) + graph->getTimePerBlock(b2) > this->input->getT())
        return false;

    return true;
};

bool Route::IsSwapTimeLowerThanT(int b1, int b2)
{
    Graph *graph = this->input->getGraph();
    return (this->time_route + this->time_blocks) + (graph->getTimePerBlock(b2) - graph->getTimePerBlock(b1)) <= input->getT();
}