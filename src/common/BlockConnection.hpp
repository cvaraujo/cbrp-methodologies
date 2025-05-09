//
// Created by Carlos on 20/04/2024.
//

#ifndef SCBRP_BC_H
#define SCBRP_BC_H

#include <utility>

#include "../classes/Arc.hpp"
#include "../classes/Graph.hpp"
#include "../classes/Parameters.hpp"
#include "ShortestPath.hpp"

class BlockConnection {

  private:
    Graph *graph;
    ShortestPath *sp;
    map<string, int> blocks_attend_cost;
    map<string, vector<int>> blocks_attend_path, best_order_attend_blocks;
    vector<vector<int>> block_2_block_cost;

  public:
    BlockConnection() = default;

    BlockConnection(Graph *graph, ShortestPath *sp) {
        this->graph = graph;
        this->sp = sp;
    };

    ~BlockConnection() {};

    int HeuristicBlockConnection(Graph *graph, ShortestPath *sp, vector<int> blocks, string key);

    set<int> computeBlock2BlockCost() {
        // Remove unecessary arcs between blocks
        int B = graph->getB();
        this->block_2_block_cost = vector<vector<int>>(B, vector<int>(B, INF));
        set<int> set_of_used_nodes, nodes_in_path, union_set;

        for (int b = 0; b < B - 1; b++) {
            for (int b2 = b + 1; b2 < B; b2++) {
                nodes_in_path = set<int>();
                int cost = this->sp->SHPBetweenBlocks(b, b2, nodes_in_path);

                if (cost < INF) {
                    union_set = set<int>();
                    set_union(nodes_in_path.begin(), nodes_in_path.end(), set_of_used_nodes.begin(), set_of_used_nodes.end(), inserter(union_set, union_set.begin()));
                    set_of_used_nodes = union_set;
                    this->block_2_block_cost[b][b2] = this->block_2_block_cost[b2][b] = cost;
                }
            }
        }

        return set_of_used_nodes;
    };

    static string GenerateStringFromIntVector(vector<int> blocks) {
        stringstream result;
        vector<int> keys(std::move(blocks));
        sort(keys.begin(), keys.end());
        copy(keys.begin(), keys.end(), ostream_iterator<int>(result, "_"));

        return result.str();
    };

    vector<vector<Arc>> createLayeredDag(vector<int> nodes, map<int, int> &dag_2_graph, int &V);

    vector<int> getBestOrderToAttendBlocks(vector<int> blocks);

    void setBlocksAttendCost(const string &key, int cost) { this->blocks_attend_cost[key] = cost; }

    void setBlocksAttendPath(const string &key, vector<int> path) { this->blocks_attend_path[key] = std::move(path); }

    int getBlocksAttendCost(const string &key) { return this->blocks_attend_cost[key]; }

    vector<int> getBlocksAttendPath(const string &key) { return this->blocks_attend_path[key]; }

    void setBlock2BlockCost(vector<vector<int>> block_2_block_cost) { this->block_2_block_cost = std::move(block_2_block_cost); }

    vector<vector<int>> getBlock2BlockCost() { return this->block_2_block_cost; }

    void setBestOrderToAttendBlocks(const string &key, vector<int> order) { this->best_order_attend_blocks[key] = std::move(order); }

    vector<int> getBestOrderToAttendBlocks(const string &key) { return this->best_order_attend_blocks[key]; }

    int getBlock2BlockCost(int i, int j) { return this->block_2_block_cost[i][j]; }

    bool keyExists(const string &key) { return this->blocks_attend_cost.find(key) != this->blocks_attend_cost.end(); }

    void updateBlock2BlockCost(int i, int j, int cost) { this->block_2_block_cost[i][j] = cost; }

    void UpdateBestKnowBlockConnection(vector<int> &blocks, vector<int> &path, int time) {
        string key = BlockConnection::GenerateStringFromIntVector(blocks);
        setBlocksAttendCost(key, time);
        setBlocksAttendPath(key, path);
    }

    void GetBestBlockOrderUsingHops(Input *input, vector<int> &blocks) {
        int N = graph->getN(), B = graph->getB();
        set<int> nodes_from_blocks;
        vector<int_pair> node_cumm_hops, block_cumm_hops;
        node_cumm_hops.reserve(N), block_cumm_hops.reserve(B);
        std::unordered_map<int, int> block_0_count;
        vector<int> curr_blocks = vector<int>(blocks.begin(), blocks.end());
        vector<bool> attended(B, false);

        // All Blocks from the route
        for (int b : blocks) {
            set<int> nodes = graph->getNodesFromBlock(b);
            nodes_from_blocks.insert(nodes.begin(), nodes.end());
            attended[b] = true;
        }

        vector<int> route;
        route.reserve(B), route.push_back(graph->getSink());

        vector<int> blocks_cumm_hops(B, 0);
        for (int node : nodes_from_blocks) {
            block_0_count[node] = 0;
            int node_hops = 0;

            for (int b : blocks) {
                int hops = graph->getNodeBlockHops(node, b);
                if (hops != -1) {
                    node_hops += hops;
                    blocks_cumm_hops[b] += hops;
                    if (hops == 0)
                        block_0_count[node]++;
                }
            }
            node_cumm_hops.emplace_back(node, node_hops);
        }

        for (int b = 0; b < B; b++) {
            if (blocks_cumm_hops[b] > 0)
                block_cumm_hops.emplace_back(b, blocks_cumm_hops[b]);
        }

        // Sort the nodes by cummulative hops
        sort(node_cumm_hops.begin(), node_cumm_hops.end(), [](const int_pair &a, const int_pair &b) {
            return a.second < b.second;
        });
        sort(block_cumm_hops.begin(), block_cumm_hops.end(), [](const int_pair &a, const int_pair &b) {
            return a.second < b.second;
        });

        for (int i = 0; i < block_cumm_hops.size() - 1; i++) {
            int_pair pair = block_cumm_hops[i];
            int node = pair.first, next_node = block_cumm_hops[i + 1].first;
            if (block_0_count[node] < block_0_count[next_node]) {
                route.push_back(node);
                for (int blc : graph->getNode(node).second) {
                    if (attended[blc]) {
                        curr_blocks.erase(remove(curr_blocks.begin(), curr_blocks.end(), blc), curr_blocks.end());
                        attended[blc] = false;
                    }
                }
                break;
            }
        }

        int last_used_node = route.back();
        int total_time = 0;
        for (auto pr : block_cumm_hops) {
            int block = pr.first;
            if (attended[block]) {
                int next_node = -1, best_time = INF, cumm_hops = INF;
                for (int node : graph->getNodesFromBlock(block)) {
                    int node_time = input->getArcTime(last_used_node, node);
                    if (node_time < best_time || (node_time <= best_time && block_0_count[node] << cumm_hops)) {
                        best_time = node_time, next_node = node;
                    }
                }
                route.push_back(next_node);
                total_time += best_time;
                last_used_node = next_node;
                for (int blc : graph->getNode(next_node).second) {
                    if (attended[blc]) {
                        curr_blocks.erase(remove(curr_blocks.begin(), curr_blocks.end(), blc), curr_blocks.end());
                        attended[blc] = false;
                    }
                }
            }
        }
        route.push_back(graph->getSink());
    }
};

#endif // SCBRP_SHP_H
