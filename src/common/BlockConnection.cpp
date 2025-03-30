
#include "BlockConnection.hpp"

int BlockConnection::HeuristicBlockConnection(
    Graph *graph,
    ShortestPath *sp,
    vector<int> blocks,
    string key)
{
    map<int, int> dag_2_graph;
    int N = graph->getN();
    vector<set<int>> nodes_per_block = graph->getNodesPerBlock();

    // Route with only one block
    if (blocks.size() == 1)
    {
        this->setBlocksAttendCost(key, 0);
        for (auto i : nodes_per_block[blocks[0]])
        {
            this->setBlocksAttendPath(key, vector<int>{N, i, N});
            break;
        }
        return 0;
    }

    // Heuristic to define the sequence of attending blocks
    blocks = this->getBestOrderToAttendBlocks(blocks);

    // Create the DAG
    int V;
    vector<vector<Arc>> dag = this->createLayeredDag(blocks, dag_2_graph, V);

    // cout << "Create DAG" << endl;

    // SHP on DAG
    vector<int> pred;
    vector<int> path = vector<int>();
    int cost = ShortestPath::DijkstraLayeredDAG(dag, V + 2, V, V + 1, pred);

    // Get the path
    int v = V + 1, last_inserted = -1;

    while (v != pred[v])
    {
        if (dag_2_graph[v] != last_inserted && last_inserted != -1)
        {
            path.push_back(dag_2_graph[v]);
            last_inserted = dag_2_graph[v];
        }

        v = pred[v];

        if (v == -1)
            return INF;
    }

    path.push_back(dag_2_graph[V]);

    this->setBlocksAttendPath(key, path);
    this->setBlocksAttendCost(key, cost);

    return cost;
}

vector<int> BlockConnection::getBestOrderToAttendBlocks(vector<int> blocks)
{
    vector<int> connect_order = vector<int>();
    vector<int> backup_blocks = blocks;
    int route_cost, num_destination_nodes, num_best_block_nodes;

    Graph *graph = this->graph;
    sort(backup_blocks.begin(), backup_blocks.end(), [graph](int a, int b)
         { return graph->getNodesFromBlock(a).size() > graph->getNodesFromBlock(b).size(); });

    while (connect_order.size() < blocks.size())
    {
        if (connect_order.empty())
        {
            connect_order.push_back(backup_blocks[0]);
            backup_blocks.erase(backup_blocks.begin());
        }

        int best_block = -1, shp = INF;
        num_best_block_nodes = 0;
        for (int i = 0; i < int(backup_blocks.size()); i++)
        {
            route_cost = this->block_2_block_cost[connect_order.back()][backup_blocks[i]];
            num_destination_nodes = graph->getNodesFromBlock(backup_blocks[i]).size();

            if (route_cost < shp || (route_cost == shp && num_destination_nodes > num_best_block_nodes))
            {
                shp = this->block_2_block_cost[connect_order.back()][backup_blocks[i]];
                best_block = i, num_best_block_nodes = graph->getNodesFromBlock(backup_blocks[i]).size();
            }
        }

        if (best_block != -1)
            connect_order.push_back(backup_blocks[best_block]), backup_blocks.erase(backup_blocks.begin() + best_block);
    }

    return connect_order;
}

int BlockConnection::SimplePathBlockConnection(vector<int> blocks, vector<int> &pred)
{
    int cost = 0;
    set<int> not_attended_blocks = set<int>(blocks.begin() + 2, blocks.end());
    vector<bool> attended_block = vector<bool>(graph->getB(), false);
    pred = vector<int>(graph->getN() + 1, -1);
    int last_block = blocks[0], next_block = blocks[1], last_node = -1;
    map<pair<int, int>, bool> temp_blocked_arc_in_route = map<pair<int, int>, bool>();
    map<int, set<int>> temp_blocks_attended_in_node = map<int, set<int>>();

    while (!not_attended_blocks.empty())
    {
        int block_l1 = last_block, block_l2 = next_block;

        set<int> l1_nodes;
        if (last_node != -1)
            l1_nodes = set<int>{last_node};
        else
            l1_nodes = graph->getNodesFromBlock(block_l1);

        set<int> l2_nodes = graph->getNodesFromBlock(block_l2);

        cout << "Connection between blocks " << block_l1 << " and " << block_l2 << endl;
        cout << "Last Node: " << last_node << endl;
        cout << "Last Block: " << last_block << endl;
        cout << "Next Block: " << next_block << endl;
        cout << "Nodes in L1: ";
        for (auto n : l1_nodes)
            cout << n << ", ";
        cout << endl;
        cout << "Nodes in L2: ";
        for (auto n : l2_nodes)
            cout << n << ", ";
        cout << endl;

        pair<int, int> lowest_arc, best_route_option;
        int lowest_cost = INF, lowest_route_option_cost = INF;

        // Try to find a direct arc
        for (int n1 : l1_nodes)
        {
            if (n1 != last_node && pred[n1] != -1)
                continue;

            for (int n2 : l2_nodes)
            {
                if (pred[n2] != -1)
                    continue;

                auto pair_n1_n2 = make_pair(n1, n2);
                if (temp_blocked_arc_in_route.find(pair_n1_n2) != temp_blocked_arc_in_route.end())
                    continue;

                if (n1 == n2)
                {
                    lowest_arc = make_pair(n1, n2);
                    lowest_cost = 0;
                    break;
                }

                Arc *arc = graph->getArc(n1, n2);

                if (graph->getArc(n1, n2) != nullptr)
                {
                    // cout << "[*] Found arc " << n1 << " -> " << n2 << endl;
                    if (arc->getLength() < lowest_cost)
                    {
                        lowest_cost = arc->getLength();
                        lowest_arc = make_pair(n1, n2);
                    }
                    // getchar();
                }
                else
                {
                    // Try to find a route
                    // cout << "[*] No arc " << n1 << " -> " << n2 << endl;

                    vector<Arc *> n1_arcs = graph->getArcs(n1);
                    // cout << "[*] Trying arcs from n1: " << n1_arcs.size() << endl;
                    for (auto n1_arc : n1_arcs)
                    {
                        int n3 = n1_arc->getD();

                        if (n3 >= graph->getN() || pred[n3] != -1 || n3 == n2)
                            continue;

                        auto pair_n1_n3 = make_pair(n1, n3);
                        if (temp_blocked_arc_in_route.find(pair_n1_n3) != temp_blocked_arc_in_route.end())
                            continue;

                        // cout << "[*] Found arc " << n1 << " -> " << n3 << " with cost: " << n1_arc->getLength() << endl;

                        vector<int> path;
                        // Change function to remove path
                        int cost = this->sp->ShortestPathST(n3, n2, path);
                        // cout << "Cost from: " << n3 << " to: " << n2 << " is " << cost << endl;
                        if (n1_arc->getLength() + cost < lowest_route_option_cost)
                        {
                            lowest_route_option_cost = n1_arc->getLength();
                            best_route_option = make_pair(n1, n3);
                        }
                        // getchar();
                    }
                }
            }
        }

        // If no direct arc, find the shortest path
        if (lowest_cost >= INF && lowest_route_option_cost >= INF)
        {
            // Reset?
            cout << "[!] Dead end!" << endl;
            cout << "Last node: " << pred[last_node] << ", New Last node: " << last_node << endl;

            cost -= graph->getArc(pred[last_node], last_node)->getLength();
            auto lb_pair = make_pair(pred[last_node], last_node);
            temp_blocked_arc_in_route[lb_pair] = true;
            int new_last_node = pred[last_node];
            pred[last_node] = -1;
            cout << "Last node: " << last_node << ", New Last node: " << new_last_node << endl;

            if (temp_blocks_attended_in_node.find(last_node) != temp_blocks_attended_in_node.end())
            {
                auto blocks_to_insert_again = temp_blocks_attended_in_node[last_node];

                for (auto bl : blocks_to_insert_again)
                {
                    not_attended_blocks.insert(bl);
                    attended_block[bl] = false;
                }
            }

            last_node = new_last_node;

            getchar();
        }
        else
        {
            // The preference is the direct arc
            int n1 = lowest_arc.first, n2 = lowest_arc.second;
            last_block = block_l2, last_node = n2;

            // Otherwise, the preference is the shortest path
            if (lowest_cost >= INF)
            {
                n1 = best_route_option.first, n2 = best_route_option.second;
                last_node = n2, last_block = block_l1, next_block = block_l2;
                lowest_cost = lowest_route_option_cost;
            }

            cost += lowest_cost, pred[n2] = n1;
            cout << "With cost " << lowest_cost << " and path " << n1 << " -> " << n2 << endl;
            set<int> n1_blocks = graph->getNode(n1).second, n2_blocks = graph->getNode(n2).second;
            vector<int> result, attended_blocks;

            set_union(n1_blocks.begin(), n1_blocks.end(), n2_blocks.begin(), n2_blocks.end(), std::inserter(result, result.begin()));
            set_difference(not_attended_blocks.begin(), not_attended_blocks.end(), result.begin(), result.end(), std::inserter(attended_blocks, attended_blocks.begin()));

            not_attended_blocks = set<int>(attended_blocks.begin(), attended_blocks.end());

            cout << "Attended block: " << endl;
            for (int block : result)
            {
                cout << block << " ";
            }
            cout << endl;

            set<int> attended_in_n1 = set<int>();
            for (int block : n1_blocks)
            {
                if (!attended_block[block] && find(blocks.begin(), blocks.end(), block) != blocks.end())
                {
                    attended_block[block] = true;
                    attended_in_n1.insert(block);
                    cout << "Attended block: " << block << " in node " << n1 << endl;
                }
            }
            if (attended_in_n1.size() > 0)
                temp_blocks_attended_in_node[n1] = attended_in_n1;

            set<int> attended_in_n2 = set<int>();
            for (int block : n2_blocks)
            {
                if (!attended_block[block] && find(blocks.begin(), blocks.end(), block) != blocks.end())
                {
                    attended_block[block] = true;
                    attended_in_n2.insert(block);
                    cout << "Attended block: " << block << " in node " << n2 << endl;
                }
            }
            if (attended_in_n2.size() > 0)
                temp_blocks_attended_in_node[n2] = attended_in_n2;

            cout << "-----------------------------------------------" << endl;
            cout << "TEMP_BLOCKS_ATTENDED_IN_NODE: " << endl;
            for (auto pt : temp_blocks_attended_in_node)
            {
                cout << pt.first << ": ";
                for (auto b : pt.second)
                {
                    cout << b << " ";
                }
                cout << endl;
            }
            cout << "-----------------------------------------------" << endl;
            cout << "Not attended blocks: ";
            for (int block : not_attended_blocks)
                cout << block << " ";
            cout << endl;

            if (lowest_cost < INF)
            {
                // Get the first not attended block
                for (int bl : blocks)
                    if (bl != last_block && !attended_block[bl])
                        next_block = bl;
            }

            getchar();
        }
        cout << "Last node: " << last_node << endl;
    }
    int last = last_node;
    cout << "Nodes/Blocks: " << temp_blocks_attended_in_node.size() << endl;
    for (auto pt : temp_blocks_attended_in_node)
    {
        cout << pt.first << ": ";
        for (auto b : pt.second)
        {
            cout << b << " ";
        }
        cout << endl;
    }
    getchar();
    cout << "Blocks to attend: " << endl;
    for (auto b : blocks)
    {
        cout << b << " ";
    }
    cout << endl;
    while (last != pred[last])
    {
        cout << pred[last] << " -> " << last << " => ";
        for (auto b : temp_blocks_attended_in_node[last])
        {
            cout << b << " ";
        }
        cout << endl;
        last = pred[last];
        getchar();
    }
    pred[last] = graph->getN();

    cout << endl;
    cout << "Cost: " << cost << endl;
    getchar();
    return cost;
}

vector<vector<Arc>> BlockConnection::createLayeredDag(vector<int> nodes, map<int, int> &dag_2_graph, int &V)
{
    V = 0;
    vector<set<int>> nodes_per_block = this->graph->getNodesPerBlock();
    vector<vector<Arc>> dag;
    vector<int> path;

    for (int i = 0; i < nodes.size(); i++)
        V += nodes_per_block[nodes[i]].size();

    dag = vector<vector<Arc>>(V + 2, vector<Arc>());
    dag_2_graph[V] = dag_2_graph[V + 1] = graph->getN();

    // Populate the Layered DAG
    int inserted_nodes = 0;
    bool insert_depot = false;
    for (int i = 0; i < nodes.size() - 1; i++)
    {
        int b1 = nodes[i], b2 = nodes[i + 1];
        int jp = inserted_nodes, start_k = jp + nodes_per_block[b1].size();

        if (i + 1 >= nodes.size() - 1)
            insert_depot = true;

        for (int j : nodes_per_block[b1])
        {
            // Add dummy depot
            if (i == 0)
                dag[V].push_back(Arc(V, jp, 0, 0));

            dag_2_graph[jp] = j;
            int kp = start_k;

            for (int k : nodes_per_block[b2])
            {
                dag_2_graph[kp] = k;
                Arc arc = Arc(jp, kp, 0, 0);

                if (j != k)
                    arc.setLength(sp->ShortestPathST(j, k, path));

                dag[jp].push_back(arc);
                if (insert_depot)
                    dag[kp].push_back(Arc(kp, V + 1, 0, 0));

                kp++;
            }
            if (i + 1 >= nodes.size() - 1)
                insert_depot = false;

            jp++;
        }
        inserted_nodes += nodes_per_block[b1].size();
    }

    return dag;
}