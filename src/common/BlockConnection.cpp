
#include "BlockConnection.hpp"

int BlockConnection::HeuristicBlockConnection(Graph *graph, ShortestPath *sp, vector<int> blocks, string key)
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

    // SHP on DAG
    vector<int> pred;
    vector<int> path = vector<int>();
    int cost = ShortestPath::DijkstraLayeredDAG(dag, V + 2, V, V + 1, pred);

    // Get the path
    int v = V + 1, last_inserted = -1;

    while (v != pred[v])
    {
        if (dag_2_graph[v] != last_inserted)
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
    this->setBestOrderToAttendBlocks(key, blocks);
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