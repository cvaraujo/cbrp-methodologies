#include "Input.hpp"

Input::Input(string file_graph, string scenarios_graph, int graph_adapt, int default_vel, int neblize_vel, int T, double alpha)
{
    this->graph = new Graph(file_graph, default_vel, neblize_vel);
    if (scenarios_graph != "")
        this->loadScenarios(scenarios_graph);

    this->sp = new ShortestPath(this->graph);
    this->bc = new BlockConnection(this->graph, this->sp);
    this->T = T;
    this->graph_adapt = graph_adapt;
    this->default_vel = default_vel;
    this->neblize_vel = neblize_vel;
    this->alpha = alpha;

    if (this->graph_adapt == 1)
        this->reduceGraphToPositiveCases();

#ifndef Silence
    cout << "[***] Input constructed Successfully!" << endl;
#endif
}
void Input::reduceGraphToPositiveCases()
{
    // Re-map blocks
    map<int, int> positive_block_to_block;
    vector<double> cases_per_block;
    vector<int> time_per_block;
    vector<set<int>> nodes_per_block;
    int B = graph->getB(), new_index = 0, N = graph->getN();

    for (int b = 0; b < B; b++)
    {
        bool has_cases = graph->getCasesPerBlock(b) > 0;
        if (!has_cases)
        {
            for (int s = 0; s < S; s++)
            {
                if (scenarios[s].getCasesPerBlock(b) > 0)
                {
                    has_cases = true;
                    break;
                }
            }
        }

        if (has_cases)
        {
            positive_block_to_block[b] = new_index++;
            cases_per_block.push_back(graph->getCasesPerBlock(b));
            time_per_block.push_back(graph->getTimePerBlock(b));
            nodes_per_block.push_back(graph->getNodesFromBlock(b));
        }
        else
            positive_block_to_block[b] = -1;
    }

    graph->setPB(new_index);
    for (int s = 0; s < S; s++)
    {
        vector<double> cases_per_block_s(new_index, 0);
        for (int b = 0; b < B; b++)
            if (positive_block_to_block[b] != -1)
                cases_per_block_s[positive_block_to_block[b]] = scenarios[s].getCasesPerBlock(b);
        scenarios[s].setCasesPerBlock(cases_per_block_s);
    }

#ifndef Silence
    cout << "[*] Reduction of " << B - new_index << " blocks" << endl;
#endif

    // Re-map blocks in nodes and arcs
    for (int i = 0; i < N; i++)
    {
        auto blocks_from_i = graph->getNodes(i).second;
        set<int> new_blocks_from_i;

        for (auto block : blocks_from_i)
        {
            if (block == -1)
            {
                new_blocks_from_i = set<int>{};
                break;
            }
            auto new_block = positive_block_to_block.find(block);
            if (new_block != positive_block_to_block.end() && new_block->second != -1)
                new_blocks_from_i.insert(new_block->second);
        }

        graph->setBlocksFromNode(i, new_blocks_from_i);
    }

    // Remove unecessary arcs between blocks
    vector<vector<int>> block_2_block_cost = vector<vector<int>>(B, vector<int>(B, INF));
    set<int> set_of_used_nodes;
    map<int, map<int, bool>> map_used_arcs;

    for (int b = 0; b < B - 1; b++)
    {
        for (int b2 = b + 1; b2 < B; b2++)
        {
            set<int> nodes_in_path;
            map<int, map<int, bool>> arcs_in_path;
            int cost = sp->SHPBetweenBlocks(b, b2, nodes_in_path, arcs_in_path);

            if (cost < INF)
            {
                set<int> union_set;
                set_union(nodes_in_path.begin(), nodes_in_path.end(), set_of_used_nodes.begin(), set_of_used_nodes.end(), inserter(union_set, union_set.begin()));
                set_of_used_nodes = union_set;
                block_2_block_cost[b][b2] = block_2_block_cost[b2][b] = cost;
            }
        }
    }

    this->bc->setBlock2BlockCost(block_2_block_cost);

    int newN = 0;
    graph->setB(new_index);
    B = graph->getB();
    vector<vector<Arc *>> new_arcs;
    vector<pair<int, set<int>>> new_nodes;
    map<int, int> map_new_nodes;
    nodes_per_block = vector<set<int>>(B);
    graph->setCasesPerBlock(cases_per_block);
    graph->setTimePerBlock(time_per_block);

    for (int i = 0; i < N; i++)
    {
        if (set_of_used_nodes.find(i) != set_of_used_nodes.end())
        {
            new_nodes.push_back(pair<int, set<int>>());
            new_nodes[newN].first = newN;
            map_new_nodes[i] = newN;

            for (int b : graph->getNode(i).second)
                if (b != -1)
                {
                    nodes_per_block[b].insert(newN);
                    new_nodes[newN].second.insert(b);
                }

            new_arcs.push_back(vector<Arc *>());
            newN++;
        }
    }

    for (int i = 0; i < N; i++)
    {
        if (map_new_nodes.find(i) == map_new_nodes.end())
            continue;

        for (auto arc : graph->getArcs(i))
        {
            int new_o = map_new_nodes[i];

            if (map_new_nodes.find(arc->getD()) != map_new_nodes.end()) //(map_used_arcs[i][arc->getD()])
            {
                int new_d = map_new_nodes[arc->getD()];
                auto new_arc = new Arc(*arc);
                new_arc->setO(new_o), new_arc->setD(new_d);
                new_arcs[new_o].push_back(new_arc);
            }
        }
    }

#ifndef Silence
    cout << "[*] Reduction of nodes from " << N << " to " << newN << endl;
#endif

    // Update number of nodes
    N = newN;
    new_nodes.push_back(make_pair(N, set<int>()));
    new_arcs.push_back(vector<Arc *>());
    for (int i = 0; i < N; i++)
        new_arcs[N].push_back(new Arc(N, i, 0, -1)), new_arcs[i].push_back(new Arc(i, N, 0, -1));
    graph->setArcs(new_arcs), graph->setNodes(new_nodes);
    graph->setN(N);
    graph->setNodesPerBlock(nodes_per_block);

    // for (auto node : new_nodes)
    // {
    //     cout << "N: " << node.first << endl;
    //     for (auto block : node.second)
    //         cout << "B:" << block << " ";
    //     cout << endl;
    // }
    // getchar();

#ifndef Silence
    cout << "[*] Preprocessing Finished!" << endl;
#endif
}

void Input::loadScenarios(string instance)
{
    string token;
    ifstream file;
    int i, block, cases;
    double probability;
    file.open(instance, fstream::in);

    if (!file.is_open())
    {
        cout << "[!] Could not open file: " << instance << endl;
        exit(EXIT_FAILURE);
    }

    file >> this->S;
    this->scenarios = vector<Scenario>(S);

    while (!file.eof())
    {
        file >> token;
        if (token == "P")
        {
            file >> i >> probability;
            vector<double> cases_per_block = vector<double>(graph->getB(), 0);
            Scenario scn(probability, cases_per_block);
            this->scenarios[i] = scn;
        }
        else if (token == "B")
        {
            file >> i >> block >> cases;
            this->scenarios[i].setCase2Block(block, cases);
        }
    }
#ifndef Silence
    cout << "Load Scenarios successfully" << endl;
#endif
}