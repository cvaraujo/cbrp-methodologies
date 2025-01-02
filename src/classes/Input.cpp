#include "Input.hpp"

Input::Input(string file_graph, string scenarios_graph, bool preprocessing, bool is_trail, bool walk_mtz_model, int default_vel, int neblize_vel, int T, double alpha)
{
    this->graph = new Graph(file_graph, default_vel, neblize_vel);

    if (scenarios_graph != "")
        this->loadScenarios(scenarios_graph);

    this->T = T;
    this->preprocessing = preprocessing;
    this->default_vel = default_vel;
    this->neblize_vel = neblize_vel;
    this->alpha = alpha;
    this->is_trail = is_trail;
    this->walk_mtz_model = walk_mtz_model;
    this->sp = new ShortestPath(graph);
    this->bc = new BlockConnection(graph, sp);
    this->bc->computeBlock2BlockCost();

    if (preprocessing)
        this->reduceGraphToPositiveCases();

    if (walk_mtz_model)
        this->walkAdaptMTZModel();

#ifndef Silence
    cout << "[***] Input constructed Successfully!" << endl;
#endif
}

void Input::updateBlocksInGraph(map<int, int> positive_block_to_block, set<int> set_of_used_nodes, vector<vector<bool>> used_arcs)
{
    // Re-map blocks in nodes and arcs
    int N = graph->getN(), newN = 0, bl;
    vector<pair<int, set<int>>> new_nodes;
    vector<vector<Arc *>> new_arcs;
    map<int, int> map_new_nodes;
    vector<set<int>> nodes_per_block = vector<set<int>>(graph->getPB());

    // Update Nodes
    for (int i = 0; i < N; i++)
    {
        if (set_of_used_nodes.find(i) == set_of_used_nodes.end())
        {
            cout << "Remove node " << i << endl;
            continue;
        }

        new_nodes.push_back(pair<int, set<int>>());
        new_nodes[newN].first = newN;
        map_new_nodes[i] = newN;

        for (int b : graph->getNode(i).second)
        {
            if (b == -1)
                continue;

            auto new_block = positive_block_to_block.find(b);
            if (new_block == positive_block_to_block.end() || new_block->second == -1)
                continue;

            bl = new_block->second;
            nodes_per_block[bl].insert(newN);
            new_nodes[newN].second.insert(bl);
        }
        new_arcs.push_back(vector<Arc *>());
        newN++;
    }

    this->graph->resetArcsMatrix(newN);

    // Update Arcs
    for (int i = 0; i < N; i++)
    {
        if (set_of_used_nodes.find(i) == set_of_used_nodes.end())
            continue;

        for (auto arc : graph->getArcs(i))
        {
            if (!used_arcs[i][arc->getD()])
            {
                cout << "Remove arc " << i << " - " << arc->getD() << endl;
                continue;
            }

            int new_o = map_new_nodes[i], new_d = map_new_nodes[arc->getD()];
            auto new_arc = new Arc(*arc);
            new_arc->setO(new_o), new_arc->setD(new_d);
            new_arcs[new_o].push_back(new_arc);
            this->graph->addArcInMatrix(new_o, new_d, new_arc);
        }
    }

    this->graph->setNodes(new_nodes);
    this->graph->setArcs(new_arcs);
    this->graph->setNodesPerBlock(nodes_per_block);
    this->graph->setN(newN);
}

void Input::getSetOfNodesPreprocessing(set<int> &used_nodes, vector<vector<bool>> &used_arcs)
{
    int B = graph->getB();
    vector<int> path;
    if (graph->getPB() < 3)
        return;

    for (int b1 = 0; b1 < B; b1++)
    {
        if (graph->getCasesPerBlock(b1) <= 0)
            continue;

        for (int b2 = 0; b2 < B; b2++)
        {
            if (b2 == b1 || graph->getCasesPerBlock(b2) <= 0)
                continue;

            for (auto i : graph->getNodesFromBlock(b1))
            {
                for (auto j : graph->getNodesFromBlock(b2))
                {
                    if (i == j)
                        continue;

                    path = this->sp->getPath(i, j);

                    // Intermediate nodes
                    for (int k = 0; k < path.size(); k++)
                    {
                        int node = path[k];
                        for (auto b3 : graph->getNode(node).second)
                        {
                            if (b3 == -1 || b3 == b1 || b3 == b2)
                                continue;

                            used_nodes.insert(node);
                            used_arcs[path[k - 1]][path[k]] = true;
                        }
                    }
                }
            }
        }
    }
}

void Input::reduceGraphToPositiveCases()
{
    // Re-map blocks
    map<int, int> positive_block_to_block;
    vector<double> cases_per_block;
    vector<int> time_per_block;
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
    cout << "[*] Reduction of Blocks from " << B << " to " << new_index << endl;
#endif

    if (this->sp == nullptr)
        this->sp = new ShortestPath(graph);

    if (this->bc == nullptr)
        this->bc = new BlockConnection(graph, sp);

    set<int> used_nodes;
    vector<vector<bool>> used_arcs = vector<vector<bool>>(N + 1, vector<bool>(N + 1, false));
    this->getSetOfNodesPreprocessing(used_nodes, used_arcs);

    this->updateBlocksInGraph(positive_block_to_block, used_nodes, used_arcs);

    graph->setCasesPerBlock(cases_per_block);
    graph->setTimePerBlock(time_per_block);
    graph->setB(new_index);

#ifndef Silence
    cout << "[*] Reduction of Nodes from " << N << " to " << graph->getN() << endl;
#endif

    graph->addArtificialNode(graph->getN());

    // Update graph dependent structs
    this->sp = new ShortestPath(graph);
    this->bc = new BlockConnection(graph, sp);
    this->bc->computeBlock2BlockCost();

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

bool isNodeInPositiveValidBlock(Graph *graph, int node)
{
    auto node_info = graph->getNode(node);

    for (int b : node_info.second)
        if (b != -1 && graph->getCasesPerBlock(b) > 0)
            return true;

    return false;
}

void Input::walkAdaptMTZModel()
{
#ifndef Silence
    cout << "[*] Block to Block Complete Digraph Adapt" << endl;
#endif

    // trail adapt MTZ model
    int i, length;
    vector<int> path;

    // Add new arcs
    for (int i = 0; i < graph->getN(); i++)
    {
        if (!isNodeInPositiveValidBlock(graph, i))
            continue;

        for (int j = 0; j < graph->getN(); j++)
        {
            if (i == j || !isNodeInPositiveValidBlock(graph, j) || graph->getArc(i, j) != nullptr)
                continue;

            length = sp->ShortestPathST(i, j, path);
            if (length != INF)
            {
                Arc *arc = new Arc(i, j, length, -1);
                graph->addArc(i, arc);
            }
        }
    }
#ifndef Silence
    cout << "[*] Create Complete Graph" << endl;
#endif
}

void Input::filterMostDifferentScenarios(int new_s)
{
    vector<double> cases_in_scenarios = this->graph->getCasesPerBlock();
    vector<Scenario> new_scenarios = vector<Scenario>(new_s);
    map<int, bool> scenarios_used;

    int ns = 0;
    while (ns < new_s)
    {
        double diff_factor = -INF;
        int best_idx = -1;

        for (int s = 0; s < this->S; s++)
        {
            if (scenarios_used.find(s) != scenarios_used.end())
                continue;

            Scenario scenario = this->scenarios[s];

            double diff = 0.0;
            for (int b = 0; b < graph->getB(); b++)
                diff += scenario.getCasesPerBlock(b) - cases_in_scenarios[b];

            if (diff > diff_factor)
            {
                best_idx = s;
                diff_factor = diff;
            }
        }

        cout << "Best Scenario: " << best_idx << endl;

        for (int b = 0; b < graph->getB(); b++)
            cases_in_scenarios[b] += this->scenarios[best_idx].getCasesPerBlock(b);

        this->scenarios[best_idx].setProbability(1.0 / double(new_s));
        new_scenarios[ns++] = this->scenarios[best_idx];
        scenarios_used[best_idx] = true;
    }

    this->S = new_s;
    this->scenarios = new_scenarios;
}
