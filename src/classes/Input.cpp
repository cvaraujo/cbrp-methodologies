#include "Input.hpp"

Input::Input(string file_graph, string scenarios_graph, bool preprocessing, bool is_trail, bool block_2_block_graph, int default_vel, int neblize_vel, int T, double alpha)
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
    this->block_2_block_graph = block_2_block_graph;

    if (preprocessing)
        this->reduceGraphToPositiveCases();

    if (block_2_block_graph)
        this->walkAdaptMTZModel();

#ifndef Silence
    cout << "[***] Input constructed Successfully!" << endl;
#endif
}

void Input::updateBlocksInGraph(map<int, int> positive_block_to_block, set<int> set_of_used_nodes)
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
            continue;

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

    // Update Arcs
    for (int i = 0; i < N; i++)
    {
        if (set_of_used_nodes.find(i) == set_of_used_nodes.end())
            continue;

        for (auto arc : graph->getArcs(i))
        {
            int new_o = map_new_nodes[i];

            if (map_new_nodes.find(arc->getD()) != map_new_nodes.end())
            {
                int new_d = map_new_nodes[arc->getD()];
                auto new_arc = new Arc(*arc);
                new_arc->setO(new_o), new_arc->setD(new_d);
                new_arcs[new_o].push_back(new_arc);
            }
        }
    }

    this->graph->setNodes(new_nodes);
    this->graph->setArcs(new_arcs);
    this->graph->setNodesPerBlock(nodes_per_block);
    this->graph->setN(newN);
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

#ifndef Silence
    cout << "[*] Create SHP and BC" << endl;
#endif

    set<int> set_of_used_nodes = bc->computeBlock2BlockCost();
    this->updateBlocksInGraph(positive_block_to_block, set_of_used_nodes);

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

void Input::walkAdaptMTZModel()
{
#ifndef Silence
    cout << "[*] Block to Block Complete Digraph Adapt" << endl;
#endif

    // trail adapt MTZ model
    int i, length, B = graph->getB();
    vector<vector<Arc *>> new_arcs = vector<vector<Arc *>>(B + 1);
    vector<vector<Arc *>> new_arcs_matrix = vector<vector<Arc *>>(B + 1, vector<Arc *>(B + 1));
    vector<pair<int, set<int>>> new_nodes = vector<pair<int, set<int>>>(B + 1);
    vector<set<int>> new_nodes_per_block = vector<set<int>>(B + 1);

    int new_n = B;
    for (int b = 0; b < B; b++)
    {
        new_nodes[b] = make_pair(b, set<int>{b});
        new_nodes_per_block[b] = set<int>{b};

        for (int bl = b + 1; bl < B; bl++)
        {
            length = bc->getBlock2BlockCost(b, bl);
            cout << "Distance from " << b << " to " << bl << ": " << length << endl;
            Arc *arc = new Arc(b, bl, length, -1);
            Arc *rev_arc = new Arc(bl, b, length, -1);

            new_arcs[b].push_back(arc), new_arcs[bl].push_back(rev_arc);
            new_arcs_matrix[b][bl] = arc, new_arcs_matrix[bl][b] = rev_arc;
        }
    }

    getchar();
    // Add artificial nodes
    new_nodes.push_back(make_pair(B, set<int>()));
    new_arcs.push_back(vector<Arc *>());
    for (int i = 0; i < B; i++)
        new_arcs[B].push_back(new Arc(B, i, 0, -1)), new_arcs[i].push_back(new Arc(i, B, 0, -1));

    // Update graph
    this->graph->setArcs(new_arcs), this->graph->setArcsMatrix(new_arcs_matrix);
    this->graph->setNodes(new_nodes), this->graph->setNodesPerBlock(new_nodes_per_block);
    this->graph->setN(new_n);

    // Show new graph
    for (int i = 0; i <= graph->getN(); i++)
    {
        for (auto arc : graph->getArcs(i))
            cout << arc->getO() << " " << arc->getD() << " " << arc->getLength() << endl;
        cout << endl;
    }
#ifndef Silence
    cout << "Walk Adapt MTZ Model Finished!" << endl;
#endif
}