#include "Input.hpp"

Input::Input(const string &file_graph, const string &scenarios_graph, bool preprocessing,
             bool is_trail, bool walk_mtz_model, int default_vel, int neblize_vel,
             int T, double alpha)
    : T(T)
    , default_vel(default_vel)
    , neblize_vel(neblize_vel)
    , alpha(alpha)
    , preprocessing(preprocessing)
    , is_trail(is_trail)
    , walk_mtz_model(walk_mtz_model)
    , graph(new Graph(file_graph, default_vel, neblize_vel)) {

    if (!scenarios_graph.empty())
        loadScenarios(scenarios_graph);

    sp = new ShortestPath(graph);
    bc = new BlockConnection(graph, sp);
    bc->computeBlock2BlockCost();

    const int N = graph->getN();
    arcs_in_path.resize(N, vector<vector<Arc *>>(N));
    arc_length.resize(N, vector<int>(N, -1));

    if (preprocessing)
        reduceGraphToPositiveCases();

    if (walk_mtz_model)
        walkAdaptMTZModel();

    startSimheuristic();

#ifndef Silence
    cout << "[*] Input constructed successfully!" << endl;
#endif
}

Input::Input(const string &file_graph, const string &scenarios_graph, int default_vel,
             int nebulize_vel, int T, double alpha)
    : T(T)
    , default_vel(default_vel)
    , neblize_vel(nebulize_vel)
    , alpha(alpha)
    , graph(new Graph(file_graph, default_vel, nebulize_vel)) {

    if (!scenarios_graph.empty())
        loadScenarios(scenarios_graph);

    sp = new ShortestPath(graph);
    bc = new BlockConnection(graph, sp);
    bc->computeBlock2BlockCost();
    updateFirstStageCases();

    const int N = graph->getN();
    arcs_in_path.resize(N, vector<vector<Arc *>>(N));
    arc_length.resize(N, vector<int>(N, -1));
    startSimheuristic();

#ifndef Silence
    cout << "[*] Input constructed successfully!" << endl;
#endif
}

void Input::updateBlocksInGraph(map<int, int> positive_block_to_block,
                                set<int> set_of_used_nodes,
                                vector<vector<bool>> used_arcs) {
    // Re-map blocks in nodes and arcs
    const int N = graph->getN();
    int newN = 0;
    vector<pair<int, set<int>>> new_nodes;
    vector<vector<Arc *>> new_arcs;
    map<int, int> map_new_nodes;
    vector<set<int>> nodes_per_block(graph->getPB());

    // Update Nodes
    for (int i = 0; i < N; ++i) {
        if (set_of_used_nodes.find(i) == set_of_used_nodes.end())
            continue;

        new_nodes.emplace_back(newN, set<int>());
        map_new_nodes[i] = newN;

        for (int b : graph->getNode(i).second) {
            if (b == -1)
                continue;

            const auto new_block_it = positive_block_to_block.find(b);
            if (new_block_it == positive_block_to_block.end() || new_block_it->second == -1)
                continue;

            const int bl = new_block_it->second;
            nodes_per_block[bl].insert(newN);
            new_nodes[newN].second.insert(bl);
        }
        new_arcs.emplace_back();
        ++newN;
    }

    graph->resetArcsMatrix(newN);

    // Update Arcs
    for (int i = 0; i < N; ++i) {
        if (set_of_used_nodes.find(i) == set_of_used_nodes.end())
            continue;

        for (const auto *arc : graph->getArcs(i)) {
            if (!used_arcs[i][arc->getD()])
                continue;

            const int new_o = map_new_nodes[i];
            const int new_d = map_new_nodes[arc->getD()];
            auto *new_arc = new Arc(*arc);
            new_arc->setO(new_o);
            new_arc->setD(new_d);
            new_arcs[new_o].push_back(new_arc);
            graph->addArcInMatrix(new_o, new_d, new_arc);
        }
    }

    graph->setNodes(new_nodes);
    graph->setArcs(new_arcs);
    graph->setNodesPerBlock(nodes_per_block);
    graph->setN(newN);

    int M = 0;
    for (int i = 0; i < newN; ++i)
        M += static_cast<int>(new_arcs[i].size());
    graph->setM(M);
}

void Input::getSetOfNodesPreprocessing(set<int> &used_nodes,
                                       vector<vector<bool>> &used_arcs) {
    const int B = graph->getB();

    if (graph->getPB() < 3)
        return;

    for (int b1 = 0; b1 < B; ++b1) {
        if (graph->getCasesPerBlock(b1) <= 0.0)
            continue;

        for (int b2 = 0; b2 < B; ++b2) {
            if (b2 == b1 || graph->getCasesPerBlock(b2) <= 0.0)
                continue;

            for (const auto i : graph->getNodesFromBlock(b1)) {
                for (const auto j : graph->getNodesFromBlock(b2)) {
                    if (i == j)
                        continue;

                    const vector<int> path = sp->getPath(i, j);

                    // Process intermediate nodes
                    for (size_t k = 1; k < path.size(); ++k) {
                        const int node = path[k];
                        for (const auto b3 : graph->getNode(node).second) {
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

void Input::reduceGraphToPositiveCases() {
    map<int, int> positive_block_to_block;
    vector<double> cases_per_block;
    vector<int> time_per_block;
    const int B = graph->getB();
    const int N = graph->getN();
    int new_index = 0;

    for (int b = 0; b < B; ++b) {
        bool has_cases = graph->getCasesPerBlock(b) > 0.0;

        if (!has_cases) {
            for (int s = 0; s < S; ++s) {
                if (scenarios[s].getCasesPerBlock(b) > 0.0) {
                    has_cases = true;
                    break;
                }
            }
        }

        if (has_cases) {
            positive_block_to_block[b] = new_index++;
            cases_per_block.push_back(graph->getCasesPerBlock(b));
            time_per_block.push_back(graph->getTimePerBlock(b));
        } else {
            positive_block_to_block[b] = -1;
        }
    }

    graph->setPB(new_index);

    for (int s = 0; s < S; ++s) {
        vector<double> cases_per_block_s(new_index, 0.0);
        for (int b = 0; b < B; ++b) {
            if (positive_block_to_block[b] != -1)
                cases_per_block_s[positive_block_to_block[b]] = scenarios[s].getCasesPerBlock(b);
        }
        scenarios[s].setCasesPerBlock(cases_per_block_s);
    }

#ifndef Silence
    cout << "[*] Reduction of blocks from " << B << " to " << new_index << endl;
#endif

    if (sp == nullptr)
        sp = new ShortestPath(graph);

    if (bc == nullptr)
        bc = new BlockConnection(graph, sp);

    set<int> used_nodes;
    vector<vector<bool>> used_arcs(N + 1, vector<bool>(N + 1, false));
    getSetOfNodesPreprocessing(used_nodes, used_arcs);

    updateBlocksInGraph(positive_block_to_block, used_nodes, used_arcs);

    graph->setCasesPerBlock(cases_per_block);
    graph->setTimePerBlock(time_per_block);
    graph->setB(new_index);

#ifndef Silence
    cout << "[*] Reduction of nodes from " << N << " to " << graph->getN() << endl;
#endif

    graph->addArtificialNode(graph->getN());

    // Update graph dependent structs
    delete sp;
    delete bc;
    sp = new ShortestPath(graph);
    bc = new BlockConnection(graph, sp);
    bc->computeBlock2BlockCost();

#ifndef Silence
    cout << "[*] Preprocessing finished!" << endl;
    cout << "[*] Resulting graph has " << graph->getN() << " nodes, "
         << graph->getM() << " arcs, and " << graph->getB() << " blocks" << endl;
#endif
}

void Input::loadScenarios(const string &instance) {
    ifstream file(instance);

    if (!file.is_open()) {
        cout << "[!] Could not open file: " << instance << endl;
        exit(EXIT_FAILURE);
    }

    file >> S;
    scenarios.resize(S);

    string token;
    int i, block, cases;
    double probability;

    while (file >> token) {
        if (token == "P") {
            file >> i >> probability;
            vector<double> cases_per_block(graph->getB(), 0.0);
            scenarios[i] = Scenario(probability, cases_per_block);
        } else if (token == "B") {
            file >> i >> block >> cases;
            scenarios[i].setCase2Block(block, cases);
        }
    }

#ifndef Silence
    cout << "[*] Scenarios loaded successfully" << endl;
#endif
}

bool Input::isNodeInPositiveValidBlock(int node) {
    const auto node_info = graph->getNode(node);

    for (const int b : node_info.second) {
        if (b == -1)
            continue;

        if (graph->getCasesPerBlock(b) > 0.0)
            return true;

        for (int s = 0; s < S; ++s) {
            if (scenarios[s].getCasesPerBlock(b) > 0.0)
                return true;
        }
    }

    return false;
}

void Input::walkAdaptMTZModel() {
#ifndef Silence
    cout << "[*] Block to block complete digraph adaptation" << endl;
#endif

    // Trail adapt MTZ model
    const int N = graph->getN();
    int M = graph->getM();
    vector<int> path;

    // Add new arcs to create complete graph
    for (int i = 0; i < N; ++i) {
        if (!isNodeInPositiveValidBlock(i))
            continue;

        for (int j = 0; j < N; ++j) {
            if (i == j || !isNodeInPositiveValidBlock(j) || graph->getArc(i, j) != nullptr)
                continue;

            const int length = sp->ShortestPathST(i, j, path);
            if (length != INF) {
                Arc *arc = new Arc(i, j, length, -1);
                graph->addArc(i, arc);
                ++M;
            }
        }
    }

    graph->setM(M);

#ifndef Silence
    cout << "[*] Complete graph created" << endl;
    cout << "[*] Resulting graph has " << graph->getN() << " nodes, "
         << graph->getM() << " arcs, and " << graph->getB() << " blocks" << endl;
#endif
}

void Input::filterMostDifferentScenarios(int new_s) {
    vector<double> cases_in_scenarios = graph->getCasesPerBlock();
    vector<Scenario> new_scenarios(new_s);
    map<int, bool> scenarios_used;
    const int B = graph->getB();

    int ns = 0;
    while (ns < new_s) {
        double diff_factor = -INF;
        int best_idx = -1;

        for (int s = 0; s < S; ++s) {
            if (scenarios_used.find(s) != scenarios_used.end())
                continue;

            const Scenario &scenario = scenarios[s];

            double diff = 0.0;
            for (int b = 0; b < B; ++b)
                diff += scenario.getCasesPerBlock(b) - cases_in_scenarios[b];

            if (diff > diff_factor) {
                best_idx = s;
                diff_factor = diff;
            }
        }

        cout << "Best scenario: " << best_idx << endl;

        for (int b = 0; b < B; ++b)
            cases_in_scenarios[b] += scenarios[best_idx].getCasesPerBlock(b);

        scenarios[best_idx].setProbability(1.0 / static_cast<double>(new_s));
        new_scenarios[ns++] = scenarios[best_idx];
        scenarios_used[best_idx] = true;
    }

    S = new_s;
    scenarios = new_scenarios;
}
