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

    updateFirstStageCases();
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

    const int N = graph->getN();
    const int B = graph->getB();

    // Step 1: Identify all nodes in positive/valid blocks and map blocks
    map<int, int> old_block_to_new_block;
    map<int, int> old_node_to_new_node;
    vector<int> valid_nodes;
    set<int> valid_blocks_set;
    int newPB = 0;

    // Identify valid blocks (blocks with positive cases)
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
            old_block_to_new_block[b] = newPB++;
            valid_blocks_set.insert(b);
        }
    }

    // Identify valid nodes (nodes that belong to valid blocks)
    for (int i = 0; i < N; ++i) {
        const auto &node_info = graph->getNode(i);
        bool is_valid = false;

        for (const int b : node_info.second) {
            if (b != -1 && valid_blocks_set.find(b) != valid_blocks_set.end()) {
                is_valid = true;
                break;
            }
        }

        if (is_valid) {
            old_node_to_new_node[i] = static_cast<int>(valid_nodes.size());
            valid_nodes.push_back(i);
        }
    }

    const int newN = static_cast<int>(valid_nodes.size());

#ifndef Silence
    cout << "[*] Reduction from " << N << " to " << newN << " nodes" << endl;
    cout << "[*] Reduction from " << B << " to " << newPB << " blocks" << endl;
#endif

    // Step 2: Create new nodes structure with remapped block IDs
    vector<pair<int, set<int>>> new_nodes(newN);
    vector<set<int>> new_nodes_per_block(newPB);

    for (int new_i = 0; new_i < newN; ++new_i) {
        const int old_i = valid_nodes[new_i];
        const auto &old_node_info = graph->getNode(old_i);

        new_nodes[new_i].first = new_i;
        new_nodes[new_i].second.clear();

        for (const int old_b : old_node_info.second) {
            if (old_b != -1 && old_block_to_new_block.find(old_b) != old_block_to_new_block.end()) {
                const int new_b = old_block_to_new_block[old_b];
                new_nodes[new_i].second.insert(new_b);
                new_nodes_per_block[new_b].insert(new_i);
            }
        }
    }

    // Step 3: Create complete graph with all arcs between valid nodes
    vector<vector<Arc *>> new_arcs(newN + 1);
    int newM = 0;

    for (int i = 0; i < newN; ++i) {
        for (int j = 0; j < newN; ++j) {
            if (i == j)
                continue;

            const int old_i = valid_nodes[i];
            const int old_j = valid_nodes[j];

            // Calculate shortest path in original graph
            vector<int> path;
            const int length = sp->ShortestPathST(old_i, old_j, path);

            if (length != INF) {
                Arc *arc = new Arc(i, j, length, -1);
                new_arcs[i].push_back(arc);
                ++newM;
            }
        }
    }

    // Step 4: Update block information (cases and times)
    vector<double> new_cases_per_block(newPB);
    vector<int> new_time_per_block(newPB);

    for (const auto &[old_b, new_b] : old_block_to_new_block) {
        new_cases_per_block[new_b] = graph->getCasesPerBlock(old_b);
        new_time_per_block[new_b] = graph->getTimePerBlock(old_b);
    }

    // Step 5: Update scenarios with new block mapping
    for (int s = 0; s < S; ++s) {
        vector<double> new_scenario_cases(newPB, 0.0);
        for (const auto &[old_b, new_b] : old_block_to_new_block) {
            new_scenario_cases[new_b] = scenarios[s].getCasesPerBlock(old_b);
        }
        scenarios[s].setCasesPerBlock(new_scenario_cases);
    }

    // Step 6: Replace graph structures
    graph->setNodes(new_nodes);
    graph->setArcs(new_arcs);
    graph->setNodesPerBlock(new_nodes_per_block);
    graph->setCasesPerBlock(new_cases_per_block);
    graph->setTimePerBlock(new_time_per_block);
    graph->setN(newN);
    graph->setM(newM);
    graph->setB(newPB);
    graph->setPB(newPB);
    graph->resetArcsMatrix(newN);

    // Rebuild arcs matrix for the new graph
    for (int i = 0; i < newN; ++i) {
        for (Arc *arc : new_arcs[i]) {
            graph->addArcInMatrix(i, arc->getD(), arc);
        }
    }

    // Add artificial depot node
    graph->addArtificialNode(newN);

    // Step 7: Recreate shortest path and block connection for new graph
    delete sp;
    delete bc;
    sp = new ShortestPath(graph);
    bc = new BlockConnection(graph, sp);
    bc->computeBlock2BlockCost();

    // Resize arc structures
    const int final_N = graph->getN();
    arcs_in_path.clear();
    arcs_in_path.resize(final_N, vector<vector<Arc *>>(final_N));
    arc_length.clear();
    arc_length.resize(final_N, vector<int>(final_N, -1));

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
