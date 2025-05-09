//
// Created by carlos on 06/07/21.
//

#include "Graph.hpp"
#include "Arc.hpp"

Graph::Graph(string instance, int km_path, int km_nebulize) {
    if (instance != "") {
        LoadGraph(instance, km_path, km_nebulize);
        this->ComputeNodeBlockHops();
    } else
        exit(EXIT_FAILURE);
}

void Graph::LoadGraph(string instance, int km_path, int km_nebulize) {
    int block, cases, i, j, k;
    double length;
    string token, aux, x, y;
    ifstream file;
    vector<string> blcs;

    file.open(instance, fstream::in);

    if (!file.is_open()) {
        cout << "[!] Could not open file: " << instance << endl;
        exit(EXIT_FAILURE);
    }

    // Default Graph dimensions
    file >> Graph::N >> Graph::M >> Graph::B;

    // Basic structures
    arcs = vector<vector<Arc *>>(N + 1, vector<Arc *>());
    arcs_matrix = vector<vector<Arc *>>(N + 2, vector<Arc *>(N + 2, nullptr));
    nodes_per_block = vector<set<int>>(B, set<int>());
    arcs_per_block = vector<vector<Arc *>>(B, vector<Arc *>());
    set<int> blocks_node;

    // Filtering blocks with positive amount of cases
    cases_per_block = vector<double>(B, 0);
    time_per_block = vector<int>(B, 0);

    // Load Graph
    double mp_path = (double(km_path * 1000) / 60.0);
    double mp_nebu = (double(km_nebulize * 1000) / 60.0);

    while (!file.eof()) {
        file >> token;
        if (token == "N") {
            file >> j >> x >> y >> aux;
            boost::split(blcs, aux, boost::is_any_of(","));
            blocks_node = set<int>();

            for (const auto &s : blcs) {
                if (s == "-1")
                    break;

                k = stoi(s);
                blocks_node.insert(k);
                nodes_per_block[k].insert(j);
            }
            nodes.emplace_back(j, blocks_node);
        } else if (token == "A") {
            file >> i >> j >> length >> block;

            if (token != "A")
                continue;

            file.ignore(numeric_limits<streamsize>::max(), '\n');
            double t_time = 10.0 * (length / mp_path);
            int travel_time = t_time > 0 ? int(ceil(t_time)) : 1;

            if (travel_time < 1) {
                cout << travel_time << endl;
                getchar();
            }

            Arc *arc = new Arc(i, j, travel_time, block);

            if (arcs_matrix[i][j] != nullptr) {
                arcs_matrix[i][j]->setBlock(block);
                arc = arcs_matrix[i][j];
            } else {
                arcs_matrix[i][j] = arc;
                arcs[i].push_back(arc);
            }

            if (arcs_matrix[j][i] == nullptr) {
                Arc *rev_arc = new Arc(j, i, travel_time, -1);
                arcs_matrix[j][i] = rev_arc;
                this->arcs[j].push_back(rev_arc);
            }

            if (block != -1) {
                arcs_per_block[block].push_back(arc);
                double nebu_time = 10.0 * (length / mp_nebu);
                time_per_block[block] += nebu_time > 0 ? ceil(nebu_time) : 1;
            }
        } else if (token == "B") {
            file >> block >> cases;
            cases_per_block[block] = cases;
        }
    }

    nodes.emplace_back(N, set<int>());
    for (i = 0; i < N; i++) {
        Arc *from_s = new Arc(N, i, 0, -1);
        Arc *to_t = new Arc(i, N, 0, -1);
        arcs[N].push_back(from_s), arcs[i].push_back(to_t);
        arcs_matrix[N][i] = from_s, arcs_matrix[i][N] = to_t;
    }

#ifndef Silence
    cout << "[*] Load graph successfully!" << endl;
#endif

    file.close();
}
