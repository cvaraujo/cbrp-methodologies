#include "../headers/GreedyHeuristic.h"

GreedyHeuristic::GreedyHeuristic(Graph *graph)
{
    this->graph = graph;
}

float GreedyHeuristic::SolveScenario(vector<double> cases, vector<int> time, float route_time_increase, int max_tries, vector<int> &y, vector<dpair> &x)
{
    // Start greedy heuristic
    int T = graph->getT();
    int available_time = T * (1.0 - route_time_increase), tries = 1;
    double of = 0;

    while (tries <= max_tries)
    {
        of = graph->knapsack(y, cases, time, available_time);

        if (y.empty())
            break;

        // Get attend time
        int block_attended_time = 0;
        for (auto b : y)
            block_attended_time += graph->time_per_block[b];

        // Generate a hash to solution
        sort(y.begin(), y.end());
        string key = graph->generateStringFromIntVector(y);

        // Get route cost
        if (graph->savedBlockConn.find(key) == graph->savedBlockConn.end())
            graph->ConnectBlocks(y, key);

        // cout << key << endl;
        // cout << block_attended_time << ", " << graph->savedBlockConn[key] << " <= " << available_time << endl;
        // getchar();
        if (block_attended_time + graph->savedBlockConn[key] <= T)
        {
            auto vertices = graph->savedBlockConnPath[key];
            for (int j = 0; j < vertices.size() - 1; j++)
                x.push_back({vertices[j], vertices[j + 1]});
            break;
        }
        else
        {
            y = vector<int>();
            x = vector<dpair>();
            available_time = T * (1.0 - float(++tries) * route_time_increase);
        }
    }

    return of;
}

float GreedyHeuristic::Run(float route_time_increase, int max_tries, vector<vector<pair<int, int>>> &sol_x, vector<vector<pair<int, int>>> &sol_y)
{
    // Get all blocks
    int S = graph->getS(), T = graph->getT(), B = graph->getB();
    vector<int> blocks = vector<int>(B, 0), time = vector<int>(B, 0);
    vector<double> cases = vector<double>(B, 0);
    vector<bool> in_first_stage = vector<bool>(B, false);
    float alpha = graph->getAlpha();
    double of = 0;

    // Solve First Stage
    for (int i = 0; i < B; i++)
    {
        blocks[i] = i;
        time[i] = graph->time_per_block[i];
        cases[i] = graph->cases_per_block[i];

        for (int s = 0; s < S; s++)
            cases[i] += (alpha * graph->scenarios[s].probability * graph->scenarios[s].cases_per_block[i]);
    }

    vector<vector<int>> y = vector<vector<int>>(S + 1, vector<int>());
    vector<vector<dpair>> x = vector<vector<dpair>>(S + 1, vector<dpair>());
    of += SolveScenario(cases, time, route_time_increase, max_tries, y[0], x[0]);

    // cout << "OF: " << of << endl;
    // getchar();

    // Solve Second Stage
    for (auto i : y[0])
        in_first_stage[i] = true;

    for (int s = 0; s < S; s++)
    {
        bool all_zeros = true;
        for (int i = 0; i < B; i++)
        {
            cases[i] = graph->scenarios[s].cases_per_block[i];
            if (in_first_stage[i])
                cases[i] *= (1 - alpha);

            if (cases[i] > 0)
                all_zeros = false;

            // cout << "B" << i << ": " << cases[i] << endl;
        }

        if (!all_zeros)
            of += graph->scenarios[s].probability * SolveScenario(cases, time, route_time_increase, max_tries, y[s + 1], x[s + 1]);

        // cout << "OF: " << of << endl;
        // getchar();
    }

    for (int s = 0; s < S + 1; s++)
    {
        sol_y.push_back(vector<pair<int, int>>());
        sol_x.push_back(vector<pair<int, int>>());
        cout << "Scenario " << s << endl;

        for (auto b : y[s])
            cout << "Y: " << b << endl;

        for (auto i : x[s])
            if (i.first > graph->getN())
                cout << "X: " << i.first - 1 << " - " << i.second << endl;
            else
            {
                sol_x[s].push_back({i.second, i.first});
                cout << "X: " << i.second << " - " << i.first << endl;
            }
        cout << "--------------------------" << endl;
    }

    return of;
}