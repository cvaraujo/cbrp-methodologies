#include "GreedyHeuristic.hpp"

float GreedyHeuristic::SolveScenario(vector<double> cases, vector<int> time, float route_time_increase, int max_tries, vector<int> &y, vector<int_pair> &x)
{
    // Start greedy heuristic
    Graph *graph = input->getGraph();
    BlockConnection *bc = input->getBlockConnection();
    int T = input->getT();
    int available_time = T, tries = 1;
    double of = 0;

    // cout << "Available time: " << available_time << endl;

    while (tries <= max_tries)
    {
        of = Knapsack::Run(y, cases, time, available_time);

        if (y.empty())
            break;

        // Get attend time
        int block_attended_time = 0;
        for (auto b : y)
            block_attended_time += graph->getTimePerBlock(b);

        // Generate a hash to solution
        sort(y.begin(), y.end());
        string key = bc->GenerateStringFromIntVector(y);

        // Get route cost
        if (!bc->keyExists(key))
            bc->HeuristicBlockConnection(graph, input->getShortestPath(), y, key);

        if (block_attended_time + bc->getBlocksAttendCost(key) <= T)
        {
            auto vertices = bc->getBlocksAttendPath(key);
            for (int j = 0; j < vertices.size() - 1; j++)
                x.push_back({vertices[j], vertices[j + 1]});
            break;
        }
        else
        {
            y = vector<int>();
            x = vector<int_pair>();
            available_time = T * (1.0 - float(tries++) * route_time_increase);
        }
    }

    return of;
}

float GreedyHeuristic::Run(float route_time_increase, int max_tries, vector<vector<pair<int, int>>> &sol_x, vector<vector<pair<int, int>>> &sol_y)
{
    // Get all blocks
    Graph *graph = input->getGraph();
    int S = input->getS(), T = input->getT(), B = graph->getB();
    vector<int> blocks = vector<int>(B, 0), time = vector<int>(B, 0);
    vector<double> cases = vector<double>(B, 0);
    vector<bool> in_first_stage = vector<bool>(B, false);
    float alpha = input->getAlpha();
    double of = 0;

    // Solve First Stage
    for (int i = 0; i < B; i++)
    {
        blocks[i] = i;
        time[i] = graph->getTimePerBlock(i);
        cases[i] = graph->getCasesPerBlock(i);

        for (int s = 0; s < S; s++)
        {
            Scenario scenario = input->getScenario(s);
            cases[i] += (alpha * scenario.getProbability() * scenario.getCasesPerBlock(i));
        }
        // cout << "B" << i << ": " << cases[i] << ", " << time[i] << endl;
    }

    vector<vector<int>> y = vector<vector<int>>(S + 1, vector<int>());
    vector<vector<int_pair>> x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
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
            cases[i] = input->getScenario(s).getCasesPerBlock(i);
            if (in_first_stage[i])
                cases[i] *= (1 - alpha);

            if (cases[i] > 0)
                all_zeros = false;

            // cout << "B" << i << ": " << cases[i] << endl;
        }

        if (!all_zeros)
            of += input->getScenario(s).getProbability() * SolveScenario(cases, time, route_time_increase, max_tries, y[s + 1], x[s + 1]);

        // cout << "OF: " << of << endl;
        // getchar();
    }

    for (int s = 0; s < S + 1; s++)
    {
        sol_y.push_back(vector<pair<int, int>>());
        sol_x.push_back(vector<pair<int, int>>());
#ifndef Silence
        cout << "Scenario " << s << endl;
#endif
        for (auto b : y[s])
        {
#ifndef Silence
            cout << "Y: " << b << endl;
#endif
        }

        for (auto i : x[s])
            if (i.first > graph->getN())
                cout << "X: " << i.first - 1 << " - " << i.second << endl;
            else
            {
                sol_x[s].push_back({i.second, i.first});
#ifndef Silence
                cout << "X: " << i.second << " - " << i.first << endl;
#endif
            }
#ifndef Silence
        cout << "--------------------------" << endl;
#endif
    }

    return of;
}