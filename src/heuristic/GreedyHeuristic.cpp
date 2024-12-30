#include "GreedyHeuristic.hpp"

double GreedyHeuristic::SolveScenario(
    vector<double> cases,
    vector<int> time,
    double route_time_increase,
    int max_tries,
    int T,
    vector<int> &y,
    vector<int_pair> &x)
{
    // Start greedy heuristic
    Graph *graph = input->getGraph();
    BlockConnection *bc = input->getBlockConnection();
    int available_time = T, tries = 1;
    double of = 0;

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
            {
                // cout << "X: " << vertices[j] << " " << vertices[j + 1] << endl;
                x.push_back({vertices[j], vertices[j + 1]});
            }
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

Solution GreedyHeuristic::Run(double route_time_increase, int max_tries, bool use_avg)
{
    // Get all blocks
    Graph *graph = input->getGraph();
    int S = input->getS(), T = input->getT(), B = graph->getB();
    vector<int> blocks = vector<int>(B, 0), time = vector<int>(B, 0);
    vector<double> cases = vector<double>(B, 0);
    vector<bool> in_first_stage = vector<bool>(B, false);
    double alpha = input->getAlpha();
    double of = 0;

    // Solve First Stage
    vector<double> real_cases = vector<double>(B, 0);
    for (int i = 0; i < B; i++)
    {
        blocks[i] = i;
        time[i] = graph->getTimePerBlock(i);
        cases[i] = graph->getCasesPerBlock(i);

        if (use_avg)
        {
            pair<double, double> values = getBlockSecondStageProfitAvg(input->getScenarios(), i);
            real_cases[i] = cases[i] + values.second;
            cases[i] += values.first;
        }
        else
            cases[i] += getBlockSecondStageProfitSum(input->getScenarios(), i);
    }

    vector<vector<int>> y = vector<vector<int>>(S + 1, vector<int>());
    vector<vector<int_pair>> x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
    of += SolveScenario(cases, time, route_time_increase, max_tries, input->getT(), y[0], x[0]);

    if (use_avg)
        of = getRealValueOfFirstStageSolution(y[0], real_cases);

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
        }

        if (!all_zeros)
            of += input->getScenario(s).getProbability() * SolveScenario(cases, time, route_time_increase, max_tries, y[s + 1], x[s + 1]);
    }

    return Solution(of, y, x);
}