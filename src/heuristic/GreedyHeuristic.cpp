
#include "GreedyHeuristic.hpp"

GreedyHeuristic::GreedyHeuristic(Input *input) {
    this->input = input;
    this->objective_value = 0;
}

double GreedyHeuristic::SolveScenario(const vector<double> &cases, const vector<int> &time, int T, vector<int> &y) {
    // Start greedy heuristic
    int available_time_to_attend = T;
    double lb = 0.5, ub = 1.0, mid = 0.0, of = 0, temp_of;

    vector<int> temp_y;

    // Optimal solution = (1.0 * T)
    temp_of = BinarySolve(cases, time, available_time_to_attend, T, temp_y);

    if (temp_of != -1) {
        y = temp_y;
        return temp_of;
    }

    // LB solution = (0.5 * T)
    available_time_to_attend = round(double(T) * lb);
    temp_of = BinarySolve(cases, time, available_time_to_attend, T, temp_y);

    if (temp_of == -1)
        ub = lb, lb = 0.0;
    else
        of = temp_of, y = temp_y;

    mid = (lb + ub) / 2.0;

    while ((ub - lb) > 0.001) {
        available_time_to_attend = round(double(T) * mid);
        temp_of = BinarySolve(cases, time, available_time_to_attend, T, temp_y);

        if (temp_of == -1)
            ub = mid;
        else
            of = temp_of, y = temp_y, lb = mid;
        mid = (lb + ub) / 2.0;
    }

    return of;
}

double GreedyHeuristic::BinarySolve(const vector<double> &cases, const vector<int> &time, int reserved_time, int T, vector<int> &y) {
    // Start greedy heuristic
    Graph *graph = input->getGraph();
    BlockConnection *bc = input->getBlockConnection();
    double of = -1;

    y = vector<int>();
    of = Knapsack::Run(y, cases, time, reserved_time);

    if (y.empty())
        return -1;

    // Get attend time
    int block_attended_time = 0;
    for (auto b : y)
        block_attended_time += graph->getTimePerBlock(b);

    // Generate a hash to solution
    string key = BlockConnection::GenerateStringFromIntVector(y);

    // Get route cost
    int connection_cost = T + 1;
    if (!bc->keyExists(key)) {
        connection_cost = bc->HeuristicBlockConnection(graph, input->getShortestPath(), y, key);
    } else
        connection_cost = bc->getBlocksAttendCost(key);

    if (block_attended_time + connection_cost <= T)
        return of;
    return -1;
}

Solution GreedyHeuristic::Run(double route_time_increase, int max_tries, bool use_avg) {
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
    for (int i = 0; i < B; i++) {
        blocks[i] = i;
        time[i] = graph->getTimePerBlock(i);
        cases[i] = graph->getCasesPerBlock(i);

        if (use_avg) {
            pair<double, double> values = getBlockSecondStageProfitAvg(input->getScenarios(), i);
            real_cases[i] = cases[i] + values.second;
            cases[i] += values.first;
        } else
            cases[i] += getBlockSecondStageProfitSum(input->getScenarios(), i);
    }

    vector<vector<int>> y = vector<vector<int>>(S + 1, vector<int>());
    vector<vector<int_pair>> x = vector<vector<int_pair>>(S + 1, vector<int_pair>());
    of += SolveScenario(cases, time, input->getT(), y[0]);

    if (use_avg)
        of = getRealValueOfFirstStageSolution(y[0], real_cases);

    // Solve Second Stage
    for (auto i : y[0])
        in_first_stage[i] = true;

    for (int s = 0; s < S; s++) {
        bool all_zeros = true;
        for (int i = 0; i < B; i++) {
            cases[i] = input->getScenario(s)->getCasesPerBlock(i);
            if (in_first_stage[i])
                cases[i] *= (1 - alpha);

            if (cases[i] > 0)
                all_zeros = false;
        }

        if (!all_zeros)
            of += input->getScenario(s)->getProbability() * SolveScenario(cases, time, T, y[s + 1]);
    }

    return {of, y, x, input};
}
