//
// Created by carlos on 05/01/25
//

#include "LocalSearch.hpp"

double LocalSearch::RunDefaultPerturbation(vector<pair<int, int_pair>> &swaps)
{
    // Best In Route Swaps
    double delta = ComputeInRouteSwapBlocksStartScenario(this->delta_type, swaps, this->use_first_improve);
    if (swaps.size() > 0)
        return delta;
}

void LocalSearch::RunInRouteSwapImprove(string delta_type, bool is_first_improve)
{
    bool is_stuck = false;
    vector<pair<int, int_pair>> best_swaps;
    Graph *graph = input->getGraph();

#ifndef Silence
    cout << "[*] Run Best Improve 2OPT!" << endl;
    cout << "[*] Start OF is matching? " << solution->getOf() << " == " << solution->ComputeCurrentSolutionOF() << endl;
#endif

    while (!is_stuck)
    {
        double delta = LocalSearch::ComputeInRouteSwapBlocksStartScenario(delta_type, best_swaps, is_first_improve);

        if (best_swaps.size() > 0)
        {
            for (auto scenario_swap : best_swaps)
            {
                int_pair swap = scenario_swap.second;
                int scenario = scenario_swap.first, b1 = swap.first, b2 = swap.second;

                if (b1 == -1 || b2 == -1)
                {
                    is_stuck = true;
                    break;
                }

#ifndef Silence
                cout << "[**] Best swap in scenario " << scenario << " = " << swap.first << " " << swap.second << endl;
#endif

                solution->ScenarioBlockSwapWithoutOF(scenario, b1, b2);
            }

            solution->setOf(solution->getOf() + delta);

#ifndef Silence
            cout << "[*] Updated OF: " << solution->getOf() << endl;
            cout << "[=] OF matching? " << solution->ComputeCurrentSolutionOF() << endl;
#endif
        }
        else
        {
#ifndef Silence
            cout << "[!] No more profitable swap found!" << endl;
#endif
            is_stuck = true;
        }
    }
};

double LocalSearch::GetWeakDeltaSwapBlocksStartScenario(int b1, int b2)
{
    Graph *graph = input->getGraph();
    double delta = 0.0, cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
    double alpha = input->getAlpha(), prob, rest = 1.0 - alpha;
    int S = input->getS();

    // First Stage delta
    delta = cases_b2 - cases_b1;

    for (int s = 1; s <= S; s++)
    {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        double cases_b1 = scenario->getCasesPerBlock(b1), cases_b2 = scenario->getCasesPerBlock(b2);

        // Update first stage change
        delta += prob * alpha * (cases_b2 - cases_b1);

        if (s_route->IsBlockAttended(b1))
            delta += alpha * prob * cases_b1;
        if (s_route->IsBlockAttended(b2))
            delta -= alpha * prob * cases_b2;
    }

    return delta;
};

int LocalSearch::GetBestSecondStageOptionIfB1LeaveFSSolution(Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1, int b2)
{
    double block_profit, best_profit = LocalSearch::GetUpdatedSecondStageCases(scenario, b1, false);
    int best_block = -1;

    for (auto block : s_stage_route->getBlocks())
    {
        if (block == b1 || block == b2)
            continue;

        bool is_block_attended_fs = f_stage_route->IsBlockAttended(block),
             is_block_attended_ss = s_stage_route->IsBlockAttended(block);

        if (!is_block_attended_ss || !s_stage_route->IsSwapTimeLowerThanT(b1, block))
            continue;

        block_profit = LocalSearch::GetUpdatedSecondStageCases(scenario, block, is_block_attended_fs);

        if (block_profit < best_profit)
            best_block = block, best_profit = block_profit;
    }
    return best_block;
}

int LocalSearch::GetBestSecondStageOptionIfB2EnterFSSolution(Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1, int b2, int changed)
{
    double best_profit = GetUpdatedSecondStageCases(scenario, b2, true);
    int best_block = -1;

    for (auto block : s_stage_route->getBlocks())
    {
        if (block == b1 || block == b2 || scenario->getCasesPerBlock(block) <= 0 || block == changed)
            continue;

        bool is_block_attended_fs = f_stage_route->IsBlockAttended(block),
             is_block_attended_ss = s_stage_route->IsBlockAttended(block);

        if (is_block_attended_ss || !s_stage_route->IsSwapTimeLowerThanT(b2, block))
            continue;

        double profit_block = GetUpdatedSecondStageCases(scenario, block, is_block_attended_fs);

        if (profit_block > best_profit)
            best_block = block, best_profit = profit_block;
    }

    return best_block;
}

double LocalSearch::GetModerateDeltaSwapBlocksStartScenario(int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps)
{
    Graph *graph = input->getGraph();
    Route *first_stage_route = solution->getRouteFromScenario(0);

    double cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2), alpha = input->getAlpha(), prob;
    int S = input->getS(), lowest_in_block = -1, highest_block = -1;
    double delta = cases_b2 - cases_b1;
    second_stage_swaps = vector<pair<int, int_pair>>();

    for (int s = 1; s <= S; s++)
    {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        double cases_b1 = scenario->getCasesPerBlock(b1), cases_b2 = scenario->getCasesPerBlock(b2);

        // Update first stage change
        delta += prob * alpha * (cases_b2 - cases_b1);

        int changed = -1;

        // B1 leaving the First Stage Solution
        if (s_route->IsBlockAttended(b1))
            delta += alpha * prob * cases_b1;
        else if (s_route->IsBlockInRoute(b1) && cases_b1 > 0)
        {
            lowest_in_block = LocalSearch::GetBestSecondStageOptionIfB1LeaveFSSolution(scenario, first_stage_route, s_route, b1, b2);

            if (lowest_in_block != -1)
            {
                bool attended_fs = first_stage_route->IsBlockAttended(lowest_in_block);
                delta += prob * (cases_b1 - GetUpdatedSecondStageCases(scenario, lowest_in_block, attended_fs));
                second_stage_swaps.push_back(make_pair(s, make_pair(lowest_in_block, b1)));
            }
        }

        if (s_route->IsBlockAttended(b2))
        {
            highest_block = LocalSearch::GetBestSecondStageOptionIfB2EnterFSSolution(scenario, first_stage_route, s_route, b1, b2, changed);

            if (highest_block != -1)
            {
                bool attended_fs = first_stage_route->IsBlockAttended(highest_block);

                delta += prob * (GetUpdatedSecondStageCases(scenario, highest_block, attended_fs) - GetUpdatedSecondStageCases(scenario, b2, false));
                second_stage_swaps.push_back(make_pair(s, make_pair(b2, highest_block)));
            }
            else
                delta -= alpha * prob * cases_b2;
        }
    }
    return delta;
};

double LocalSearch::ComputeInRouteSwapBlocksStartScenario(string delta_type, vector<pair<int, int_pair>> &best_swaps, bool is_first_improve)
{
    Route *route = solution->getRouteFromScenario(0);
    double best = 0.0, delta = 0.0;

    set<int> set_blocks = route->getBlocks();
    vector<int> blocks = vector<int>(set_blocks.begin(), set_blocks.end());

    best_swaps = vector<pair<int, int_pair>>();
    vector<pair<int, int_pair>> curr_swaps;

    int i, j, b1, b2, best_b1 = -1, best_b2 = -1;

    for (i = 0; i < blocks.size(); i++)
    {
        b1 = blocks[i];

        // If b1 is not attended, continue
        if (!route->IsBlockAttended(b1))
            continue;

        for (j = 0; j < blocks.size(); j++)
        {
            b2 = blocks[j];

            // If b2 is attended, continue
            if (i == j || route->IsBlockAttended(b2) || !route->IsSwapFeasible(b1, b2))
                continue;

            delta = 0.0;
            if (delta_type == "weak")
                delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
            else if (delta_type == "moderate")
                delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, curr_swaps);

            if (delta > best)
            {
                best = delta, best_b1 = b1, best_b2 = b2;
                if (delta_type != "weak")
                    best_swaps = curr_swaps;
                if (is_first_improve)
                {
                    best_swaps.push_back(make_pair(0, make_pair(best_b1, best_b2)));
                    return delta;
                }
            }
        }
    }

    if (best_b1 == -1 || best_b2 == -1)
        return 0.0;

    best_swaps.push_back(make_pair(0, make_pair(best_b1, best_b2)));
    return best;
}

double LocalSearch::ComputeOutRouteSwapBlocksStartScenario(string delta_type, vector<pair<int, int_pair>> &best_swaps, bool is_first_improve)
{
    Graph *graph = input->getGraph();
    Route *route = solution->getRouteFromScenario(0);
    set<int> set_blocks = route->getBlocks();
    vector<int> route_blocks = vector<int>(set_blocks.begin(), set_blocks.end());
    best_swaps = vector<pair<int, int_pair>>();
    vector<pair<int, int_pair>> curr_swaps;
    int i, j, b1, b2, best_b1 = -1, best_b2 = -1;
    double best = 0.0, delta = 0.0;

    for (j = 0; j < route_blocks.size(); j++)
    {
        b1 = route_blocks[j];

        // If b1 is not attended, continue
        if (!route->IsBlockAttended(b1))
            continue;

        for (b2 = 0; b2 < graph->getB(); b2++)
        {
            // If b2 is attended, continue
            if (b1 == b2 || route->IsBlockInRoute(b2) || !route->IsSwapFeasible(b1, b2))
                continue;

            delta = 0.0;
            if (delta_type == "weak")
                delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
            else if (delta_type == "moderate")
                delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, curr_swaps);

            if (delta > best)
            {
                best = delta, best_b1 = b1, best_b2 = b2;
                if (delta_type != "weak")
                    best_swaps = curr_swaps;
                if (is_first_improve)
                {
                    best_swaps.push_back(make_pair(0, make_pair(best_b1, best_b2)));
                    return delta;
                }
            }
        }
    }

    if (best_b1 == -1 || best_b2 == -1)
        return 0.0;

    best_swaps.push_back(make_pair(0, make_pair(best_b1, best_b2)));
    return best;
}

int_pair LocalSearch::GetRandomB1NB2(Route *route, vector<int> &blocks)
{
    Utils::FastShuffle(blocks);
    int b1 = -1, b2 = -1;
    for (int b : blocks)
    {
        if (b1 == -1 && route->IsBlockAttended(b))
            b1 = b;
        else if (b2 == -1 && !route->IsBlockAttended(b))
            b2 = b;

        if (b1 != -1 && b2 != -1)
            break;
    }
    return make_pair(b1, b2);
}

double LocalSearch::ComputeInRouteRandomSwapBlocksStartScenario(string delta_type, vector<pair<int, int_pair>> &best_swaps)
{
    Graph *graph = input->getGraph();
    Route *route = solution->getRouteFromScenario(0);
    double delta = 0.0;

    set<int> set_blocks = route->getBlocks();
    vector<int> blocks = vector<int>(set_blocks.begin(), set_blocks.end());

    int_pair b1nb2 = GetRandomB1NB2(route, blocks);
    int b1 = b1nb2.first, b2 = b1nb2.second;

    cout << "[*] Random swap " << b1 << " " << b2 << endl;

    delta = 0.0;
    if (delta_type == "weak")
        delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
    else if (delta_type == "moderate")
        delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, best_swaps);

    best_swaps.push_back(make_pair(0, make_pair(b1, b2)));
    cout << "[*] Delta: " << delta << endl;
    return delta;
}

Route *LocalSearch::RemoveBlockFromRoute(int route_idx, int block)
{
    Route *route = solution->getRouteFromScenario(route_idx);
    try
    {
        route->RemoveBlockFromRoute(block);
    }
    catch (std::runtime_error &e)
    {
        return route;
    }
    return route;
}

void LocalSearch::InsertOutRouteBlock(int route_idx, int block)
{
}
