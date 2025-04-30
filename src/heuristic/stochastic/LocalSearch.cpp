//
// Created by carlos on 05/01/25
//

#include "LocalSearch.hpp"
#include <algorithm>
#include <cstdlib>
#include <utility>

Change LocalSearch::RunDefaultPerturbation(bool use_random) {
    cout << "[!] Running default perturbation" << endl;
    Change change;

    cout << "[!] Try InRoute Swaps" << endl;
    // Best InRoute Swaps
    change = ComputeSwapBlocks(false);
    if (!ChangeUtils::isEmpty(change))
        return change;

    cout << "[!] Try OutRoute Swaps" << endl;
    // Best OutRoute Swap
    change = ComputeSwapBlocks(true);
    if (!ChangeUtils::isEmpty(change))
        return change;

    cout << "[!] Try Random Block Changes" << endl;
    // Ramdom Block Change
    change = RandomBlockChange();
    if (!ChangeUtils::isEmpty(change))
        return change;

    return ChangeUtils::createEmptyChange();
}

Change LocalSearch::RandomBlockChange() {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distrib(0, 2);
    int rand_option = distrib(gen);

    // TODO: generalize the options for Remove and Insert
    if (rand_option == 0) {
        return SelectRandomSwapBlocks();
    } else if (rand_option == 1) {
        return SelectRemoveBlock(3);
    } else {
        return SelectInsertBlock(3);
    }
}

int LocalSearch::SelectRandomInsertBlock(bool try_improve_route) {
    if (try_improve_route)
        ImproveRouteTime();

    Route *route = solution->getRouteFromScenario(0);
    Graph *graph = input->getGraph();
    int B = graph->getB();

    vector<int> blocks = vector<int>();
    blocks.reserve(B);
    for (int b = 0; b < B; b++) {
        if (!route->IsBlockAttended(b) && input->getFirstStageProfit(b) > 0 && route->IsBlockInsertionFactible(b)) {
            blocks.push_back(b);
        }
    }

    if (blocks.size() <= 0)
        return -1;

    Utils::FastShuffle(blocks);
    return blocks[0];
}

double LocalSearch::GetWeakDeltaInsertBlock(int block) {
    double delta = input->getFirstStageProfit(block);

    for (int s = 1; s <= input->getS(); s++) {
        Route *s_route = solution->getRouteFromScenario(s);
        if (s_route->IsBlockAttended(block))
            delta -= input->getSecondStageProfit(s - 1, block);
    }

    return delta;
}

double LocalSearch::GetWeakDeltaRemoveBlock(int block) {
    Route *first_stage_route = solution->getRouteFromScenario(0);
    if (!first_stage_route->IsBlockAttended(block))
        return 0.0;

    double delta = -input->getFirstStageProfit(block);
    for (int s = 1; s <= input->getS(); s++) {
        Route *s_route = solution->getRouteFromScenario(s);
        if (s_route->IsBlockAttended(block))
            delta += input->getSecondStageProfit(s - 1, block);
    }

    return delta;
}

double LocalSearch::GetModerateDeltaRemoveBlock(int block, vector<pair<int, int_pair>> &second_stage_swaps) {
    Route *first_stage_route = solution->getRouteFromScenario(0);
    if (!first_stage_route->IsBlockAttended(block))
        return 0.0;

    double delta = -input->getFirstStageProfit(block), cases_bs, prob;
    int lowest_in_block = -1, S = input->getS();
    second_stage_swaps = vector<pair<int, int_pair>>();
    second_stage_swaps.reserve(S);

    for (int s = 1; s <= S; s++) {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        cases_bs = scenario->getCasesPerBlock(block);

        if (s_route->IsBlockAttended(block))
            delta += input->getSecondStageProfit(s - 1, block);
        else if (s_route->IsBlockInRoute(block) && cases_bs > 0) {
            lowest_in_block = GetBestSecondStageOptionIfB1LeaveFSSolution(scenario, first_stage_route, s_route, block, block);

            if (lowest_in_block != -1) {
                bool attended_fs = first_stage_route->IsBlockAttended(lowest_in_block);
                delta += prob * (cases_bs - GetUpdatedSecondStageCases(scenario, lowest_in_block, attended_fs));
                second_stage_swaps.emplace_back(s, make_pair(lowest_in_block, block));
            }
        }
    }
    return delta;
}

double LocalSearch::GetModerateDeltaInsertBlock(int block, vector<pair<int, int_pair>> &second_stage_swaps) {
    Route *first_stage_route = solution->getRouteFromScenario(0);
    double prob, cases_bs, delta = input->getFirstStageProfit(block);
    int S = input->getS(), highest_block = -1;
    bool attended_fs;
    second_stage_swaps = vector<pair<int, int_pair>>();
    second_stage_swaps.reserve(S);

    for (int s = 1; s <= S; s++) {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        cases_bs = scenario->getCasesPerBlock(block);

        if (s_route->IsBlockAttended(block)) {
            highest_block = GetBestSecondStageOptionIfB2EnterFSSolution(scenario, first_stage_route, s_route, block, block, -1);

            if (highest_block != -1) {
                attended_fs = first_stage_route->IsBlockAttended(highest_block);
                delta += prob * (GetUpdatedSecondStageCases(scenario, highest_block, attended_fs) - GetUpdatedSecondStageCases(scenario, block, false));
                second_stage_swaps.emplace_back(s, make_pair(block, highest_block));
            }
        }
    }
    return delta;
}

double LocalSearch::GetWeakDeltaSwapBlocksStartScenario(int b1, int b2) {
    Graph *graph = input->getGraph();
    double delta = 0.0, cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
    double alpha = input->getAlpha(), prob;

    // First Stage delta
    delta = cases_b2 - cases_b1;

    for (int s = 1; s <= input->getS(); s++) {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        cases_b1 = scenario->getCasesPerBlock(b1), cases_b2 = scenario->getCasesPerBlock(b2);

        // Update first stage change
        delta += prob * alpha * (cases_b2 - cases_b1);

        if (s_route->IsBlockAttended(b1))
            delta += alpha * prob * cases_b1;
        if (s_route->IsBlockAttended(b2))
            delta -= alpha * prob * cases_b2;
    }

    return delta;
};

int LocalSearch::GetBestSecondStageOptionIfB1LeaveFSSolution(
    const Scenario *scenario,
    const Route *f_stage_route,
    const Route *s_stage_route,
    int b1, int b2) {

    double block_profit, best_profit = LocalSearch::GetUpdatedSecondStageCases(scenario, b1, false);
    int best_block = -1;

    for (auto block : s_stage_route->getBlocks()) {
        if (block == b1 || block == b2)
            continue;

        bool is_block_attended_fs = f_stage_route->IsBlockAttended(block),
             is_block_attended_ss = s_stage_route->IsBlockAttended(block);

        if (!is_block_attended_ss ||
            !s_stage_route->IsSwapTimeLowerThanT(b1, block))
            continue;

        block_profit = LocalSearch::GetUpdatedSecondStageCases(scenario, block, is_block_attended_fs);

        if (block_profit < best_profit)
            best_block = block, best_profit = block_profit;
    }
    return best_block;
}

int LocalSearch::GetBestSecondStageOptionIfB2EnterFSSolution(Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1, int b2, int changed) {
    double best_profit = GetUpdatedSecondStageCases(scenario, b2, true);
    int best_block = -1;

    for (auto block : s_stage_route->getBlocks()) {
        if (block == b1 || block == b2 || scenario->getCasesPerBlock(block) <= 0 ||
            block == changed)
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

double LocalSearch::GetModerateDeltaSwapBlocksStartScenario(
    int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps) {
    Graph *graph = input->getGraph();
    Route *first_stage_route = solution->getRouteFromScenario(0);

    double cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2), alpha = input->getAlpha(),
           prob;
    int S = input->getS(), lowest_in_block = -1, highest_block = -1;
    double delta = cases_b2 - cases_b1;
    second_stage_swaps = vector<pair<int, int_pair>>();

    for (int s = 1; s <= S; s++) {
        Scenario *scenario = input->getScenario(s - 1);
        Route *s_route = solution->getRouteFromScenario(s);
        prob = scenario->getProbability();
        cases_b1 = scenario->getCasesPerBlock(b1),
        cases_b2 = scenario->getCasesPerBlock(b2);

        // Update first stage change
        delta += prob * alpha * (cases_b2 - cases_b1);

        int changed = -1;

        // B1 leaving the First Stage Solution
        if (s_route->IsBlockAttended(b1))
            delta += alpha * prob * cases_b1;
        else if (s_route->IsBlockInRoute(b1) && cases_b1 > 0) {
            lowest_in_block =
                LocalSearch::GetBestSecondStageOptionIfB1LeaveFSSolution(
                    scenario, first_stage_route, s_route, b1, b2);

            if (lowest_in_block != -1) {
                bool attended_fs = first_stage_route->IsBlockAttended(lowest_in_block);
                delta +=
                    prob * (cases_b1 - GetUpdatedSecondStageCases(
                                           scenario, lowest_in_block, attended_fs));
                second_stage_swaps.emplace_back(s, make_pair(lowest_in_block, b1));
            }
        }

        if (s_route->IsBlockAttended(b2)) {
            highest_block = LocalSearch::GetBestSecondStageOptionIfB2EnterFSSolution(
                scenario, first_stage_route, s_route, b1, b2, changed);

            if (highest_block != -1) {
                bool attended_fs = first_stage_route->IsBlockAttended(highest_block);

                delta += prob * (GetUpdatedSecondStageCases(scenario, highest_block,
                                                            attended_fs) -
                                 GetUpdatedSecondStageCases(scenario, b2, false));
                second_stage_swaps.emplace_back(s, make_pair(b2, highest_block));
            } else
                delta -= alpha * prob * cases_b2;
        }
    }
    return delta;
};

Change LocalSearch::ComputeSwapBlocks(bool is_out_route) {
    Graph *graph = input->getGraph();
    Route *route = solution->getRouteFromScenario(0);
    double best = 0.0, delta = 0.0;
    set<int> route_blocks = route->getBlocks();
    vector<pair<int, int_pair>> curr_swaps, best_swaps;
    int best_b1 = -1, best_b2 = -1;

    for (int b1 : route_blocks) {
        // If b1 is not attended, continue
        if (!route->IsBlockAttended(b1))
            continue;

        if (is_out_route) {
            for (int b2 = 0; b2 < graph->getB(); b2++) {
                // If b2 is attended, continue
                if (b1 == b2 || this->input->getFirstStageProfit(b2) <= 0 ||
                    route->IsBlockInRoute(b2) || !route->IsOutSwapFeasible(b1, b2))
                    continue;

                delta = 0.0;
                if (this->delta_type == "weak")
                    delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
                else if (this->delta_type == "moderate")
                    delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, curr_swaps);

                if (delta > best) {
                    best = delta, best_b1 = b1, best_b2 = b2;
                    if (this->delta_type != "weak")
                        best_swaps = curr_swaps;
                    if (this->use_first_improve) {
                        curr_swaps.emplace_back(0, make_pair(best_b1, best_b2));
                        return ChangeUtils::newChange(delta, curr_swaps);
                    }
                }
            }
        } else {
            for (int b2 : route_blocks) {
                // If b2 is attended, continue
                if (b1 == b2 || this->input->getFirstStageProfit(b2) <= 0 ||
                    route->IsBlockAttended(b2) || !route->IsSwapFeasible(b1, b2))
                    continue;

                delta = 0.0;
                if (this->delta_type == "weak")
                    delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
                else if (this->delta_type == "moderate")
                    delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, curr_swaps);

                if (delta > best) {
                    best = delta, best_b1 = b1, best_b2 = b2;
                    if (this->delta_type != "weak")
                        best_swaps = curr_swaps;
                    if (this->use_first_improve) {
                        curr_swaps.emplace_back(0, make_pair(best_b1, best_b2));
                        return ChangeUtils::newChange(delta, curr_swaps);
                    }
                }
            }
        }
    }

    if (best_b1 == -1 || best_b2 == -1)
        return ChangeUtils::createEmptyChange();

    best_swaps.emplace_back(0, make_pair(best_b1, best_b2));
    return ChangeUtils::newChange(best, best_swaps);
}

Change LocalSearch::ComputeOutRouteSwapBlocksStartScenario() {
    Graph *graph = input->getGraph();
    Route *route = solution->getRouteFromScenario(0);
    set<int> route_blocks = route->getBlocks();

    vector<pair<int, int_pair>> curr_swaps = vector<pair<int, int_pair>>(),
                                best_swaps = vector<pair<int, int_pair>>();
    curr_swaps.reserve(1), best_swaps.reserve(1);

    int j, b2, best_b1 = -1, best_b2 = -1;
    double best = 0.0, delta = 0.0;

    for (int b1 : route_blocks) {
        // If b1 is not attended, continue
        if (!route->IsBlockAttended(b1))
            continue;

        for (b2 = 0; b2 < graph->getB(); b2++) {
            // If b2 is attended, continue
            if (b1 == b2 ||
                this->input->getFirstStageProfit(b2) <= 0 ||
                route->IsBlockInRoute(b2) ||
                !route->IsOutSwapFeasible(b1, b2))
                continue;

            delta = 0.0;
            if (this->delta_type == "weak")
                delta = GetWeakDeltaSwapBlocksStartScenario(b1, b2);
            else if (this->delta_type == "moderate")
                delta = GetModerateDeltaSwapBlocksStartScenario(b1, b2, curr_swaps);

            if (delta > best) {
                best = delta, best_b1 = b1, best_b2 = b2;
                if (this->delta_type != "weak")
                    best_swaps = curr_swaps;
                if (this->use_first_improve) {
                    Change change = ChangeUtils::newChange(best);
                    ChangeUtils::addSwap(change, 0, best_b1, best_b2);
                    return change;
                }
            }
        }
    }

    if (best_b1 == -1 || best_b2 == -1)
        return ChangeUtils::createEmptyChange();

    Change change = ChangeUtils::newChange(best);
    ChangeUtils::addSwap(change, 0, best_b1, best_b2);
    return change;
}

int_pair LocalSearch::GetRandomB1NB2(const Route *route, vector<int> &blocks) {
    Utils::FastShuffle(blocks);
    int b1 = -1, b2 = -1;

    for (int b : blocks) {
        if (route->IsBlockAttended(b))
            b1 = b;
        else if (!route->IsBlockAttended(b))
            b2 = b;

        if (b1 != -1 && b2 != -1)
            break;
    }

    return make_pair(b1, b2);
}

int_pair LocalSearch::GetRandomBlocksFeasibleSwap(Route *route) {
    Graph *graph = solution->getGraph();
    int B = graph->getB();

    // Copy Attended blocks
    set<int> r_blocks = route->getBlocks();
    vector<int> route_blocks;
    route_blocks.reserve(r_blocks.size());
    for (int b : r_blocks)
        if (route->IsBlockAttended(b))
            route_blocks.push_back(b);

    // Copy Unattended blocks
    vector<int> other_blocks;
    other_blocks.reserve(B);
    for (int b = 0; b < B; b++)
        if (!route->IsBlockAttended(b))
            other_blocks.push_back(b);

    Utils::FastShuffle(route_blocks);
    Utils::FastShuffle(other_blocks);

    int b1 = -1, b2 = -1;
    bool is_feasible, is_in_route_swap;
    for (int b_insert : other_blocks) {
        is_in_route_swap = route->IsBlockInRoute(b_insert);

        for (int b_remove : route_blocks) {
            if (is_in_route_swap) {
                is_feasible = route->IsSwapFeasible(b_remove, b_insert);
            } else {
                is_feasible = route->IsOutSwapFeasible(b_remove, b_insert);
            }

            if (is_feasible) {
                b1 = b_remove, b2 = b_insert;
                break;
            }
        }
    }

    return make_pair(b1, b2);
}

Change LocalSearch::SelectRandomSwapBlocks() {
    cout << "\t[*] SelectRandomSwapBlocks" << endl;
    Route *route = solution->getRouteFromScenario(0);
    int_pair b1nb2 = GetRandomBlocksFeasibleSwap(route);
    int to_remove = b1nb2.first, to_insert = b1nb2.second;

    if (to_remove == to_insert)
        return ChangeUtils::createEmptyChange();

    double delta = 0.0;
    vector<pair<int, int_pair>> swaps;
    swaps.reserve(this->input->getS() + 1);

    if (this->delta_type == "weak")
        delta = GetWeakDeltaSwapBlocksStartScenario(to_remove, to_insert);
    else
        delta = GetModerateDeltaSwapBlocksStartScenario(to_remove, to_insert, swaps);

    swaps.emplace_back(0, make_pair(to_remove, to_insert));
    return ChangeUtils::newChange(delta, swaps);
}

// 0 = HighestTime, 1 = lowestProfit, 2 = lowestProportion, 3 = random
Change LocalSearch::SelectRemoveBlock(int option) {
    cout << "\t[*] SelectRemoveBlock" << endl;
    int to_remove_b = -1;
    double delta;
    if (option <= 0) {
        to_remove_b = SelectTopTimeBlock(false, true);
    } else if (option <= 1) {
        to_remove_b = SelectTopProfitBlock(true, true);
    } else if (option <= 2) {
        to_remove_b = SelectTopProportionBlock(true, true);
    } else
        to_remove_b = SelectRandomRemoveBlock();

    vector<pair<int, int_pair>> second_stage_swaps;
    if (this->delta_type == "weak")
        delta = GetWeakDeltaRemoveBlock(to_remove_b);
    else
        delta = GetModerateDeltaRemoveBlock(to_remove_b, second_stage_swaps);

    Change change = ChangeUtils::newChange(delta, second_stage_swaps);
    ChangeUtils::addDeletion(change, 0, to_remove_b);
    return change;
}

// 0 = LowestTime, 1 = HighestProfit, 2 = HighestProportion, 3 = Random
Change LocalSearch::SelectInsertBlock(int option) {
    cout << "\t[*] SelectInsertBlock" << endl;
    int to_insert_b = -1;
    if (option <= 0) {
        to_insert_b = SelectTopTimeBlock(true, false);
    } else if (option <= 1) {
        to_insert_b = SelectTopProfitBlock(false, false);
    } else if (option <= 2) {
        to_insert_b = SelectTopProportionBlock(false, false);
    }

    if (to_insert_b == -1)
        to_insert_b = SelectRandomInsertBlock(true);

    if (to_insert_b == -1)
        return ChangeUtils::createEmptyChange();

    double delta;
    vector<pair<int, int_pair>> second_stage_swaps;
    if (this->delta_type == "weak")
        delta = GetWeakDeltaInsertBlock(to_insert_b);
    else
        delta = GetModerateDeltaInsertBlock(to_insert_b, second_stage_swaps);

    Change change = ChangeUtils::newChange(delta, second_stage_swaps);
    ChangeUtils::addInsertion(change, 0, to_insert_b);
    return change;
}

Route *LocalSearch::RemoveBlockFromRoute(int route_idx, int block) {
    Route *route = solution->getRouteFromScenario(route_idx);
    try {
        route->RemoveBlockFromRoute(block);
    } catch (std::runtime_error &e) {
        return route;
    }
    return route;
}

void LocalSearch::InsertOutRouteBlock(int route_idx, int block) {}

void LocalSearch::RunInRouteSwapImprove() {
    bool is_stuck = false;
    vector<pair<int, int_pair>> best_swaps;
    cout << "[*] Run Best Improve 2OPT!" << endl;
    cout << "[*] Start OF is matching? " << solution->getOf()
         << " == " << solution->ComputeCurrentSolutionOF() << endl;

    while (!is_stuck) {
        Change change = LocalSearch::ComputeSwapBlocks(false);

        if (!ChangeUtils::isEmpty(change)) {
            for (auto scenario_swap : change.swaps) {
                int_pair swap = scenario_swap.second;
                int scenario = scenario_swap.first, b1 = swap.first, b2 = swap.second;
                cout << "[**] Best swap in scenario " << scenario << " = " << swap.first
                     << " " << swap.second << endl;
                solution->ScenarioBlockSwapWithoutOF(scenario, b1, b2);
            }

            solution->setOf(solution->getOf() + change.delta);

            cout << "[*] Updated OF: " << solution->getOf() << endl;
            cout << "[=] OF matching? " << solution->ComputeCurrentSolutionOF()
                 << endl;
        } else {
            cout << "[!] No more profitable swap found!" << endl;
            is_stuck = true;
        }
    }
};

void LocalSearch::ImproveRouteTime() {
    Route *first_stage_route = this->solution->getRouteFromScenario(0);
    vector<int> route = first_stage_route->getRoute();
    bool has_improved = false;
    int time_diff;

    while (true) {
        time_diff = ApplyNodeSwap(route);
        if (time_diff <= 0)
            break;

        has_improved = true;
        first_stage_route->ChangeRouteTime(-time_diff);
    }

    if (has_improved) {
        // Update route
        first_stage_route->setRoute(route);
        set<int> route_blocks = first_stage_route->getRouteBlocks();
        vector<int> att_blocks;
        att_blocks.reserve(route_blocks.size());

        // Get only attended blocks
        for (int b : route_blocks)
            if (first_stage_route->IsBlockAttended(b))
                att_blocks.push_back(b);

        // Generate key
        BlockConnection *bc = input->getBlockConnection();
        bc->UpdateBestKnowBlockConnection(att_blocks, route, first_stage_route->getTimeRoute());
    }
}

int LocalSearch::ApplyNodeSwap(vector<int> &route) {
    int i, j, N = int(route.size()), best_time_improve = 0;
    int curr_time_i, new_time_i, curr_time_j, new_time_j, time_diff;
    for (i = 1; i < N; i++) {
        curr_time_i = getRouteConnectionTime(route[i - 1], route[i], route[i + 1]);

        for (j = i + 1; j < N - 2; j++) {
            curr_time_j = getRouteConnectionTime(route[j - 1], route[j], route[j + 1]);
            new_time_j = getRouteConnectionTime(route[i - 1], route[j], route[i + 1]);
            new_time_i = getRouteConnectionTime(route[j - 1], route[i], route[j + 1]);

            time_diff = (curr_time_i + curr_time_j) - (new_time_i + new_time_j);

            if (time_diff > best_time_improve) {
                if (this->use_first_improve) {
                    std::swap(route[i], route[j]);
                    return time_diff;
                }
                best_time_improve = time_diff;
            }
        }
    }
    return best_time_improve;
}

int LocalSearch::getRouteConnectionTime(int prev, int node, int next) {
    return input->getArcTime(prev, node) + input->getArcTime(node, next);
}
