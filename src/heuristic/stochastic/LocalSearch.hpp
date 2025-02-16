//
// Created by carlos on 05/01/25
//

#ifndef DPARP_STOCHASTIC_LSEARCH_H
#define DPARP_STOCHASTIC_LSEARCH_H

#include "../../classes/Parameters.hpp"
#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"

class LocalSearch
{

public:
    static void RunMoreProfitable2OPT(Input *input, Solution *solution, string delta_type)
    {
        bool is_stuck = false;
        map<int, int_pair> best_fs_swap;
        Graph *graph = input->getGraph();
        cout << "[*] Start OF is matching? " << solution->ComputeCurrentSolutionOF() << endl;

        while (!is_stuck)
        {
            double delta = LocalSearch::ComputeBestSwapBlocksStartScenario(input, solution, delta_type, best_fs_swap);

            if (best_fs_swap.size() > 0)
            {
                for (auto scenario_swap : best_fs_swap)
                {
                    int_pair swap = scenario_swap.second;
                    int scenario = scenario_swap.first, b1 = swap.first, b2 = swap.second;

                    if (b1 == -1 || b2 == -1)
                    {
                        is_stuck = true;
                        break;
                    }

                    cout << "[*] Best swap in scenario " << scenario << " = " << best_fs_swap[0].first << " " << best_fs_swap[0].second << endl;

                    Route *r1 = solution->getRouteFromScenario(scenario);

                    solution->ScenarioBlockSwapWithoutOF(scenario, b1, b2);

                    solution->setOf(solution->getOf() + delta);
                    cout << "[*] Updated OF: " << solution->getOf() << endl;

                    cout << "[=] OF matching? " << solution->ComputeCurrentSolutionOF() << endl;
                    getchar();
                }
            }
            else
            {
                cout << "[!] No more profitable swap found!" << endl;
                is_stuck = true;
            }
        }
    };

    static double GetWeakDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
    {
        Graph *graph = input->getGraph();
        double delta = 0.0, cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
        double alpha = input->getAlpha(), prob, rest = 1.0 - alpha;
        int S = input->getS();
        // First Stage delta
        delta = cases_b2 - cases_b1;

        // if (b1 == 11 && b2 == 37)
        // {
        //     cout << "B1 = " << b1 << ", B2 = " << b2 << endl;
        //     cout << "[FS] Cases B1: " << cases_b1 << " Cases B2: " << cases_b2 << " Delta: " << delta << endl;
        // }

        // B1 turned off
        // B2 turned on
        for (int s = 1; s <= S; s++)
        {
            Scenario *scenario = input->getScenario(s - 1);
            Route *s_route = solution->getRouteFromScenario(s);
            prob = scenario->getProbability();
            double cases_b1 = scenario->getCasesPerBlock(b1), cases_b2 = scenario->getCasesPerBlock(b2);
            // Update first stage change
            delta += prob * alpha * (cases_b2 - cases_b1);

            // if (b1 == 11 && b2 == 37)
            // {
            //     cout << s << "_B1 = " << cases_b1 << ", " << s << "_B2 = " << cases_b2 << endl;
            //     cout << "[FSS] Delta: " << delta << endl;
            // }

            // Independent scenario profit
            // if (s_route->isBlockAttended(b1) && s_route->isBlockAttended(b2))
            // {
            //     delta += alpha * prob * cases_b1;
            //     delta -= alpha * prob * cases_b2;
            // }
            // else
            // {
            if (s_route->isBlockAttended(b1))
                delta += alpha * prob * cases_b1;
            // delta += (alpha * prob * cases_b1) - (rest * prob * cases_b1);
            if (s_route->isBlockAttended(b2))
                delta -= alpha * prob * cases_b2;
            // delta += (rest * prob * cases_b2) - (alpha * prob * cases_b2);
            // }
            // if (b1 == 11 && b2 == 37)
            // {
            //     cout << "[SSS] Delta: " << delta << endl;
            //     getchar();
            // }
        }
        if (delta < 0.001)
            return -1;
        return delta;
    };

    static double GetUpdatedSecondStageCases(Input *input, Scenario *scenario, int block, bool attended_first_stage)
    {
        double cases = scenario->getCasesPerBlock(block), alpha = input->getAlpha();
        return attended_first_stage ? (1.0 - alpha) * cases : cases;
    }

    double GetUpdatedFirstStageCases(Input *input, int block)
    {
        Graph *graph = input->getGraph();
        int S = input->getS();
        double delta = graph->getCasesPerBlock(block), alpha = input->getAlpha();

        for (int s = 0; s < S; s++)
        {
            Scenario *scenario = input->getScenario(s);
            delta += scenario->getProbability() * alpha * scenario->getCasesPerBlock(block);
        }

        return delta;
    }

    static pair<int, double> GetBestSecondStageOptionIfB1EnterFSSolution(
        Input *input, Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1)
    {
        double best_profit = LocalSearch::GetUpdatedSecondStageCases(input, scenario, b1, false);
        int best_block = -1;

        for (auto block : s_stage_route->getBlocks())
        {
            bool is_block_attended_fs = f_stage_route->isBlockAttended(block);
            bool is_block_attended_ss = s_stage_route->isBlockAttended(block);
            double profit_block = LocalSearch::GetUpdatedSecondStageCases(input, scenario, block, is_block_attended_fs);

            if (block == b1 || !is_block_attended_ss || !s_stage_route->isSwapTimeLowerThanT(b1, block))
                continue;

            if (profit_block < best_profit)
                best_block = block, best_profit = profit_block;
        }

        return make_pair(best_block, best_profit);
    }

    static pair<int, double> GetBestSecondStageOptionIfB1LeaveFSSolution(
        Input *input, Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1)
    {
        double best_profit = GetUpdatedSecondStageCases(input, scenario, b1, true);
        int best_block = -1;

        for (auto block : s_stage_route->getBlocks())
        {
            bool is_block_attended_fs = f_stage_route->isBlockAttended(block);
            bool is_block_attended_ss = s_stage_route->isBlockAttended(block);
            double profit_block = GetUpdatedSecondStageCases(input, scenario, block, is_block_attended_fs);

            if (block == b1 || is_block_attended_ss || !s_stage_route->isSwapTimeLowerThanT(b1, block))
                continue;

            if (profit_block > best_profit)
                best_block = block, best_profit = profit_block;
        }

        return make_pair(best_block, best_profit);
    }

    /*
        TODO: Need to copy the route and update the values

        Consider the block swap in S0 and the replacement
        of the new block, if is necessary for another feasible block in the route.
    */
    static double GetModerateDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
    {
        Graph *graph = input->getGraph();
        double cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
        double alpha = input->getAlpha(), prob;
        int S = input->getS(), lowest_block = -1, highest_block = -1;
        double delta = cases_b2 - cases_b1;
        Route *first_stage_route = solution->getRouteFromScenario(0);

        for (int s = 1; s <= S; s++)
        {
            Scenario *scenario = input->getScenario(s - 1);
            Route *s_route = solution->getRouteFromScenario(s);
            prob = scenario->getProbability();

            /*
                B1 was attendent,   then not    -> increase value in scenario S
                B2 not attendent, then attended ->  reduce  value in scenario S

                B1 é atendido em S:
                    Nada a fazer
                B2 não é atendido em S mas pertence a rota:
                    Nada a fazer
                B1 não é atendido em S mas pertence a rota:
                    Verificar se é possível trocar por outro bloco que seja rentável
                B2 é atendido em S:
                    Verificar se é possível trocar por outro bloco que seja rentável

            */

            pair<int, double> pair_swap_s_stage;
            if (!s_route->isBlockAttended(b1) && s_route->isBlockInRoute(b1))
            {
                pair_swap_s_stage = LocalSearch::GetBestSecondStageOptionIfB1EnterFSSolution(input, scenario, first_stage_route, s_route, b1);
                lowest_block = pair_swap_s_stage.first;

                if (lowest_block != -1)
                    delta += prob * alpha * (scenario->getCasesPerBlock(b1) - scenario->getCasesPerBlock(lowest_block));
            }

            if (s_route->isBlockAttended(b2))
            {
                pair_swap_s_stage = LocalSearch::GetBestSecondStageOptionIfB1LeaveFSSolution(input, scenario, first_stage_route, s_route, b2);
                highest_block = pair_swap_s_stage.first;

                if (highest_block != -1)
                    delta += prob * alpha * (scenario->getCasesPerBlock(highest_block) - scenario->getCasesPerBlock(b2));
            }
        }

        return delta;
    };

    static double ComputeBestSwapBlocksStartScenario(Input *input, Solution *solution, string delta_type, map<int, int_pair> &best_swap)
    {
        Graph *graph = input->getGraph();
        Route *route = solution->getRouteFromScenario(0);
        double best = 0.0, delta = 0.0;
        set<int> set_blocks = route->getBlocks();
        vector<int> blocks = vector<int>(set_blocks.begin(), set_blocks.end());
        best_swap = map<int, int_pair>();
        int i, j, b1, b2, best_b1 = -1, best_b2 = -1;

        // cout << "Route" << endl;
        // for (auto i : route->getRoute())
        // {
        //     int n1 = i.first, n2 = i.second;
        //     cout << "Blcoks in node " << n1 << ": ";
        //     for (auto b : graph->getNode(n1).second)
        //         cout << b << " ";
        //     cout << endl;

        //     cout << "Blcoks in node " << n2 << ": ";
        //     for (auto b : graph->getNode(n2).second)
        //         cout << b << " ";
        //     cout << endl;
        //     getchar();
        // }

        // cout << "Possible blocks to swap: " << endl;
        // for (auto i : blocks)
        //     cout << i << " ";
        // cout << endl;
        // getchar();

        for (i = 0; i < blocks.size(); i++)
        {
            b1 = blocks[i];
            // If the block is not attended, continue
            if (!route->isBlockAttended(b1))
                continue;

            // cout << "Check to swap: " << b1 << endl;

            for (j = 0; j < blocks.size(); j++)
            {
                b2 = blocks[j];
                // cout << "With: " << b2 << ", Attended: " << route->isBlockAttended(b2) << " or Feasible: " << route->isSwapFeasible(b1, b2) << endl;

                // If the block is attended, continue
                if (i == j || route->isBlockAttended(b2) || !route->isSwapFeasible(b1, b2))
                    continue;

                // getchar();
                delta = 0.0;
                if (delta_type == "weak")
                {
                    // cout << "[!] Weak" << endl;
                    delta = GetWeakDeltaSwapBlocksStartScenario(input, solution, b1, b2);
                    // cout << "B1: " << b1 << " B2: " << b2 << " Delta: " << delta << endl;
                    // getchar();
                }
                else if (delta_type == "moderate")
                {
                    cout << "[!] Moderate" << endl;
                    // delta = route->getDeltaSwapBlocksModerate(input, b1, b2);
                }
                else if (delta_type == "strong")
                {
                    cout << "[!] Strong" << endl;
                    // delta = route->getDeltaSwapBlocksStrong(input, b1, b2);}
                }

                if (delta > best)
                    best = delta, best_b1 = b1, best_b2 = b2;
            }
        }

        best_swap[0] = make_pair(best_b1, best_b2);

        cout << "[*] Best swap: " << best_swap[0].first << " " << best_swap[0].second << " = " << best << endl;
        getchar();
        return best;
    }
};
#endif
