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
    static void RunRandom2OPT(Input *input, Solution *solution)
    {
        Route *route = solution->getRouteFromScenario(0);
    };

    /*
        Consider the block swap in S0 without changing nothing in the routes.
    */
    static double GetWeakDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
    {
        double delta = 0.0, cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
        double alpha = input->getAlpha(), prob;
        Graph *graph = input->getGraph();
        int S = input->getS();
        delta = cases_b2 - cases_b1;

        for (int s = 1; s <= S; s++)
        {
            Scenario scenario = input->getScenario(s - 1);
            prob = scenario.getProbability();
            delta += prob * alpha * (scenario.getCasesPerBlock(b2) - scenario.getCasesPerBlock(b1));
        }

        return delta;
    };

    static double GetUpdatedSecondStageCases(Input *input, Scenario *scenario, int block, bool attended_first_stage)
    {
        double cases = scenario->getCasesPerBlock(block), alpha = input->getAlpha();
        return attended_first_stage ? (1.0 - alpha) * cases : cases;
    }

    static double GetUpdatedFirstStageCases(Input *input, int block)
    {
        Graph *graph = input->getGraph();
        int S = input->getS();
        double delta = graph->getCasesPerBlock(block), alpha = input->getAlpha();

        for (int s = 0; s < S; s++)
        {
            Scenario scenario = input->getScenario(s);
            delta += scenario.getProbability() * alpha * scenario.getCasesPerBlock(block);
        }

        return delta;
    }

    /*
        Consider the block swap in S0 and the replacement
        of the new block, if is necessary for another feasible block in the route.
    */
    static double GetModerateDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
    {
        double cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
        double alpha = input->getAlpha(), prob, lowest_old_profit = INF;
        Graph *graph = input->getGraph();
        int S = input->getS(), lowest_block = -1;
        double delta = cases_b2 - cases_b1;
        Route *first_stage_route = solution->getRouteFromScenario(0);

        for (int s = 1; s <= S; s++)
        {
            Scenario scenario = input->getScenario(s - 1);
            Route *s_route = solution->getRouteFromScenario(s);
            prob = scenario.getProbability();

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

            if (!s_route->isBlockAttended(b1) && s_route->isBlockInRoute(b1))
            {
                double profit_b1 = getUpdatedSecondStageCases(input, scenario, b1, false);

                // Check if B1 can replace another attended block in the route
                for (auto block : s_route->getBlocks())
                {
                    bool is_block_attended_fs = first_stage_route->isBlockAttended(block);
                    bool is_block_attended_ss = s_route->isBlockAttended(block);
                    double profit_block = getUpdatedSecondStageCases(input, scenario, block, is_block_attended_fs);

                    if (block == b1 || !is_block_attended_ss || !s_route->isSwapTimeLowerThanT(b1, block))
                        continue;

                    if (profit_b1 > profit_block && profit_block < lowest_old_profit)
                    {
                        lowest_block = block;
                        lowest_old_profit = profit_block;
                    }
                }
            }

            if (s_route->isBlockAttended(b2))
            {
                double profit_b2 = getUpdatedSecondStageCases(input, scenario, b2, true);

                // Check if B2 could be replaced by another block in the route
                for (auto block : s_route->getBlocks())
                {
                    bool is_block_attended_fs = first_stage_route->isBlockAttended(block);
                    bool is_block_attended_ss = s_route->isBlockAttended(block);
                    double profit_block = getUpdatedSecondStageCases(input, scenario, block, is_block_attended_fs);

                    if (block == b2 || is_block_attended_ss || !s_route->isSwapTimeLowerThanT(b2, block))
                        continue;

                    if (profit_block > profit_b2 && profit_block < lowest_old_profit)
                    {
                        lowest_block = block;
                        lowest_old_profit = profit_block;
                    }
                }
            }

            delta += prob * alpha * (scenario.getCasesPerBlock(b2) - scenario.getCasesPerBlock(b1));
        }

        return delta;
    };

    double ComputeBestInRouteBlockSwap(Input *input, Route *route)
    {
        double best = 0.0;
        vector<int> blocks = route->getBlocks();
        int i, j, b1, b2;

        for (i = 0; i < blocks.size(); i++)
        {
            b1 = blocks[i];

            if (!route->isBlockAttended(b1))
                continue;

            for (j = 0; j < blocks.size(); j++)
            {
                b2 = blocks[j];
                if (i == j || route->isBlockAttended(b2))
                    continue;

                if (route->isSwapFeasible(b1, b2))
                {
                    double delta = route->getDeltaSwapBlocks(b1, b2);
                    if (delta < best)
                        best = delta;
                }
            }
        }
    };
};
#endif
