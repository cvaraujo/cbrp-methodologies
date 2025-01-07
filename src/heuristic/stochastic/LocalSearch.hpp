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
    static double GetWeekDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
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

    /*
        Consider the block swap in S0 and the replacement
        of the new block, if is necessary for another feasible block in the route.
    */
    static double GetMediumDeltaSwapBlocksStartScenario(Input *input, Solution *solution, int b1, int b2)
    {
        double delta = 0.0, cases_b1 = graph->getCasesPerBlock(b1), cases_b2 = graph->getCasesPerBlock(b2);
        double alpha = input->getAlpha(), prob;
        Graph *graph = input->getGraph();
        int S = input->getS();
        delta = cases_b2 - cases_b1;

        for (int s = 1; s <= S; s++)
        {
            Scenario scenario = input->getScenario(s - 1);
            Route *s_route = solution->getRouteFromScenario(s);
            prob = scenario.getProbability();

            if (!s_route->isBlockAttended(b1) && s_route->isBlockInRoute(b1))
            {
                for (auto block : s_route->getBlocks())
                {
                    if (block != b1 && scenario.getCasesPerBlock(block) > s_route->getLowestProfitableBlock() && s_route->isSwapTimeLowerThanT(b1, block))
                    {
                    }
                    if (block != b1 &&)
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
