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

    static void Swap2InBlocks(Input *input, Route *route, int i, int j) {

    };
};
#endif
