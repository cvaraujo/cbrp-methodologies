//
// Created by carlos on 07/12/24.
//

#ifndef DPARP_STOCHASTIC_ROUTELSEARCH_H
#define DPARP_STOCHASTIC_ROUTELSEARCH_H

#include "../../classes/Parameters.hpp"
#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"

class RouteLocalSearch
{

public:
    static void RunRandom2Opt(Solution *solution, int s)
    {
        vector<int> route = solution->getRouteFromScenario(s);
        Graph *graph = solution->getGraph();
        int N = graph->getN();
    };
};
#endif
