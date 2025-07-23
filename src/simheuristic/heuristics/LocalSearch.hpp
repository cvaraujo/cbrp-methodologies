//
// Created by carlos on 09/11/24.
//

#ifndef DPARP_LOCALSEARCH_H
#define DPARP_LOCALSEARCH_H

#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "../../common/BlockConnection.hpp"

class LocalSearch {
    Input *input;
    Solution *bestSolution, *current_solution;

  public:
    LocalSearch(Input *input, Solution *initial_solution) {
        this->input = input;
        this->bestSolution = initial_solution;
        this->current_solution = initial_solution;
    };

    ~LocalSearch() { delete bestSolution, current_solution; }

    Solution *getBestSolution() { return bestSolution; }

    bool isFeasibleInRouteSwap(Route *route, int b_remove, int b_insert) {
        int time_remove = input->getBlockTime(b_remove),
            time_insert = input->getBlockTime(b_insert);
        return route->getTimeRoute() + route->getTimeAttBlocks() - time_remove + time_insert <= input->getT();
    }

    // TODO: Implement this function
    bool isFeasibleOutRouteSwap(Route *route, int b_remove, int b_insert) {
        return false;
    }

    bool isSwapFeasible(Route *route, int b_remove, int b_insert) {
        if (route->IsBlockInRoute(b_insert)) {
            return isFeasibleInRouteSwap(route, b_remove, b_insert);
        } else {
            return isFeasibleOutRouteSwap(route, b_remove, b_insert);
        }
    }

    int getDeltaSwap(Route *route, int b_remove, int b_insert) {
        return int(input->getCasesFromScenarioBlock(0, b_insert) - input->getCasesFromScenarioBlock(0, b_remove));
    }

    int getBestSwap(Route *route, int_pair &bestSwap, bool useFirstImprove) {
        auto *Graph = this->input->getGraph();
        int b_insert, B = Graph->getB(), best_delta = -1;
        int_pair best_swap = {0, 0};

        for (int b_remove : route->getSequenceOfAttendingBlocks()) {
            for (b_insert = 0; b_insert < B; b_insert++) {
                if (route->IsBlockAttended(b_insert))
                    continue;

                if (isSwapFeasible(route, b_remove, b_insert)) {
                    int delta = getDeltaSwap(route, b_remove, b_insert);
                    if (delta >= best_delta) {
                        if (useFirstImprove) {
                            bestSwap = {b_remove, b_insert};
                            return delta;
                        }
                        best_swap = {b_remove, b_insert};
                        best_delta = delta;
                    }
                }
            }
        }

        bestSwap = best_swap;
        return best_delta;
    }

    void RunLocalSearch() {
        auto *route = this->current_solution->getRouteFromScenario(0);
        // Get the options then apply to the solution
    }
};

#endif
