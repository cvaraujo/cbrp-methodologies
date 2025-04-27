//
// Created by carlos on 05/01/25
//

#ifndef DPARP_STOCHASTIC_LSEARCH_H
#define DPARP_STOCHASTIC_LSEARCH_H

#include "../../classes/Parameters.hpp"
#include "../../classes/Input.hpp"
#include "../../classes/Solution.hpp"
#include "Utils.hpp"

class LocalSearch
{

private:
    Input *input;
    Solution *solution;
    string delta_type = "moderate";
    bool use_first_improve = false;

public:
    LocalSearch(Input *input, Solution *solution)
    {
        this->input = input;
        this->solution = solution;
    }

    void RunInRouteSwapImprove(string delta_type, bool is_first_improve);

    double GetWeakDeltaSwapBlocksStartScenario(int b1, int b2);

    double GetUpdatedSecondStageCases(Scenario *scenario, int block, bool attended_first_stage)
    {
        double cases = scenario->getCasesPerBlock(block), alpha = input->getAlpha();
        return attended_first_stage ? (1.0 - alpha) * cases : cases;
    }

    double GetUpdatedFirstStageCases(int block)
    {
        return input->getFirstStageProfit(block);
    }

    int GetBestSecondStageOptionIfB1LeaveFSSolution(Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1, int b2);

    int GetBestSecondStageOptionIfB2EnterFSSolution(Scenario *scenario, Route *f_stage_route, Route *s_stage_route, int b1, int b2, int changed);

    double GetModerateDeltaSwapBlocksStartScenario(int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps);

    double GetModerateDeltaOutRouteSwap(int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps);

    double ComputeSwapBlocks(string delta_type, vector<pair<int, int_pair>> &best_swaps, bool is_first_improve, bool is_out_route);

    double ComputeOutRouteSwapBlocksStartScenario(string delta_type, vector<pair<int, int_pair>> &best_swaps, bool is_first_improve);

    int_pair GetRandomB1NB2(Route *route, vector<int> &blocks);

    double ComputeInRouteRandomSwapBlocksStartScenario(string delta_type, vector<pair<int, int_pair>> &best_swaps);

    Route *RemoveBlockFromRoute(int route_idx, int block);

    void InsertOutRouteBlock(int route_idx, int block);

    double RunDefaultPerturbation(vector<pair<int, int_pair>> &swaps);
};

#endif
