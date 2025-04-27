//
// Created by carlos on 05/01/25
//

#ifndef DPARP_STOCHASTIC_LSEARCH_H
#define DPARP_STOCHASTIC_LSEARCH_H

#include "../../classes/Changes.hpp"
#include "../../classes/Input.hpp"
#include "../../classes/Parameters.hpp"
#include "../../classes/Solution.hpp"
#include "Utils.hpp"

class LocalSearch {

  private:
    Input *input;
    Solution *solution;
    string delta_type = "moderate";
    bool use_first_improve = false;

  public:
    LocalSearch(Input *input, Solution *solution) {
        this->input = input;
        this->solution = solution;
    }

    void RunInRouteSwapImprove();

    double GetWeakDeltaSwapBlocksStartScenario(int b1, int b2);

    double GetUpdatedSecondStageCases(const Scenario *scenario, int block, bool attended_first_stage) {
        double cases = scenario->getCasesPerBlock(block), alpha = input->getAlpha();
        return attended_first_stage ? (1.0 - alpha) * cases : cases;
    }

    double GetUpdatedFirstStageCases(int block) {
        return input->getFirstStageProfit(block);
    }

    int GetBestSecondStageOptionIfB1LeaveFSSolution(const Scenario *scenario,
                                                    const Route *f_stage_route,
                                                    const Route *s_stage_route, int b1,
                                                    int b2);

    int GetBestSecondStageOptionIfB2EnterFSSolution(Scenario *scenario,
                                                    Route *f_stage_route,
                                                    Route *s_stage_route, int b1,
                                                    int b2, int changed);

    double GetModerateDeltaSwapBlocksStartScenario(
        int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps);

    double
    GetModerateDeltaOutRouteSwap(int b1, int b2,
                                 vector<pair<int, int_pair>> &second_stage_swaps);

    double ComputeSwapBlocks(vector<pair<int, int_pair>> &best_swaps, bool is_out_route);

    double ComputeOutRouteSwapBlocksStartScenario(vector<pair<int, int_pair>> &best_swaps);

    static int_pair GetRandomB1NB2(const Route *route, vector<int> &blocks);

    double SelectRandomSwapBlocks(vector<pair<int, int_pair>> &best_swaps);

    Change SelectRandomRemoveBlock();

    double TryImproveRouteTime();

    double SelectRandomInsertBlock();

    int_pair GetRandomBlocksFeasibleSwap(Route *route);

    Route *RemoveBlockFromRoute(int route_idx, int block);

    void InsertOutRouteBlock(int route_idx, int block);

    double RunDefaultPerturbation(vector<pair<int, int_pair>> &swaps, bool use_random);

    double RandomBlockChange(vector<pair<int, int_pair>> &swaps);

    double ComputeRandomBlockDiversification(vector<pair<int, int_pair>> &swaps);

    double ComputeRandomBlockIntensification(vector<pair<int, int_pair>> &swaps);
};

#endif
