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
        double cases = scenario->getCasesPerBlock(block);
        return attended_first_stage ? (1.0 - input->getAlpha()) * cases : cases;
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

    double GetWeakDeltaInsertBlock(int block);

    double GetWeakDeltaRemoveBlock(int block);

    double GetModerateDeltaInsertBlock(int block, vector<pair<int, int_pair>> &second_stage_swaps);

    double GetModerateDeltaRemoveBlock(int block, vector<pair<int, int_pair>> &second_stage_swaps);

    double GetModerateDeltaOutRouteSwap(int b1, int b2, vector<pair<int, int_pair>> &second_stage_swaps);

    Change ComputeSwapBlocks(bool is_out_route);

    Change ComputeOutRouteSwapBlocksStartScenario();

    static int_pair GetRandomB1NB2(const Route *route, vector<int> &blocks);

    Change SelectRandomSwapBlocks();

    Change SelectRandomRemoveBlock();

    Change SelectRandomInsertBlock(bool try_improve_route);

    int_pair GetRandomBlocksFeasibleSwap(Route *route);

    Route *RemoveBlockFromRoute(int route_idx, int block);

    void InsertOutRouteBlock(int route_idx, int block);

    Change RunDefaultPerturbation(bool use_random);

    Change RandomBlockChange();

    double ComputeRandomBlockDiversification(vector<pair<int, int_pair>> &swaps);

    double ComputeRandomBlockIntensification(vector<pair<int, int_pair>> &swaps);

    // TODO: implement the following functions
    Change SelectTimeInsertBlock();

    Change SelectTimeRemoveBlock();

    Change SelectProfitInsertBlock();

    Change SelectProfitRemoveBlock();

    void ImproveRouteTime();

    int ApplyNodeSwap(vector<int> &route);

    int getRouteConnectionTime(int prev, int node, int next);
};

#endif
