#include "src/classes/Input.hpp"
// #include "src/common/Postgree.hpp"
// #include "src/heuristic/metaheuristics/SimulatedAnnealing.hpp"
// #include "src/heuristic/stochastic/LocalSearch.hpp"
// #include "src/heuristic/stochastic/StartSolution.hpp"
#include "src/simheuristic/simheuristic.hpp"
#include <string>

// #include "src/heuristic/Lagrangean.hpp"
// #include "src/exact/DeterministicModel.hpp"
// #include "src/exact/DeterministicModelWalk.hpp"
// #include "src/exact/StochasticModel.hpp"
// #include "src/exact/StochasticModelWalk.hpp"

// void print_result(Route *r) {
//     cout << "=================\nRoute: ";
//     for (auto node : r->getRoute())
//         cout << node << ", ";
//     cout << endl;

//     cout << "---------------------\nAtt. Blocks: ";
//     for (auto block : r->getSequenceOfAttendingBlocks())
//         cout << block << ", ";
//     cout << endl;

//     cout << "---------------------\nRoute Blocks: ";
//     for (auto block : r->getRouteBlocks())
//         cout << block << ", ";
//     cout << endl;
//     cout << "Time: " << r->getTimeRoute() << " + " << r->getTimeAttBlocks() << endl;
//     cout << "=================" << endl;
// }

int main(int argc, const char *argv[]) {
    // random_device rd; // seed
    // double temperature, double temperature_max, double alpha, int max_iterations, const string &delta_type, bool first_improve
    string file_graph = argv[1];
    // string file_scenarios = argv[2];
    // string result_file = argv[3];
    int T = atoi(argv[2]);
    string conn_address = argv[3];
    // double temperature = atof(argv[5]);
    // double temperature_max = atof(argv[6]);
    // double alpha_sa = atof(argv[7]);
    // int max_iters_sa = atoi(argv[8]);
    // string delta_type = argv[9];
    // bool first_improve = atoi(argv[10]);
    int default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;

    // DataAccess da = DataAccess();
    // auto new_scenarios = da.GetCasesFromScenarios(0);
    auto *input = new Input(file_graph, "", default_vel, neblize_vel, T, alpha);
    Simheuristic simHeu = Simheuristic(input, conn_address);
    simHeu.Run();

    // Solution sol = StartSolution::CreateStartSolution(input);

    // auto *sa = new SimulatedAnnealing(temperature, temperature_max, alpha_sa, max_iters_sa, delta_type, first_improve);
    // auto start = std::chrono::high_resolution_clock::now();
    // Solution *new_sol = sa->Run(input, sol, rd);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> duration = end - start;
    // std::cout << "Execution time: " << duration.count() << " seconds\n";
    // new_sol->setRuntime(duration.count());
    // new_sol->WriteSolution(result_file);

    // vector<pair<int, int_pair>> best_swaps;
    // local_search->ComputeInRouteRandomSwapBlocksStartScenario(input, &sol, delta_type, best_swaps);

    // Lagrangean *lag = new Lagrangean(input);
    // lag->lagrangean_relax(result_file, lambda, maxIters, reduction);

    // if (stochastic_model == "FALSE")
    // {
    //   // DeterministicModel *dm = new DeterministicModel(input);
    //   DeterministicModelWalk *dm = new DeterministicModelWalk(input);
    //   Solution sol = dm->Run(false, "3600", model, frac_cut);
    //   sol.WriteSolution(result_file);
    // }
    // else
    // {
    //   StochasticModel *sm = new StochasticModel(input);
    //   // StochasticModelWalk *sm = new StochasticModelWalk(input);
    //   Solution sol = sm->Run(false, "3600", model, frac_cut);
    //   sol.WriteSolution(result_file);
    // }

    return 0;
}
