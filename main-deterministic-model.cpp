#include "src/classes/Input.hpp"
#include "src/exact/DeterministicModel.hpp"
#include "src/exact/DeterministicModelWalk.hpp"
#include <string>

int main(int argc, const char *argv[]) {
    string file_graph = argv[1];
    string model = argv[2];
    string result_file = argv[3];
    int T = 1200;
    bool use_preprocessing = bool(atoi(argv[4]));
    bool use_frac_cut = bool(atoi(argv[5]));
    bool use_warm_start = bool(atoi(argv[6]));
    int default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;

    auto *input = new Input(file_graph, "", use_preprocessing, true, false, default_vel, neblize_vel, T, alpha);
    auto *dm = new DeterministicModel(input);
    Solution sol = dm->Run(use_warm_start, "3600", model, use_frac_cut);
    sol.WriteDeterministicSolution(result_file);

    return 0;
}
