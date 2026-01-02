#include "src/classes/Input.hpp"
#include "src/exact/DeterministicModel.hpp"
#include "src/exact/DeterministicModelWalk.hpp"
#include <string>

int main(int argc, const char *argv[]) {
    string file_graph = argv[1];
    string model = argv[2];
    string result_file = argv[3];
    string model_type = argv[4];
    int T = 1200;
    bool use_preprocessing = bool(atoi(argv[5]));
    bool use_frac_cut = bool(atoi(argv[6]));
    bool use_warm_start = bool(atoi(argv[7]));
    bool is_walk = model_type == "walk";
    int default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;

    auto *input = new Input(file_graph, "", use_preprocessing, true, is_walk, default_vel, neblize_vel, T, alpha);
    if (is_walk) {
        auto *dm = new DeterministicModelWalk(input);
    } else {
        auto *dm = new DeterministicModel(input);
    }

    Solution sol = dm->Run(use_warm_start, "3600", model, use_frac_cut);
    sol.WriteDeterministicSolution(result_file);
    return 0;
}
