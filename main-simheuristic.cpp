#include "src/classes/Input.hpp"
#include "src/simheuristic/heuristics/LocalSearch.hpp"
#include "src/simheuristic/heuristics/MultiStart.hpp"
#include "src/simheuristic/simheuristic.hpp"
#include <string>

int main(int argc, const char *argv[]) {
    int T = 1200;
    string file_graph = argv[1];
    T = atoi(argv[2]);
    string conn_address = argv[3];

    int default_vel = 20, neblize_vel = 10;
    double alpha = 0.8;

    auto *input = new Input(file_graph, "", default_vel, neblize_vel, T, alpha);
    Simheuristic simHeu = Simheuristic(input, conn_address);
    simHeu.Run();

    return 0;
}
