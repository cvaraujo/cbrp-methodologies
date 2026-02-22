#include "src/classes/Input.hpp"
#include "src/simheuristic/simheuristic.hpp"
#include <string>

int main(int argc, const char *argv[]) {
    string file_graph = argv[1];
    string file_scenarios = argv[2];
    string result_file = argv[3];
    string conn_address = argv[4];
    double alpha = 0.8;
    int T = 1200;
    int default_vel = 20, neblize_vel = 10;

    DataAccess da = DataAccess();
    auto new_scenarios = da.GetCasesFromScenarios(0);
    auto *input = new Input(file_graph, file_scenarios, default_vel, neblize_vel, T, alpha);
    Simheuristic simHeu = Simheuristic(input, conn_address);
    simHeu.Run();

    return 0;
}
