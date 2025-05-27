
//
// Created by Carlos on 25/05/2025.
//

#ifndef SCBRP_POSTGREE_H
#define SCBRP_POSTGREE_H

#include "../classes/Parameters.hpp"
#include "pqxx/pqxx"
#include <unordered_map>

class DataAccess {

    unique_ptr<pqxx::connection> conn;

  public:
    explicit DataAccess() {
        try {
            this->conn = std::make_unique<pqxx::connection>("dbname=dengue-propagation user=postgres password=07021997 host=localhost port=5432");

            if (conn->is_open()) {
                std::cout << "Connected to database: " << conn->dbname() << "\n";
            } else {
                throw std::runtime_error("Can't open database");
            }
        } catch (const std::exception &e) {
            std::cerr << "Connection failed: " << e.what() << "\n";
            throw;
        }
    };

    ~DataAccess() {
        if (conn && conn->is_open()) {
            conn.reset();
            std::cout << "Disconnected.\n";
        }
    }

    std::unordered_map<int, std::unordered_map<int, int>> GetCasesFromScenarios(const int exec_id) {
        pqxx::work txn(*conn);
        pqxx::result result = txn.exec_params(
            "SELECT living_place, simulation_id FROM metrics_infected_people WHERE execution_id = $1",
            exec_id);

        std::unordered_map<int, std::unordered_map<int, int>> scenario_block_cases;
        for (const auto &row : result) {
            int living_place = row["living_place"].as<int>();
            int simulation_id = row["simulation_id"].as<int>();

            if (scenario_block_cases.find(simulation_id) == scenario_block_cases.end()) {
                scenario_block_cases[simulation_id] = std::unordered_map<int, int>();
            }

            if (scenario_block_cases[simulation_id].find(living_place) == scenario_block_cases[simulation_id].end()) {
                scenario_block_cases[simulation_id][living_place] = 0;
            }

            scenario_block_cases[simulation_id][living_place]++;
        }

        txn.commit();

        return scenario_block_cases;
    }
};

#endif // SCBRP_POSTGREE_H
