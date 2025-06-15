//
// Created by carlos on 25/05/25.
//

#ifndef CBRP_SIMHEURISTIC_H
#define CBRP_SIMHEURISTIC_H

#include "../classes/Input.hpp"
#include "../common/Postgree.hpp"
#include "boost/algorithm/string/split.hpp"
#include "heuristics/MultiStart.hpp"
#include "zmq.hpp"
#include <string>
#include <unordered_map>
#include <vector>
#include <zmq.h>

class Simheuristic {

  private:
    Input *input;
    DataAccess *data_access;
    string conn_address;
    MultiStart *multi_start;

    void loadNewScenarios(int exec_id) {
        unordered_map<int, unordered_map<int, double>> new_scenarios = data_access->GetCasesFromScenarios(exec_id);
        auto *graph = input->getGraph();
        int B = graph->getB();
        for (auto &[scenario, blocks] : new_scenarios) {
            vector<double> cases_per_block = vector<double>(B, 0);

            for (auto &[block, cases] : blocks) {
                cases_per_block[block] = cases;
            }
            auto new_scenario = Scenario(1.0, cases_per_block);
            this->input->appendNewScenario(new_scenario);
        }
    }

  public:
    int runHeuristic(string &blocks_str) {
        auto *solution = multi_start->GenerateNewSolution();
        auto blocks = solution->getY()[0];

        for (int i = 0; i < blocks.size(); i++) {
            blocks_str += to_string(blocks[i]);
            if (i < blocks.size() - 1) {
                blocks_str += ",";
            }
        }

        return int(solution->getOf());
    }
    explicit Simheuristic(Input *input, string &conn_address) {
        this->input = input;
        this->data_access = new DataAccess();
        this->conn_address = conn_address;
        this->multi_start = new MultiStart(input);
    }

    void Run() {
        std::cout << "[Simheuristic] Starting..." << std::endl;
        zmq::context_t context(1);
        zmq::socket_t subscriber(context, ZMQ_REP);
        subscriber.bind(conn_address.c_str());

        string reply = "done";

        while (true) {
            zmq::message_t message;
            try {
                auto result = subscriber.recv(message, zmq::recv_flags::none);

                if (!result.has_value()) {
                    continue;
                }

                string msg(static_cast<char *>(message.data()), message.size());
                vector<string> command;
                boost::split(command, msg, boost::is_any_of(":"), boost::token_compress_on);

                string action = command[0];
                cout << "action: " << action << endl;
                if (action == "load") {
                    int exec_id = atoi(command[1].c_str());
                    loadNewScenarios(exec_id);
                } else if (action == "run") {
                    string blocks_str;
                    int of = runHeuristic(blocks_str);
                    reply = "solution:" + blocks_str + ":" + to_string(of);
                } else if (action == "stop") {
                    cout << "[Simheuristic] ending the run..." << endl;
                    subscriber.send(zmq::buffer(reply), zmq::send_flags::none);
                    break;
                } else if (action == "check_conn") {
                    reply = "connected";
                } else {
                    continue;
                }
                cout << "reply: " << reply << endl;
                subscriber.send(zmq::buffer(reply), zmq::send_flags::none);
            } catch (const zmq::error_t &e) {
                cout << "[Simheuristic] Error: " << e.what() << endl;
                break;
            }
        }
    }
};

#endif