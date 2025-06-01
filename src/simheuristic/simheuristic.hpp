//
// Created by carlos on 25/05/25.
//

#ifndef CBRP_SIMHEURISTIC_H
#define CBRP_SIMHEURISTIC_H

#include "../classes/Input.hpp"
#include "../common/Postgree.hpp"
#include "boost/algorithm/string/split.hpp"
#include "zmq.hpp"
#include <unordered_map>
#include <vector>
#include <zmq.h>

class Simheuristic {

  private:
    Input *input;
    DataAccess *data_access;
    string listen_address;
    string publish_address;

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

    void runHeuristic() {
    }

    void insertNewSolutionIntoDatabase() {
    }

  public:
    explicit Simheuristic(Input *input, string &listen_address, string &publish_address) {
        this->input = input;
        this->data_access = new DataAccess();
        this->listen_address = listen_address;
        this->publish_address = publish_address;
    }

    // Commands = {
    //     "run": 0,
    //     "end": 0,
    //     "load": exec_id
    // }
    void Run() {
        std::cout << "[Simheuristic] Starting..." << std::endl;
        zmq::context_t context(1);
        zmq::socket_t subscriber(context, ZMQ_REP);
        subscriber.bind(listen_address.c_str());
        string reply = "done";

        while (true) {
            zmq::message_t message;
            auto result = subscriber.recv(message, zmq::recv_flags::none);

            if (result.has_value()) {
                string msg(static_cast<char *>(message.data()), message.size());
                vector<string> command;
                boost::split(command, msg, boost::is_any_of(":"));

                string action = command[0];
                if (command[0] == "load") {
                    int exec_id = atoi(command[1].c_str());
                    loadNewScenarios(exec_id);
                } else if (command[0] == "run") {
                    runHeuristic();
                } else if (command[0] == "end") {
                    cout << "[Simheuristic] ending the run..." << endl;
                    subscriber.send(zmq::buffer(reply), zmq::send_flags::none);
                    break;
                } else {
                    continue;
                }
                subscriber.send(zmq::buffer(reply), zmq::send_flags::none);
            }
        }
    }
};

#endif