#ifndef DPARP_SCENARIO_H
#define DPARP_SCENARIO_H

#include "Parameters.hpp"

class Scenario
{
    double probability = 0.0;
    vector<double> cases_per_block;

public:
    Scenario(double probability, vector<double> cases_per_block)
    {
        this->probability = probability;
        this->cases_per_block = cases_per_block;
    }

    Scenario() {};

    void setCase2Block(int block, double cases) { this->cases_per_block[block] = cases; };

    float getCasesPerBlock(int block) { return this->cases_per_block[block]; };

    void setCasesPerBlock(vector<double> cases) { this->cases_per_block = cases; };

    vector<double> getCases() { return this->cases_per_block; };

    float getProbability() { return this->probability; };

    void setProbability(double probability) { this->probability = probability; };
};

#endif