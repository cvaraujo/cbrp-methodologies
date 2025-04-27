//
// Created by Carlos on 27/05/2025.
//
#include <utility>

#include "Parameters.hpp"

#ifndef DPARP_CHANGES_H
#define DPARP_CHANGES_H

struct Change {
    double delta = -INF;
    vector<pair<int, int_pair>> swaps;
    vector<int_pair> insertions;
    vector<int_pair> deletions;
};

namespace ChangeUtils {

ChangeAction decideAction(const Change &change) {
    if (!change.swaps.empty()) {
        return ChangeAction::Swap;
    }
    if (!change.insertions.empty()) {
        return ChangeAction::Insertion;
    }
    if (!change.deletions.empty()) {
        return ChangeAction::Deletion;
    }
    return ChangeAction::None;
}

Change createEmptyChange() {
    Change change;
    change.delta = 0.0;
    change.swaps.clear();
    change.insertions.clear();
    change.deletions.clear();
    return change;
}

Change newChange(double delta) {
    Change change;
    change.delta = delta;
    change.swaps = vector<pair<int, int_pair>>();
    change.insertions = vector<int_pair>();
    change.deletions = vector<int_pair>();
    return change;
}

Change newChange(double delta, vector<pair<int, int_pair>> swaps, vector<int_pair> insertions, vector<int_pair> deletions) {
    Change change;
    change.delta = delta;
    change.swaps = std::move(swaps);
    change.insertions = std::move(insertions);
    change.deletions = std::move(deletions);
    return change;
}

Change newChange(double delta, vector<int_pair> deletions) {
    Change change;
    change.delta = delta;
    change.deletions = std::move(deletions);
    return change;
}

void clear(Change &change) {
    change.delta = 0.0;
    change.swaps.clear();
    change.insertions.clear();
    change.deletions.clear();
}

// Add a swap
void addSwap(Change &change, int scenario, int to_remove, int to_insert) {
    change.swaps.emplace_back(scenario, int_pair{to_remove, to_insert});
}

// Add an insertion
void addInsertion(Change &change, int scenario, int to_insert) {
    change.insertions.emplace_back(scenario, to_insert);
}

// Add a deletion
void addDeletion(Change &change, int scenario, int to_remove) {
    change.deletions.emplace_back(scenario, to_remove);
}

}; // namespace ChangeUtils

#endif
