#include "LocalSearch.hpp"

Solution *LocalSearch::Run2Opt(int i, int j)
{
    cout << "Running 2-opt" << endl;
    Graph *graph = input->GetGraph();
    int N = graph->GetN();

    if (i >= N || j >= N)
    {
        cout << "Invalid i or j" << endl;
        return nullptr;
    }

    vector<int> new_route = current_solution->GetRoute();
    int node_i = route[i], node_j = route[j];
    int pred_i = pred[node_i], pred_j = pred[node_j];

    if (pred_i == node_j || pred_j == node_i)
    {
        cout << "Invalid i or j" << endl;
        return nullptr;
    }

    auto old_arc_ii = graph->getArc(pred_i, node_i);
    auto old_arc_jj = graph->getArc(pred_j, node_j);
    auto new_arc_ij = graph->getArc(pred_i, node_j);
    auto new_arc_ji = graph->getArc(pred_j, node_i);

    if (new_arc_ij == nullptr || new_arc_ji == nullptr)
    {
        cout << "Invalid i or j" << endl;
        return nullptr;
    }

    if ((old_arc_ii->getLength() + old_arc_jj->getLength()) > (new_arc_ij->getLength() + new_arc_ji->getLength()))
    {
        new_route[i] = node_j;
        new_route[j] = node_i;

        vector<int> new_pred = pred;
        vector<int> new_y = y;

        new_pred[node_i] = pred_j;
        new_pred[node_j] = pred_i;

        new_y[pred_i] = node_j;
        new_y[pred_j] = node_i;

        this->current_solution->setPred(new_pred);
        this->current_solution->setRoute(new_route);
    }

    return nullptr;
}