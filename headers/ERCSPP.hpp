#ifndef RCSPP_HPP_
#define RCSPP_HPP_

#include <vector>
#include <algorithm>
#include <bitset>
#include <queue>
#include <iostream>

#include <boost/format.hpp>
#include <boost/log/trivial.hpp>
#include <gurobi_c++.h>
#include <cmath>
#include <set>

#define BITSETLENGTH 21

using namespace std;

class GraphRCSPP
{
public:
    const vector<int> V;
    const vector<vector<double>> time;
    const vector<vector<double>> cost;
    vector<vector<int>> adjList;
    vector<vector<int>> revAdjList;
    double T;
};

class Label
{
public:
    int node;
    double cost;
    double time;
    bitset<BITSETLENGTH> nodes;

    explicit Label()
    {
        nodes.reset();
    }
    explicit Label(int node_, double cost_, double time_)
        : node(node_), cost(cost_), time(time_)
    {
        nodes.reset();
        nodes.set(node, 1);
    }
    bool operator==(const Label &label)
    {
        return nodes == label.nodes && cost == label.cost && time == label.time;
    }
};

class MonodirLabeling
{
public:
    //
    const GraphRCSPP &g;
    const int &s;
    const int &t;
    vector<vector<Label>> node_labels;
    double UB;

    explicit MonodirLabeling(const GraphRCSPP &g_, const int &s_, const int &t_, const double &UB_)
        : g(g_), s(s_), t(t_), node_labels(g.V.size(), vector<Label>()), UB(UB_) {}

    void run()
    {
        const auto &adjList = g.adjList;

        node_labels[s].push_back(Label(s, 0, 0));
        std::queue<Label> labels;
        labels.push(node_labels[s].front());

        while (!labels.empty())
        {
            const Label label = labels.front();
            labels.pop();

            const int &curr = label.node;

            for (const int &next : adjList[curr])
            {
                if (!possibleExtension(label, next))
                    continue;

                Label new_label;
                extend(new_label, label, curr, next);

                if (dominated(new_label, node_labels[next]))
                    continue;

                if (boundingPrune(new_label))
                    continue;

                node_labels[next].push_back(new_label);

                if (new_label.node != t)
                    labels.push(new_label);
                else
                    UB = min(UB, new_label.cost);
            }
        }
    }

    inline bool boundingPrune(const Label &label)
    {
        const auto &V = g.V;
        const auto &revAdjList = g.revAdjList;
        const auto &time = g.time;
        const auto &cost = g.cost;
        const auto &T = g.T;
        const auto &n = V.size();
        const auto &i = label.node;

        vector<int> candidates;
        set<int> candidates_set;
        vector<double> min_time;
        vector<double> min_cost;

        for (const int &j : V)
            if (!label.nodes[j])
                candidates.push_back(j);

        const int n_ = int(candidates.size());

        if (n_ == 0)
            return false;
        if (n_ == 1)
            return label.cost + cost[label.node][candidates.front()] > UB;
        if (n_ == 2)
            return label.cost + min(
                                    cost[label.node][candidates.front()] + cost[candidates.front()][candidates.back()],
                                    cost[label.node][candidates.back()] + cost[candidates.back()][candidates.front()]) >
                   UB;

        candidates_set = set<int>(candidates.begin(), candidates.end());

        min_time = vector<double>(n_, GRB_INFINITY);
        min_cost = vector<double>(n_, GRB_INFINITY);

        for (int j_idx; j_idx < n_; j_idx++)
        {
            const auto &j = candidates[j_idx];
            for (const auto &k : revAdjList[j])
            {
                if (j == k || candidates_set.count(k) == 0)
                    continue;

                min_time[j_idx] = min(min_time[j_idx], time[k][j]);
                min_cost[j_idx] = min(min_cost[j_idx], cost[k][j]);
            }
        }

        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set("OutputFlag", "0");
        model.update();

        vector<GRBVar> x = vector<GRBVar>(n_);
        vector<GRBVar> y = vector<GRBVar>(n_);

        for (int i = 0; i < n_; i++)
        {
            x[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "x");
            y[i] = model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, "y");
        }

        GRBLinExpr obj_func_expr;
        for (int idx; idx < n_; idx++)
            obj_func_expr += x[idx] * min_cost[idx] + y[idx] * cost[i][candidates[idx]];

        model.setObjective(obj_func_expr, GRB_MINIMIZE);

        GRBLinExpr knapsack_expr;
        for (int idx; idx < n_; idx++)
        {
            const auto &j = candidates[idx];

            knapsack_expr += x[idx] * min_time[idx] + y[idx] * time[i][j];
        }

        model.addConstr(knapsack_expr <= T);

        for (int idx; idx < n_; idx++)
            model.addConstr(x[idx] + y[idx] <= 1);

        GRBLinExpr adjacent;
        for (int idx; idx < n_; idx++)
            adjacent += y[idx];

        model.addConstr(adjacent == 1);
        model.update();

        try
        {
            model.write("model.lp");
            model.optimize();
        }
        catch (const GRBException &e)
        {
            cout << "Concert exception caught: " << endl;
            throw e;
        }
        catch (...)
        {
            throw string("Error in getting solution");
        }

        //
        double bound = model.get(GRB_DoubleAttr_ObjVal);

        return label.cost + bound > UB;
    }
    //
    inline bool possibleExtension(const Label &label, const int &next)
    {
        return !label.nodes[next] && label.time + g.time[label.node][next] + g.time[next][t] <= g.T;
    }
    //
    inline bool dominates(const Label &label1, const Label &label2)
    {
        if (label1.node != label2.node)
            return false;
        if (label1.cost > label2.cost)
            return false;
        if (label1.time > label2.time)
            return false;
        const bool contained = ((~label1.nodes) | label2.nodes).all();
        if (!contained)
            return false;
        return label1.cost < label2.cost || label1.time < label2.time || label1.nodes.count() < label2.nodes.count();
    }
    //
    inline bool dominated(const Label &label1, const vector<Label> &labels)
    {
        for (const Label &label2 : labels)
            if (dominates(label2, label1))
                return true;
        return false;
    }
    inline void setUnreachableNodes(Label &label) const
    {
        for (const int &i : g.V)
            if (!label.nodes[i] && label.time + g.time[label.node][i] + g.time[i][t] > g.T)
                label.nodes.set(i, true);
    }
    //
    inline void extend(
        Label &new_label,
        const Label &old_label,
        const int &i,
        const int &j) const
    {

        new_label.node = j;

        new_label.time = old_label.time + g.time[i][j];
        new_label.cost = old_label.cost + g.cost[i][j];

        new_label.nodes = old_label.nodes;
        new_label.nodes.set(j, true);

        setUnreachableNodes(new_label);
    }
    //
    inline bool extensionMatch(
        const Label &old_label,
        const Label &new_label) const
    {

        const int &i = old_label.node,
                  j = new_label.node;

        return i != j && new_label.time == old_label.time + g.time[i][j] && new_label.cost == old_label.cost + g.cost[i][j] && (new_label.nodes ^ old_label.nodes).count() >= 1;
    }
    //
    void _getPaths(const Label &label, vector<int> &path, vector<vector<int>> &paths)
    {

        //
        const int &curr = label.node;

        //
        path.push_back(curr);

        // edge case
        if (curr == s)
        {
            vector<int> _path = path;
            reverse(_path.begin(), _path.end());
            paths.push_back(_path);

            return;
        }

        //
        for (const auto &prev : g.revAdjList[curr])
            for (const auto &prev_label : node_labels[prev])
                if (extensionMatch(prev_label, label))
                {
                    _getPaths(prev_label, path, paths);
                    path.pop_back();
                }
    }
    //
    vector<vector<int>> getPaths()
    {
        vector<vector<int>> paths;
        double best_cost = INT_MAX;
        for (const auto &label : node_labels[t])
            best_cost = min(label.cost, best_cost);

        vector<Label> best_labels;
        for (const auto &label : node_labels[t])
            if (label.cost == best_cost)
            {
                vector<int> path;
                _getPaths(label, path, paths);
            }

        //
        return paths;
    }
};

void test_rcspp()
{

    //////////////////////////

    // x y profit
    // Set 2/15 = 120
    vector<std::tuple<double, double, double>> node_entries = {
        make_tuple(4.6, 7.1, 0),
        make_tuple(5.7, 11.4, 20),
        make_tuple(4.4, 12.3, 20),
        make_tuple(2.8, 14.3, 30),
        make_tuple(3.2, 10.30, 15),
        make_tuple(3.5, 9.8, 15),
        make_tuple(4.4, 8.4, 10),
        make_tuple(7.8, 11.0, 20),
        make_tuple(8.8, 9.8, 20),
        make_tuple(7.7, 8.2, 20),
        make_tuple(6.3, 7.9, 15),
        make_tuple(5.4, 8.2, 10),
        make_tuple(5.8, 6.8, 10),
        make_tuple(6.7, 5.8, 25),
        make_tuple(13.8, 13.1, 40),
        make_tuple(14.1, 14.2, 40),
        make_tuple(11.2, 13.6, 30),
        make_tuple(9.7, 16.4, 30),
        make_tuple(9.5, 18.8, 50),
        make_tuple(4.7, 16.8, 30),
        make_tuple(5.0, 5.6, 0),
    };
    double opt_cost = 120;
    double T = 15;
    // Set 2/20 = 200
    node_entries = {
        make_tuple(4.6, 7.1, 0),
        make_tuple(5.7, 11.4, 20),
        make_tuple(4.4, 12.3, 20),
        make_tuple(2.8, 14.3, 30),
        make_tuple(3.2, 10.30, 15),
        make_tuple(3.5, 9.8, 15),
        make_tuple(4.4, 8.4, 10),
        make_tuple(7.8, 11.0, 20),
        make_tuple(8.8, 9.8, 20),
        make_tuple(7.7, 8.2, 20),
        make_tuple(6.3, 7.9, 15),
        make_tuple(5.4, 8.2, 10),
        make_tuple(5.8, 6.8, 10),
        make_tuple(6.7, 5.8, 25),
        make_tuple(13.8, 13.1, 40),
        make_tuple(14.1, 14.2, 40),
        make_tuple(11.2, 13.6, 30),
        make_tuple(9.7, 16.4, 30),
        make_tuple(9.5, 18.8, 50),
        make_tuple(4.7, 16.8, 30),
        make_tuple(5.0, 5.6, 0),
    };
    opt_cost = 200;
    T = 20;

    const double UB = -opt_cost;
    const int source = 0, target = 20;

    const int n = node_entries.size();
    vector<vector<int>> adjList(n, vector<int>());
    vector<vector<int>> revAdjList(n, vector<int>());
    vector<vector<double>> time(n, vector<double>(n, 0));
    vector<vector<double>> cost(n, vector<double>(n, 0));
    vector<int> V;

    for (int i = 0; i < n; ++i)
        V.push_back(i);

    for (const auto &i : V)
    {
        const auto &x_i = get<0>(node_entries[i]),
                   y_i = get<1>(node_entries[i]),
                   p_i = get<2>(node_entries[i]);

        for (const auto &j : V)
        {
            if (i == j)
                continue;

            const auto &x_j = get<0>(node_entries[j]),
                       y_j = get<1>(node_entries[j]),
                       p_j = get<2>(node_entries[j]);

            adjList[i].push_back(j);
            revAdjList[j].push_back(i);
            time[i][j] = sqrt(pow(x_i - x_j, 2) + pow(y_i - y_j, 2));
            cost[i][j] = -(p_i + p_j) / 2.;
        }
    }

    GraphRCSPP g(V, time, cost, adjList, revAdjList, T);
    MonodirLabeling alg(g, source, target, UB);

    alg.run();

    for (const int &i : V)
        cout << i << ": " << alg.node_labels[i].size() << endl;

    ////////
    const auto &paths = alg.getPaths();
    const auto &path = paths.front();

    for (const auto &i : path)
        cout << i << ", ";
    cout << endl;

    double cal_cost = 0;
    double cal_time = 0;
    for (int idx = 1; idx < path.size(); ++idx)
    {
        const int i = path[idx - 1],
                  j = path[idx];

        cal_cost += g.cost[i][j];
        cal_time += g.time[i][j];
    }
    cout << "Best cost: " << cal_cost << endl;
    cout << "Best time: " << cal_time << endl;
}

#endif