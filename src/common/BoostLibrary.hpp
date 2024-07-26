//
// Created by carlos on 27/05/19.
//

#ifndef DPARP_BOOST_H
#define DPARP_BOOST_H

#include "Parameters.hpp"
#include <boost/graph/adjacency_list.hpp>

using namespace boost;

class BoostLibrary
{
    struct SPPRC_Graph_Vert_Prop
    {
        SPPRC_Graph_Vert_Prop(int n = 0, int l = 0, int m = 0) : num(n), lim(l), max(m) {}
        int num;
        int lim;
        int max;
    };

    struct SPPRC_Graph_Arc_Prop
    {
        SPPRC_Graph_Arc_Prop(int n = 0, float c = 0, int t = 0, int o = 0) : num(n), cost(c), time(t), cnt(o) {}
        int num;
        // traversal cost
        float cost;
        // traversal time
        int time;
        // Arcs count
        int cnt;
    };

    // data structures for shortest path problem with time constraint
    // ResourceContainer model
    struct spp_res_cont
    {
        spp_res_cont(float c = 0, int t = 0, int o = 0) : cost(c), time(t), cnt(o) {}
        spp_res_cont &operator=(const spp_res_cont &other)
        {
            if (this == &other)
                return *this;
            this->~spp_res_cont();
            new (this) spp_res_cont(other);
            return *this;
        }
        float cost;
        int time;
        int cnt;
    };

    typedef adjacency_list<vecS, vecS, directedS, SPPRC_Graph_Vert_Prop, SPPRC_Graph_Arc_Prop> SPPRC_Graph;
    typedef graph_traits<SPPRC_Graph>::vertex_descriptor vertex_descriptor;
    typedef graph_traits<SPPRC_Graph>::edge_descriptor edge_descriptor;
    typedef adjacency_list<vecS, vecS, directedS, no_property, property<edge_weight_t, int, property<edge_weight2_t, int>>> boostGraph;

    bool operator==(
        const spp_res_cont &res_cont_1, const spp_res_cont &res_cont_2)
    {
        return (res_cont_1.cost == res_cont_2.cost && res_cont_1.time == res_cont_2.time);
    }

    bool operator<(
        const spp_res_cont &res_cont_1, const spp_res_cont &res_cont_2)
    {
        if (res_cont_1.cost > res_cont_2.cost)
            return false;
        if (res_cont_1.cost == res_cont_2.cost)
            return res_cont_1.time < res_cont_2.time;
        return true;
    }

    // ResourceExtensionFunction model
    class ref_spprc
    {
    public:
        inline bool operator()(const SPPRC_Graph &g,
                               spp_res_cont &new_cont, const spp_res_cont &old_cont,
                               graph_traits<SPPRC_Graph>::edge_descriptor ed) const
        {
            const SPPRC_Graph_Arc_Prop &arc_prop = get(edge_bundle, g)[ed];
            const SPPRC_Graph_Vert_Prop &vert_prop = get(vertex_bundle, g)[target(ed, g)];

            new_cont.cost = old_cont.cost + arc_prop.cost;
            int &i_time = new_cont.time;
            int &i_cnt = new_cont.cnt;
            i_time = old_cont.time + arc_prop.time;
            i_cnt = old_cont.cnt + arc_prop.cnt;

            return i_time <= vert_prop.lim && i_cnt <= vert_prop.max ? true : false;
        }
    };

    // DominanceFunction model
    class dominance_spptw
    {
    public:
        inline bool operator()(const spp_res_cont &res_cont_1,
                               const spp_res_cont &res_cont_2) const
        {
            return res_cont_1.cost <= res_cont_2.cost && res_cont_1.time <= res_cont_2.time;
        }
    };
    // end data structures for shortest path problem with resource constraint

public:
    BoostLibrary() = default;

    ~BoostLibrary() = default;

    double BoostLibrary::run_spprc(set<pair<int, int>> &x)
    {
        // Run the shortest path with resource constraints
        vector<vector<edge_descriptor>> opt_solutions;
        vector<spp_res_cont> pareto_opt;

        r_c_shortest_paths(G,
                           get(&SPPRC_Graph_Vert_Prop::num, G),
                           get(&SPPRC_Graph_Arc_Prop::num, G),
                           getDepot(),
                           getSink(),
                           opt_solutions,
                           pareto_opt,
                           spp_res_cont(0, 0),
                           ref_spprc(),
                           dominance_spptw(),
                           allocator<r_c_shortest_paths_label<SPPRC_Graph, spp_res_cont>>(),
                           default_r_c_shortest_paths_visitor());

        map<pair<int, int>, double> costs;
        double real_of = 0;

        cout << "Finished run of RCSPP" << endl;

        if (!pareto_opt.empty())
        {
            int last_elem = int(pareto_opt.size()) - 1;

            for (int j = 0; j < int(opt_solutions[last_elem].size()); j++)
            {
                auto arc = opt_solutions[last_elem][j];
                SPPRC_Graph_Arc_Prop &arc_prop = get(edge_bundle, G)[arc];

                pair<int, int> arc_pair = make_pair(source(arc, G), target(arc, G));
                x.insert(arc_pair);
                costs[arc_pair] = arc_prop.cost;
            }

            for (auto p : x)
                real_of += costs[p];
            return pareto_opt[last_elem].cost;
        }

        return numeric_limits<int>::max();
    }

    void BoostLibrary::updateBoostArcCost(int i, int j, boostGraph G, double new_cost)
    {
        edge_descriptor ed;
        bool found;

        boost::tie(ed, found) = boost::edge(i, j, G);

        if (found)
        {
            SPPRC_Graph_Arc_Prop &arc = get(edge_bundle, G)[ed];
            arc.cost = new_cost;
        }
    }
};

#endif
