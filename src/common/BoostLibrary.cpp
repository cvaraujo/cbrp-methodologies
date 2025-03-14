#include "BoostLibrary.hpp"

struct spp_res_cont
{
    spp_res_cont(float c = 0, int t = 0) : cost(c), time(t) {}
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
};

// ResourceExtensionFunction model
class ref_spprc
{
public:
    inline bool operator()(const SPPRC_Graph &g, spp_res_cont &new_cont, const spp_res_cont &old_cont, graph_traits<SPPRC_Graph>::edge_descriptor ed) const
    {
        const SPPRC_Graph_Arc_Prop &arc_prop = get(edge_bundle, g)[ed];
        const SPPRC_Graph_Vert_Prop &vert_prop = get(vertex_bundle, g)[target(ed, g)];

        new_cont.cost = old_cont.cost + arc_prop.cost;
        int &i_time = new_cont.time;
        i_time = old_cont.time + arc_prop.time;

        return i_time <= vert_prop.time;
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

BoostLibrary::BoostLibrary(Input *input)
{
    this->input = input;
    this->set_boost_graph();
}

void BoostLibrary::set_boost_graph()
{
    Graph *graph = input->getGraph();
    G = SPPRC_Graph();
    int N = graph->getN(), T = input->getT();

    for (int i = 0; i <= N + 1; i++)
        add_vertex(SPPRC_Graph_Vert_Prop(i, T), G);

    int num_arcs = 0;
    for (int i = 0; i < N; i++)
        for (auto arc : graph->getArcs(i))
            add_edge(i, arc->getD(), SPPRC_Graph_Arc_Prop(num_arcs++, 0.0, arc->getLength()), G);

    for (int i = 0; i < N; i++)
        add_edge(N + 1, i, SPPRC_Graph_Arc_Prop(num_arcs++, 0.0, 0), G);
}

void BoostLibrary::update_arc_cost(int i, int j, double cost)
{
    edge_descriptor ed;
    bool found;
    tie(ed, found) = edge(i, j, G);

    if (found)
    {
        SPPRC_Graph_Arc_Prop &arc_prop = get(edge_bundle, G)[ed];
        arc_prop.cost = cost;
    }
    else
    {
        cout << "No edge from " << i << " to " << j << endl;
        exit(1);
    }
}

pair<int, double> BoostLibrary::run_spprc(set<pair<int, int>> &x)
{
    // Run the shortest path with resource constraints
    vector<edge_descriptor> opt_solution;
    spp_res_cont pareto_opt;

    int N = this->input->getGraph()->getN();
    int s = N + 1, t = N;
    cout << "[*] Running SPPRC..." << endl;

    r_c_shortest_paths(G,
                       get(&SPPRC_Graph_Vert_Prop::num, G),
                       get(&SPPRC_Graph_Arc_Prop::num, G),
                       s,
                       t,
                       opt_solution,
                       pareto_opt,
                       spp_res_cont(0, 0),
                       ref_spprc(),
                       dominance_spptw());

    int route_cost = numeric_limits<int>::max();
    double route_time = 0.0;
    cout << "\t[*] Route Cost: " << pareto_opt.cost << endl;
    cout << "\t[*] Route Time: " << pareto_opt.time << endl;

    int last_size = 0;
    if (!opt_solution.empty())
    {
        route_cost = 0.0;
        for (int j = 0; j < int(opt_solution.size()); j++)
        {
            auto arc = opt_solution[j];
            SPPRC_Graph_Arc_Prop &arc_prop = get(edge_bundle, G)[arc];
            pair<int, int> arc_p = make_pair(source(arc, G), target(arc, G));
            x.insert(arc_p);
            if (x.size() > last_size)
            {
                last_size = x.size();
                route_cost += arc_prop.cost;
            }
        }
    }
    return make_pair(pareto_opt.time, pareto_opt.cost);
}
