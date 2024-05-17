//
// Created by carlos on 06/07/21.
//

#include "../headers/Graph.h"

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
    i_time = old_cont.time + arc_prop.time;
    return i_time <= vert_prop.lim ? true : false;
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

Graph::Graph(string instance, int graph_adapt, int km_path, int km_nebulize, int T)
{
  this->T = T;
  load_instance(instance, graph_adapt, km_path, km_nebulize, T);
}

// Graph adapt
// 0 - No adapt
// 1 - Create reverse missing arcs
// 2 - Complete Digraph
void Graph::load_instance(string instance, int graph_adapt, int km_path, int km_nebulize, int T)
{
  int block, cases, i, j, k;
  float length;
  string token, aux, x, y;
  ifstream file;
  vector<string> blcs;

  file.open(instance, fstream::in);

  file >> Graph::N >> Graph::M >> Graph::B;
  arcs = vector<vector<Arc *>>(N + 1, vector<Arc *>());
  nodes_per_block = vector<set<int>>(B, set<int>());
  arcs_per_block = vector<vector<Arc *>>(B, vector<Arc *>());
  p_blocks = vector<int>(B, -1);
  set<int> blocks_node;
  cases_per_block = vector<int>();
  cases_block = map<int, int>();

  // Reading Blocks from file
  for (int b = 0; b < B; b++)
    cases_block[b] = -1;

  while (!file.eof())
  {
    file >> token;

    if (token == "B")
    {
      file >> i >> j;
      cases_block[i] = j;
      p_blocks[i] = PB++;
    }
  }

  // Filtering blocks with positive amount of cases
  cases_per_block = vector<int>(PB, 0);
  time_per_block = vector<int>(PB, 0);

  for (i = 0; i < B; i++)
  {
    int mapped_block = p_blocks[i];
    if (mapped_block != -1)
      cases_per_block[mapped_block] = cases_block[i];
  }

  // Restarting the read
  file.clear(); // clear fail and eof bits
  file.seekg(0, std::ios::beg);
  file >> token >> token >> token;

  // Create nodes
  for (i = 0; i < N; i++)
  {
    file >> token >> j >> x >> y >> aux;

    if (token != "N")
      continue;

    boost::split(blcs, aux, boost::is_any_of(","));
    blocks_node = set<int>();

    for (auto s : blcs)
    {
      if (s == "-1")
        break;
      k = stoi(s);
      block = p_blocks[k];

      if (block != -1)
      {
        blocks_node.insert(block);
        nodes_per_block[block].insert(j);
      }
    }
    nodes.push_back(make_pair(j, blocks_node));
    boost::add_vertex(SPPRC_Graph_Vert_Prop(j, T), G);
  }

  // Creating arcs
  double mp_path = ((km_path * 1000) / 60);
  double mp_nebu = ((km_nebulize * 1000) / 60);

  for (k = 0; k < M; k++)
  {
    file >> token >> i >> j >> length >> block;

    if (token != "A")
      continue;

    int new_block = -1;
    if (block != -1)
      new_block = p_blocks[block];

    file.ignore(numeric_limits<streamsize>::max(), '\n');
    int travel_time = length <= 0 ? 0 : round(10 * (length / mp_path));

    Arc *arc = new Arc(i, j, travel_time, new_block);

    if ((i == 0 && j == 3) || (i == 3 && j == 9))
      boost::add_edge(i, j, SPPRC_Graph_Arc_Prop(k, -1, travel_time), G);
    else
      boost::add_edge(i, j, SPPRC_Graph_Arc_Prop(k, 0, travel_time), G);

    arcs[i].push_back(arc);
    if (new_block != -1)
    {
      arcs_per_block[new_block].push_back(arc);
      time_per_block[new_block] += length <= 0 ? 0 : round(10 * (length / mp_nebu));
    }
  }

  for (int b = 0; b < PB; b++)
    cout << "B" << b << " = " << time_per_block[b] << ", " << cases_per_block[b] << endl;

  nodes.push_back(make_pair(N, set<int>()));
  boost::add_vertex(SPPRC_Graph_Vert_Prop(N, T), G);
  boost::add_vertex(SPPRC_Graph_Vert_Prop(N + 1, T), G);
  k = M;

  for (i = 0; i < N; i++)
  {
    arcs[N].push_back(new Arc(N, i, 0, -1));
    arcs[i].push_back(new Arc(i, N + 1, 0, -1));

    boost::add_edge(N, i, SPPRC_Graph_Arc_Prop(k++, 0, 0), G);
    boost::add_edge(i, N + 1, SPPRC_Graph_Arc_Prop(k++, 0, 0), G);
  }

  cout << "Load graph successfully" << endl;
}

bool Graph::exist_arc(int i, int j)
{
  if (i > N)
    return false;
  for (auto *arc : arcs[i])
  {
    if (arc->getD() == j)
    {
      return true;
    }
  }
  return false;
}

void Graph::showGraph()
{
  for (int i = 0; i <= N; i++)
    for (auto *arc : arcs[i])
      cout << "[" << i << ", " << arc->getD() << "] - " << arc->getLength() << ", " << arc->getBlock() << endl;
}

double Graph::run_spprc(vector<pair<int, int>> &x)
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

  if (!pareto_opt.empty())
  {
    int last_elem = int(pareto_opt.size()) - 1;
    for (int j = 0; j < int(opt_solutions[last_elem].size()); j++)
    {
      auto arc = opt_solutions[last_elem][j];
      x.push_back(make_pair(source(arc, G), target(arc, G)));
    }

    return pareto_opt[last_elem].cost;
  }

  return INFINITY;
}

double Graph::knapsack(vector<int> &y, vector<int> cases, vector<int> time, int MT)
{
  int i, w;
  int s = cases.size();
  int dp[s + 1][MT + 1];

  // for (int i = 0; i < cases.size(); i++)
  //   cout << cases[i] << " -> " << time[i] << ", ";
  // cout << endl;

  for (i = 0; i <= s; i++)
  {
    for (w = 0; w <= MT; w++)
    {
      if (i == 0 || w == 0)
        dp[i][w] = 0;
      else if (time[i - 1] <= w)
        dp[i][w] = max(dp[i - 1][w], dp[i - 1][w - time[i - 1]] + cases[i - 1]);
      else
        dp[i][w] = dp[i - 1][w];
    }
  }

  // Retrieving the items
  int res = dp[s][MT];
  w = MT;

  for (i = s; i > 0 && res > 0; i--)
  {
    if (res == dp[i - 1][w])
      continue;
    else
    {
      y.push_back(i - 1);

      res -= cases[i - 1];
      w -= time[i - 1];
    }
  }
  return dp[s][MT];
}

set<int> Graph::getBlocksFromRoute(vector<pair<int, int>> x)
{
  int i;
  set<int> blocks;

  for (auto p : x)
  {
    i = p.first;
    for (auto b : this->nodes[i].second)
      if (b != -1)
        blocks.insert(b);
  }

  return blocks;
}

void Graph::populateKnapsackVectors(set<int> blocks, vector<int> &cases, vector<int> &time)
{
  for (auto b : blocks)
  {
    cases.push_back(cases_per_block[b]);
    time.push_back(time_per_block[b]);
  }
}

int Graph::getN() const
{
  return N;
}

int Graph::getM() const
{
  return M;
}

int Graph::getB() const
{
  return B;
}

int Graph::getPB() const
{
  return PB;
}

int Graph::getDepot() const
{
  return N;
}

int Graph::getSink() const
{
  return N + 1;
}
