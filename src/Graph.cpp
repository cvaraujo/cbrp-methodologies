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

Graph::Graph(string instance, string scenarios, int graph_adapt, int km_path, int km_nebulize, int T)
{
  this->T = T;
  load_instance(instance, graph_adapt, km_path, km_nebulize, T);
  load_scenarios_instance(scenarios);
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
  arcs_matrix = vector<vector<Arc *>>(N + 2, vector<Arc *>(N + 2, nullptr));
  nodes_per_block = vector<set<int>>(B, set<int>());
  arcs_per_block = vector<vector<Arc *>>(B, vector<Arc *>());
  p_blocks = vector<int>(B, -1);
  set<int> blocks_node;
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
  cases_per_block = vector<float>(B, 0);
  positive_cases_per_block = vector<int>(PB, 0);
  time_per_block = vector<int>(B, 0);

  for (i = 0; i < B; i++)
  {
    int mapped_block = p_blocks[i];
    if (mapped_block != -1)
    {
      positive_cases_per_block[mapped_block] = cases_block[i];
      cases_per_block[i] = cases_block[i];
    }
    else
    {
      cases_per_block[i] = 0;
    }
  }

  cout << "Read blocks" << endl;
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
      blocks_node.insert(k);
      nodes_per_block[k].insert(j);
    }
    nodes.push_back(make_pair(j, blocks_node));
    boost::add_vertex(SPPRC_Graph_Vert_Prop(j, T, PB + 1), G);
  }
  cout << "Read Nodes" << endl;

  // Creating arcs
  double mp_path = ((km_path * 1000) / 60);
  double mp_nebu = ((km_nebulize * 1000) / 60);

  for (k = 0; k < M; k++)
  {
    file >> token >> i >> j >> length >> block;

    if (token != "A")
      continue;

    file.ignore(numeric_limits<streamsize>::max(), '\n');
    int travel_time = length <= 0 ? 1 : 10 * (length / mp_path);

    Arc *arc = new Arc(i, j, travel_time, block);
    boost::add_edge(i, j, SPPRC_Graph_Arc_Prop(k, 0, travel_time, 1), G);

    arcs[i].push_back(arc);
    if (block != -1)
    {
      arcs_per_block[block].push_back(arc);
      time_per_block[block] += length <= 0 ? 0 : 10 * (length / mp_nebu);
    }
  }

  cout << "Read Arcs" << endl;
  nodes.push_back(make_pair(N, set<int>()));
  boost::add_vertex(SPPRC_Graph_Vert_Prop(N, T, PB + 1), G);
  boost::add_vertex(SPPRC_Graph_Vert_Prop(N + 1, T, PB + 1), G);
  k = M;

  for (i = 0; i < N; i++)
  {
    arcs[N].push_back(new Arc(N, i, 0, -1));
    arcs[i].push_back(new Arc(i, N, 0, -1));

    boost::add_edge(N, i, SPPRC_Graph_Arc_Prop(k++, 0, 0, 1), G);
    boost::add_edge(i, N + 1, SPPRC_Graph_Arc_Prop(k++, 0, 0, 1), G);
  }

  cout << "Load graph successfully" << endl;
}

void Graph::load_scenarios_instance(string instance)
{
  string token;
  ifstream file;
  int i, block, cases;
  float prob;
  file.open(instance, fstream::in);
  file >> Graph::S;
  Graph::scenarios = vector<Scenario>(Graph::S);

  while (!file.eof())
  {
    file >> token;
    if (token == "P")
    {
      file >> i >> prob;
      vector<int> cases_per_block = vector<int>(Graph::B, 0);
      Scenario scn(prob, cases_per_block);
      Graph::scenarios[i] = scn;
    }
    else if (token == "B")
    {
      file >> i >> block >> cases;
      Graph::scenarios[i].SetCases(block, cases);
    }
  }
  cout << "Load Scenarios successfully" << endl;
}

void Graph::showGraph()
{
  for (int i = 0; i <= N; i++)
    for (auto *arc : arcs[i])
      cout << "[" << i << ", " << arc->getD() << "] - " << arc->getLength() << ", " << arc->getBlock() << endl;
}

void Graph::showScenarios()
{
  for (int i = 0; i < S; i++)
  {
    cout << "Scenario i: " << i << ": " << scenarios[i].probability << endl;
    for (int b = 0; b < B; b++)
    {
      if (scenarios[i].cases_per_block[b] > 0)
        cout << b << ": " << scenarios[i].cases_per_block[b] << endl;
    }
  }
}

double Graph::run_spprc(set<pair<int, int>> &x)
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

double Graph::knapsack(vector<int> &y, vector<double> cases, vector<int> time, int MT)
{
  if (cases.empty() || MT <= 1)
    return 0;

  int i, w;
  int s = cases.size();
  double dp[s + 1][MT + 1];

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

set<int> Graph::getBlocksFromRoute(set<pair<int, int>> x)
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

void Graph::populateKnapsackVectors(set<int> blocks, vector<double> &cases, vector<int> &time)
{
  for (auto b : blocks)
  {
    cases.push_back(cases_per_block[b]);
    time.push_back(time_per_block[b]);
  }
}

void Graph::updateBoostArcCost(int i, int j, double new_cost)
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

Arc *Graph::getArc(int i, int j)
{
  if (arcs_matrix[i][j] != nullptr)
    return arcs_matrix[i][j];

  for (auto arc : arcs[i])
    if (arc->getD() == j)
    {
      arcs_matrix[i][j] = arc;
      return arc;
    }

  return nullptr;
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

int Graph::getT() const
{
  return T;
}

int Graph::getS() const
{
  return S;
}

int Graph::getDepot() const
{
  return N;
}

int Graph::getSink() const
{
  return N + 1;
}

void Graph::setS(int s)
{
  this->S = s;
}