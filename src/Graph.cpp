//
// Created by carlos on 06/07/21.
//

#include "../headers/Graph.h"

const int INF = INT_MAX;

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

Graph::Graph(string instance, string scenarios, int graph_adapt, int km_path, int km_nebulize, int T, int s, float alpha)
{
  this->T = T;
  this->alpha = alpha;
  load_instance(instance, graph_adapt, km_path, km_nebulize, T);
  load_scenarios_instance(scenarios);

  if (s < S)
    this->S = s;

  if (graph_adapt == 1)
    reduceGraphToPositiveCases();
  else
  {
    nodes_per_block = backup_nodes_per_block;
    arcs_per_block = backup_arcs_per_block;
    cases_per_block = backup_cases_per_block;
    time_per_block = backup_time_per_block;
  }
}

void Graph::load_instance(string instance, int graph_adapt, int km_path, int km_nebulize, int T)
{
  int block, cases, i, j, k;
  float length;
  string token, aux, x, y;
  ifstream file;
  vector<string> blcs;

  file.open(instance, fstream::in);
  // Default Graph dimensions
  file >> Graph::N >> Graph::M >> Graph::B;

  // Basic structures
  boostGraph G = boostGraph(N);
  arcs = vector<vector<Arc *>>(N + 1, vector<Arc *>());
  arcs_matrix = vector<vector<Arc *>>(N + 2, vector<Arc *>(N + 2, nullptr));
  backup_nodes_per_block = vector<set<int>>(B, set<int>());
  backup_arcs_per_block = vector<vector<Arc *>>(B, vector<Arc *>());
  set<int> blocks_node;

  // Filtering blocks with positive amount of cases
  backup_cases_per_block = vector<float>(B, 0);
  backup_time_per_block = vector<int>(B, 0);

  // Load Graph
  double mp_path = (float(km_path * 1000) / 60.0);
  double mp_nebu = (float(km_nebulize * 1000) / 60.0);

  while (!file.eof())
  {
    file >> token;
    if (token == "N")
    {
      file >> j >> x >> y >> aux;
      boost::split(blcs, aux, boost::is_any_of(","));
      blocks_node = set<int>();

      for (auto s : blcs)
      {
        if (s == "-1")
          break;

        k = stoi(s);
        blocks_node.insert(k);
        backup_nodes_per_block[k].insert(j);
      }
      nodes.push_back(make_pair(j, blocks_node));
    }
    else if (token == "A")
    {
      file >> i >> j >> length >> block;

      if (token != "A")
        continue;

      file.ignore(numeric_limits<streamsize>::max(), '\n');
      int travel_time = length <= 0.0 ? 1 : 100.0 * (length / mp_path);

      Arc *arc = new Arc(i, j, travel_time, block);

      arcs[i].push_back(arc);
      if (block != -1)
      {
        backup_arcs_per_block[block].push_back(arc);
        backup_time_per_block[block] += length <= 0.0 ? 1 : 100.0 * (length / mp_nebu);
      }
      add_edge(i, j, travel_time, G);
    }
    else if (token == "B")
    {
      file >> block >> cases;
      backup_cases_per_block[block] = cases;
    }
  }
  nodes.push_back(make_pair(N, set<int>()));
  for (i = 0; i < N; i++)
    arcs[N].push_back(new Arc(N, i, 0, -1)), arcs[i].push_back(new Arc(i, N, 0, -1));

  cout << "Load graph successfully" << endl;
}

void Graph::AllPairsShortestPath()
{
  dist = vector<vector<int>>(N, vector<int>(N, INF));
  next = vector<vector<int>>(N, vector<int>(N, -1));
  ij_path = vector<vector<vector<int>>>(N, vector<vector<int>>(N, vector<int>{}));

  // Initialize the next matrix
  for (int i = 0; i < N; i++)
  {
    dist[i][i] = 0;
    for (auto arc : arcs[i])
    {
      int j = arc->getD();
      if (j >= N)
        continue;

      dist[i][j] = arc->getLength();
      next[i][j] = j;
    }
  }

  for (int k = 0; k < N; ++k)
  {
    for (int i = 0; i < N; ++i)
    {
      for (int j = 0; j < N; ++j)
      {
        if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j])
        {
          dist[i][j] = dist[i][k] + dist[k][j];
          next[i][j] = next[i][k];
        }
      }
    }
  }
}

int Graph::getShortestPath(int s, int t, vector<int> &path)
{
  if (dist[s][t] == INF)
    return INF;

  if (s == t)
  {
    path.push_back(s);
    return 0;
  }

  if (ij_path[s][t].size() > 0)
  {
    path = ij_path[s][t];
    return dist[s][t];
  }

  int i = s, j = t;
  path.push_back(i);
  while (i != j)
  {
    i = next[i][j];
    path.push_back(i);
  }
  ij_path[s][t] = path;

  return dist[s][t];
}

void Graph::reduceGraphToPositiveCases()
{
  // Re-map blocks
  positive_block_to_block = map<int, int>();

  int new_index = 0;

  for (int b = 0; b < B; b++)
  {
    bool has_cases = backup_cases_per_block[b] > 0;
    if (!has_cases)
    {
      for (int s = 0; s < S; s++)
      {
        if (scenarios[s].cases_per_block[b] > 0)
        {
          has_cases = true;
          break;
        }
      }
    }

    if (has_cases)
    {
      positive_block_to_block[b] = new_index++;
      cases_per_block.push_back(backup_cases_per_block[b]);
      time_per_block.push_back(backup_time_per_block[b]);
      arcs_per_block.push_back(backup_arcs_per_block[b]);
      nodes_per_block.push_back(backup_nodes_per_block[b]);
    }
    else
      positive_block_to_block[b] = -1;
  }

  PB = new_index;

  for (int s = 0; s < S; s++)
  {
    vector<float> cases_per_block_s(PB, 0);
    for (int b = 0; b < B; b++)
      if (positive_block_to_block[b] != -1)
        cases_per_block_s[positive_block_to_block[b]] = scenarios[s].cases_per_block[b];
    scenarios[s].cases_per_block = cases_per_block_s;
  }

  cout << "[*] Reduction of " << B - PB << " blocks" << endl;

  B = PB;
  // Re-map blocks in nodes and arcs
  for (int i = 0; i < N; i++)
  {
    auto blocks_from_i = nodes[i].second;
    nodes[i].second.clear();

    for (auto block : blocks_from_i)
    {
      if (block == -1)
      {
        nodes[i].second = set<int>{};
        break;
      }
      auto new_block = positive_block_to_block.find(block);
      if (new_block != positive_block_to_block.end() && new_block->second != -1)
        nodes[i].second.insert(new_block->second);
    }

    if (nodes[i].second.size() == 0)
      nodes[i].second = set<int>{};

    for (auto arc : arcs[i])
    {
      auto arc_new_block = positive_block_to_block.find(arc->getBlock());
      if (arc_new_block != positive_block_to_block.end())
        arc->setBlock(arc_new_block->second);
    }
  }

  // Get all pairs shortest Path
  AllPairsShortestPath();

  // Remove unecessary arcs between blocks
  block_2_block_shp = vector<vector<int>>(PB, vector<int>(PB, INF));
  block_2_block_shp_nodes = vector<vector<set<int>>>(PB, vector<set<int>>(PB, set<int>()));
  block_2_block_shp_arcs = vector<vector<vector<Arc *>>>(PB, vector<vector<Arc *>>(PB, vector<Arc *>()));

  set<int> set_of_used_nodes;
  map<int, map<int, bool>> map_used_arcs;

  for (int b = 0; b < PB - 1; b++)
  {
    for (int b2 = b + 1; b2 < PB; b2++)
    {
      set<int> nodes_in_path;
      map<int, map<int, bool>> arcs_in_path;
      int cost = SHPBetweenBlocks(b, b2, nodes_in_path, arcs_in_path);

      if (cost < INF)
      {
        set<int> union_set;
        set_union(nodes_in_path.begin(), nodes_in_path.end(), set_of_used_nodes.begin(), set_of_used_nodes.end(), inserter(union_set, union_set.begin()));
        set_of_used_nodes = union_set;
        block_2_block_shp[b][b2] = cost;
        block_2_block_shp[b2][b] = cost;

        for (auto it = arcs_in_path.begin(); it != arcs_in_path.end(); ++it)
          for (auto it2 = it->second.begin(); it2 != it->second.end(); ++it2)
            map_used_arcs[it->first][it2->first] = map_used_arcs[it2->first][it->first] = true;
      }
    }
  }

  int newN = 0;
  vector<vector<Arc *>> new_arcs;
  vector<pair<int, set<int>>> new_nodes = vector<pair<int, set<int>>>();
  map<int, int> map_new_nodes = map<int, int>();
  nodes_per_block = vector<set<int>>(PB);

  for (int i = 0; i < N; i++)
  {
    if (set_of_used_nodes.find(i) != set_of_used_nodes.end())
    {
      new_nodes.push_back(nodes[i]);
      new_nodes[newN].first = newN;
      map_new_nodes[i] = newN;

      for (int b : nodes[i].second)
        if (b != -1)
          nodes_per_block[b].insert(newN);

      new_arcs.push_back(vector<Arc *>());
      newN++;
    }
  }
  for (int i = 0; i < N; i++)
  {
    if (map_new_nodes.find(i) == map_new_nodes.end())
      continue;

    for (auto arc : arcs[i])
    {
      int new_o = map_new_nodes[i];

      if (map_new_nodes.find(arc->getD()) != map_new_nodes.end()) //(map_used_arcs[i][arc->getD()])
      {
        int new_d = map_new_nodes[arc->getD()];
        auto new_arc = new Arc(*arc);
        new_arc->setO(new_o), new_arc->setD(new_d);
        new_arcs[new_o].push_back(new_arc);
      }
    }
  }
  arcs = new_arcs;
  nodes = new_nodes;
  cout << "[*] Reduction of nodes from " << N << " to " << newN << endl;

  N = newN;
  nodes.push_back(make_pair(N, set<int>()));
  arcs.push_back(vector<Arc *>());
  for (int i = 0; i < N; i++)
    arcs[N].push_back(new Arc(N, i, 0, -1)), arcs[i].push_back(new Arc(i, N, 0, -1));

  // for (int i = 0; i < N; i++)
  // {
  //   cout << "N" << i << ": " << endl;
  //   for (auto block : nodes[i].second)
  //     cout << "B" << block << " [" << time_per_block[block] << ", " << cases_per_block[block] << "]; " << endl;
  //   cout << endl;
  // }

  // for (int b = 0; b < B; b++)
  // {
  //   cout << "Nodes from " << b << ": " << endl;
  //   for (auto node : nodes_per_block[b])
  //   {
  //     cout << node << ", ";
  //   }
  //   cout << endl;
  // }

  // cout << "[*] Reduction Finished!" << endl;
}

int Graph::SHPBetweenBlocks(int b1, int b2, set<int> &nodes, map<int, map<int, bool>> &arcs)
{
  auto nodes_from_b1 = nodes_per_block[b1];
  auto nodes_from_b2 = nodes_per_block[b2];

  if (nodes_from_b1.size() == 0 || nodes_from_b2.size() == 0)
    return INF;

  // Both blocks attended by the same node
  for (auto node : nodes_from_b1)
  {
    auto it = find(nodes_from_b2.begin(), nodes_from_b2.end(), node);
    if (it != nodes_from_b2.end())
    {
      nodes.insert(node);
      return 0;
    }
  }

  int shortest_path = INF;
  vector<int> shp;

  for (int node : nodes_from_b1)
  {
    for (int node2 : nodes_from_b2)
    {
      vector<int> path;
      int value = getShortestPath(node, node2, path);

      if (value < shortest_path)
        shortest_path = value, shp = path;
    }
  }

  // cout << "Shortest path between blocks " << b1 << " and " << b2 << ": " << shortest_path << endl;
  if (shortest_path < INF)
  {
    for (int i = 0; i < shp.size(); i++)
    {
      nodes.insert(shp[i]);
      if (i + 1 < shp.size())
      {
        arcs[shp[i]][shp[i + 1]] = true;
      }
    }
    return shortest_path;
  }

  return INF;
}

void Graph::reduceGraphToCompleteDigraphBlocks()
{
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
      vector<float> cases_per_block = vector<float>(Graph::B, 0);
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
      cout << "[" << i << ", " << arc->getD() << "] - " << arc->getBlock() << endl;
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
  if (cases.size() <= 0 || MT <= 1)
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

  for (i = s; i > 0; i--)
  {
    if (dp[i][w] != dp[i - 1][w])
    {
      y.push_back(i - 1);
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
    if (cases_per_block[b] <= 0)
      continue;
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

string Graph::generateStringFromIntVector(vector<int> blocks)
{
  stringstream result;
  copy(blocks.begin(), blocks.end(), ostream_iterator<int>(result, ""));

  return result.str();
}

int Graph::ConnectBlocks(vector<int> blocks, string key)
{
  vector<int> connect_order = vector<int>();
  vector<int> backup_blocks = blocks, path;
  vector<vector<Arc>> dag = vector<vector<Arc>>();
  map<int, int> dag_2_graph = map<int, int>();

  while (connect_order.size() < blocks.size())
  {
    if (connect_order.empty())
    {
      connect_order.push_back(blocks[0]);
      backup_blocks.erase(backup_blocks.begin());
    }

    int best_block = -1, shp = INF;
    for (int i = 0; i < int(backup_blocks.size()); i++)
    {
      if (block_2_block_shp[connect_order.back()][backup_blocks[i]] < shp)
      {
        shp = block_2_block_shp[connect_order.back()][backup_blocks[i]];
        best_block = i;

        if (shp == 0)
          break;
      }
    }

    if (best_block != -1)
    {
      connect_order.push_back(backup_blocks[best_block]);
      backup_blocks.erase(backup_blocks.begin() + best_block);
    }
  }

  // cout << "Connect order: " << endl;
  // for (auto b : connect_order)
  // {
  //   cout << b << " ";
  //   cout << endl;
  // }

  // Create DAG

  int V = 0;
  for (int i = 0; i < connect_order.size(); i++)
    V += nodes_per_block[connect_order[i]].size();
  dag = vector<vector<Arc>>(V + 2, vector<Arc>());

  // cout << "V: " << V << endl;
  dag_2_graph[V] = dag_2_graph[V + 1] = getN();

  int inserted_nodes = 0;
  bool insert_depot = false;
  for (int i = 0; i < connect_order.size() - 1; i++)
  {
    int b1 = connect_order[i];
    int b2 = connect_order[i + 1];

    int jp = inserted_nodes;
    int start_k = jp + nodes_per_block[b1].size();

    if (i + 1 >= connect_order.size() - 1)
      insert_depot = true;

    for (int j : nodes_per_block[b1])
    {
      // Dummy depot
      if (i == 0)
        dag[V].push_back(Arc(V, jp, 0, 0));

      dag_2_graph[jp] = j;
      int kp = start_k;

      for (int k : nodes_per_block[b2])
      {
        dag_2_graph[kp] = k;
        Arc arc = Arc(jp, kp, 0, 0);

        if (j != k)
          arc.setLength(getShortestPath(j, k, path));

        dag[jp].push_back(arc);

        if (insert_depot)
          dag[kp].push_back(Arc(kp, V + 1, 0, 0));

        kp++;
      }
      if (i + 1 >= connect_order.size() - 1)
        insert_depot = false;

      jp++;
    }
    inserted_nodes += nodes_per_block[b1].size();
  }

  // for (int i = 0; i < dag.size(); i++)
  //   for (auto arc : dag[i])
  //     cout << i << " -> " << arc.getD() << " (" << arc.getLength() << ")" << endl;
  // SHP on DAG
  int cost = DijkstraDAG(V + 2, dag, key, dag_2_graph);
  this->savedBlockConn[key] = cost;
  return cost;
}

int Graph::DijkstraDAG(int N, vector<vector<Arc>> dag, string key, map<int, int> dag_2_graph)
{
  priority_queue<dpair, vector<dpair>, greater<dpair>> pq;
  int n = N;
  int s = n - 2, t = n - 1;

  vector<int> distance(n, INF);
  vector<int> predecessor(n, -1);

  pq.push(make_pair(0, s));
  distance[s] = 0;
  predecessor[s] = s;

  while (!pq.empty())
  {
    int u = pq.top().second;
    pq.pop();

    for (auto arc : dag[u])
    {
      int v = arc.getD();

      if (distance[v] > distance[u] + arc.getLength())
      {
        distance[v] = distance[u] + arc.getLength();
        predecessor[v] = u;
        pq.push(make_pair(distance[v], v));
      }
    }
  }

  // Get the path
  int v = t;
  vector<int> path;
  int last_inserted = -1;
  while (v != predecessor[v])
  {
    if (dag_2_graph[v] != last_inserted)
    {
      path.push_back(dag_2_graph[v]);
      last_inserted = dag_2_graph[v];
    }
    v = predecessor[v];

    if (v == -1)
      return INF;
  }
  path.push_back(dag_2_graph[s]);

  this->savedBlockConnPath[key] = path;
  return distance[t];
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

float Graph::getAlpha() const
{
  return alpha;
}