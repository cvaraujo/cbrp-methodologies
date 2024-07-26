//
// Created by carlos on 06/07/21.
//

#include "Graph.hpp"

Graph::Graph(string instance, int km_path, int km_nebulize)
{
  if (instance != "")
    LoadGraph(instance, km_path, km_nebulize);
  else
    exit(EXIT_FAILURE);
}

void Graph::LoadGraph(string instance, int km_path, int km_nebulize)
{
  int block, cases, i, j, k;
  double length;
  string token, aux, x, y;
  ifstream file;
  vector<string> blcs;

  file.open(instance, fstream::in);

  // Default Graph dimensions
  file >> Graph::N >> Graph::M >> Graph::B;

  // Basic structures
  arcs = vector<vector<Arc *>>(N + 1, vector<Arc *>());
  arcs_matrix = vector<vector<Arc *>>(N + 2, vector<Arc *>(N + 2, nullptr));
  nodes_per_block = vector<set<int>>(B, set<int>());
  arcs_per_block = vector<vector<Arc *>>(B, vector<Arc *>());
  set<int> blocks_node;

  // Filtering blocks with positive amount of cases
  cases_per_block = vector<double>(B, 0);
  time_per_block = vector<int>(B, 0);

  // Load Graph
  double mp_path = (double(km_path * 1000) / 60.0);
  double mp_nebu = (double(km_nebulize * 1000) / 60.0);

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
        nodes_per_block[k].insert(j);
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

      arcs_matrix[i][j] = arc;
      arcs[i].push_back(arc);
      if (block != -1)
      {
        arcs_per_block[block].push_back(arc);
        time_per_block[block] += length <= 0.0 ? 1 : 100.0 * (length / mp_nebu);
      }
    }
    else if (token == "B")
    {
      file >> block >> cases;
      cases_per_block[block] = cases;
    }
  }

  nodes.push_back(make_pair(N, set<int>()));
  for (i = 0; i < N; i++)
  {
    Arc *from_s = new Arc(N, i, 0, -1);
    Arc *to_t = new Arc(i, N, 0, -1);
    arcs[N].push_back(from_s), arcs[i].push_back(to_t);
    arcs_matrix[N][i] = from_s, arcs_matrix[i][N] = to_t;
  }

#ifdef Silence
  cout << "[*] Load graph successfully!" << endl;
#endif

  file.close();
}
