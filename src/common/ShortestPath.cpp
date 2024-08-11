#include "ShortestPath.hpp"

int ShortestPath::SHPBetweenBlocks(int b1, int b2, set<int> &nodes, map<int, map<int, bool>> &arcs)
{
    auto nodes_from_b1 = graph->getNodesFromBlock(b1);
    auto nodes_from_b2 = graph->getNodesFromBlock(b2);

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
            int value = ShortestPathST(node, node2, path);

            if (value < shortest_path)
                shortest_path = value, shp = path;
        }
    }

    if (shortest_path < INF)
    {
        for (int i = 0; i < shp.size(); i++)
        {
            nodes.insert(shp[i]);
            if (i + 1 < shp.size())
                arcs[shp[i]][shp[i + 1]] = true;
        }
        return shortest_path;
    }

    return INF;
}

int ShortestPath::ShortestPathST(int s, int t, vector<int> &path)
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

int ShortestPath::DijkstraLayeredDAG(vector<vector<Arc>> dag, int n, int s, int t, vector<int> &pred)
{
    priority_queue<int_pair, vector<int_pair>, greater<int_pair>> pq;
    vector<int> distance(n, INF), predecessor(n, -1);

    pq.push(make_pair(0, s));
    distance[s] = 0, predecessor[s] = s;

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

    pred = predecessor;

    return distance[t];
}

void ShortestPath::allPairsShortestPath()
{
    int N = graph->getN();
    dist = vector<vector<int>>(N, vector<int>(N, INF));
    next = vector<vector<int>>(N, vector<int>(N, -1));
    ij_path = vector<vector<vector<int>>>(N, vector<vector<int>>(N, vector<int>{}));

    // Initialize the next matrix
    for (int i = 0; i < N; i++)
    {
        dist[i][i] = 0;
        for (auto arc : graph->getArcs(i))
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

vector<int> ShortestPath::getPath(int s, int t)
{
    if (s >= ij_path.size() || t >= ij_path[s].size())
        return vector<int>();

    if (!ij_path[s][t].empty())
        return ij_path[s][t];

    // cout << "Path to " << s << " to " << t << endl;
    vector<int> path = {s};
    int v = s;
    while (v != t)
    {
        v = next[v][t];
        path.push_back(v);
        // cout << "V: " << v << endl;
    }

    ij_path[s][t] = path;
    return path;
}