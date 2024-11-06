//
// Created by carlos on 06/07/21.
//

#include "DeterministicModel.hpp"

class cyclecallback : public GRBCallback
{

public:
  double lastiter, lastnode;
  int numvars, cuts = 0, num_frac_cuts = 0, num_lazy_cuts = 0;
  bool frac_cut = false;
  vector<vector<GRBVar>> x, y;
  typedef ListDigraph G;
  typedef G::Arc Arc;
  typedef G::ArcIt ArcIt;
  typedef G::Node Node;
  typedef G::ArcMap<double> LengthMap;
  typedef G::NodeMap<bool> BoolNodeMap;
  Input *input;

  cyclecallback(Input *xinput, int xnumvars, vector<vector<GRBVar>> xx, vector<vector<GRBVar>> yy, bool frac_cut)
  {
    lastiter = lastnode = 0;
    numvars = xnumvars;
    x = xx;
    y = yy;
    input = xinput;
    this->frac_cut = frac_cut;
  }

protected:
  void callback()
  {
    if (where == GRB_CB_MIPSOL)
    {
      try
      {
        bool is_feasible = true;
        Graph *graph = input->getGraph();

        int i, j, s, n = graph->getN();
        vector<vector<int>> g = vector<vector<int>>(n + 2, vector<int>());
        vector<bool> used_node = vector<bool>(n + 1);

        // Create graph G'
        for (i = 0; i <= n; i++)
        {
          for (auto *arc : graph->getArcs(i))
          {
            if (getSolution(x[i][arc->getD()]) > 0.1)
            {
              g[i].push_back(arc->getD());
              used_node[i] = used_node[arc->getD()] = true;
            }
          }
        }

        vector<bool> visited(n + 1, false);
        vector<int> node_connected_component = vector<int>(n + 1, -1);
        vector<vector<int>> connected_component;
        vector<vector<int_pair>> arcs_from_component;
        int idx = 0;

        // DFS
        for (i = n; i >= 0; i--)
        {
          if (!used_node[i] || visited[i])
            continue;

          connected_component.push_back(vector<int>());
          arcs_from_component.push_back(vector<int_pair>());

          vector<int> stack;
          stack.push_back(i);

          while (!stack.empty())
          {
            s = stack.back();
            stack.pop_back();

            if (!visited[s])
            {
              connected_component[idx].push_back(s);
              node_connected_component[s] = idx;
              visited[s] = true;
            }

            for (auto k : g[s])
            {
              if (!visited[k])
                stack.push_back(k);
              arcs_from_component[idx].push_back(make_pair(s, k));
            }
          }
          idx++;
        }

        // Feasible solution
        if (idx == 1)
          return;

        // Need Cuts
        is_feasible = false;

        if (input->isTrail())
        {
          for (i = 1; i < connected_component.size(); i++)
          {
            vector<int> s_nodes = connected_component[i];
            vector<int_pair> s_arcs = arcs_from_component[i];
            GRBLinExpr in_arcs, cut_arcs;
            int num_in_arcs = s_arcs.size();

            // Arcs inside S
            for (auto pair : s_arcs)
              in_arcs += x[pair.first][pair.second];

            addLazy(in_arcs <= num_in_arcs - 1);
            num_lazy_cuts++;
          }
        }
        else
        {
          // Need Cuts
          for (i = 1; i < connected_component.size(); i++)
          {
            vector<int> s_nodes = connected_component[i];
            vector<int_pair> s_arcs = arcs_from_component[i];
            GRBLinExpr in_arcs, cut_arcs;
            int num_in_arcs = s_arcs.size();

            // Arcs inside S
            for (auto pair : s_arcs)
              in_arcs += x[pair.first][pair.second];

            // Arcs in the cut S
            for (int j = 0; j < n; j++)
              if (node_connected_component[j] != i)
                for (auto arc : graph->getArcs(j))
                  if (node_connected_component[arc->getD()] == i)
                    cut_arcs += x[j][arc->getD()];

            addLazy(in_arcs <= num_in_arcs - 1 + cut_arcs);
            num_lazy_cuts++;
          }
        }
        if (is_feasible)
          return;
      }
      catch (GRBException e)
      {
        cout << "[LAZZY] Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
      }
      catch (...)
      {
        cout << "Error during callback" << endl;
      }
    }

    else if (where == GRB_CB_MIPNODE)
    {
      try
      {
        if (!frac_cut)
          return;

        int mipStatus = getIntInfo(GRB_CB_MIPNODE_STATUS);

        if (mipStatus == GRB_OPTIMAL)
        {
          Graph *graph = input->getGraph();
          int i, j, u, v, n = graph->getN();

          // Basic structures to use Lemon
          G flow_graph;
          LengthMap capacity(flow_graph);
          vector<Node> set_nodes = vector<Node>(n + 1);
          vector<bool> used_node = vector<bool>(n, false);
          vector<Arc> set_arcs;

          // Create the node set
          for (i = 0; i <= n; i++)
            set_nodes[i] = flow_graph.addNode();

          // Create the edge set
          for (i = 0; i <= n; i++)
          {
            for (auto *arc : graph->getArcs(i))
            {
              j = arc->getD();

              if (getNodeRel(x[i][j]) > 0)
              {
                set_arcs.push_back(flow_graph.addArc(set_nodes[i], set_nodes[j]));
                capacity[set_arcs[set_arcs.size() - 1]] = double(getNodeRel(x[i][j]));
                used_node[i] = used_node[j] = true;
              }
            }
          }

          // Init necessary structures
          double mincut_value;
          for (i = 0; i < n; i++)
          {
            // If there is no arc using this node, ignore it
            if (!used_node[i])
              continue;

            // Lemon MaxFlow instance
            Preflow<G, LengthMap> preflow(flow_graph, capacity, set_nodes[n], set_nodes[i]);
            preflow.runMinCut();
            mincut_value = preflow.flowValue();

            if (mincut_value >= 1.0)
              continue;

            // Create basic variables
            GRBLinExpr cut_arcs, other_side_arcs;
            double cut_value = 0, other_side_value = 0;
            int num_arcs_in_other_side = 0;

            // Get S and cut arcs (values)
            for (j = 0; j <= n; j++)
            {
              if (preflow.minCut(set_nodes[j]))
              {
                for (auto arc : graph->getArcs(j))
                {
                  if (!preflow.minCut(set_nodes[arc->getD()]))
                  {
                    cut_arcs += x[j][arc->getD()];
                    cut_value += getNodeRel(x[j][arc->getD()]);
                  }
                }
              }
              else
              {
                for (auto arc : graph->getArcs(j))
                {
                  if (!preflow.minCut(set_nodes[arc->getD()]))
                  {
                    other_side_arcs += x[j][arc->getD()];
                    other_side_value += getNodeRel(x[j][arc->getD()]);
                    num_arcs_in_other_side++;
                  }
                }
              }
            }

            // Add cut
            if (num_arcs_in_other_side <= 0 || other_side_value <= cut_value)
              continue;

            if (!input->isTrail())
            {
              addCut(other_side_arcs <= num_arcs_in_other_side - 1 + cut_arcs);
              num_frac_cuts++;
            }
            else
            {
              addCut(other_side_arcs <= num_arcs_in_other_side - 1);
              num_frac_cuts++;
            }
          }
        }
      }
      catch (GRBException e)
      {
        cout << "[FRAC] Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
      }
      catch (...)
      {
        cout << "Error during callback" << endl;
      }
    }
  }
};

Solution DeterministicModel::Run(bool use_warm_start, string time_limit, string model, bool use_cuts)
{
  this->createVariables();
  this->initModel(model);

  if (model == "MTZ")
    this->solveCompact(time_limit);
  else if (model == "EXP")
    this->solveExponential(time_limit, use_cuts);
  else
  {
    cout << "[!] Model not found!" << endl;
    exit(EXIT_FAILURE);
  }

  this->checkSolution();

  return this->getSolution();
}

void DeterministicModel::createVariables()
{
  Graph *graph = this->input->getGraph();
  int o, d, k, n = graph->getN(), m = graph->getM(), b = graph->getB();

  try
  {
    env.set("LogFile", "MS_mip.log");
    env.start();

    x = vector<vector<GRBVar>>(n + 1, vector<GRBVar>(n + 1));
    y = vector<vector<GRBVar>>(n, vector<GRBVar>(b));
    t = vector<vector<GRBVar>>(n + 1, vector<GRBVar>(n + 1));

    // X
    char name[40];
    for (o = 0; o <= n; o++)
    {
      for (auto *arc : graph->getArcs(o))
      {
        d = arc->getD();
        sprintf(name, "x_%d_%d", o, d);
        x[o][d] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
      }
    }
    // Y
    for (int i = 0; i < n; i++)
    {
      o = graph->getNodes()[i].first;
      for (auto bl : graph->getNode(i).second)
      {
        sprintf(name, "y_%d_%d", o, bl);
        y[o][bl] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
      }
    }

    // T
    for (o = 0; o <= n; o++)
    {
      for (auto *arc : graph->getArcs(o))
      {
        d = arc->getD();
        sprintf(name, "t_%d_%d", o, d);
        t[o][d] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
      }
    }

    model.update();
#ifndef Silence
    cout << "[*] Create variables" << endl;
#endif
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
    cout << ex.getErrorCode() << endl;
    exit(EXIT_FAILURE);
  }
}

void DeterministicModel::initModel(string model)
{
#ifndef Silence
  cout << "[***] Creating " << model << " model!" << endl;
#endif

  objectiveFunction();
  artificialNodes(), flowConservation();
  maxAttending(), attendingPath();
  timeConstraint();
  if (model == "MTZ")
    compactTimeConstraint();

  this->model.update();

#ifndef Silence
  cout << "[***] Constraints and objective function created!" << endl;
#endif
}

/*
void DeterministicModel::WarmStart()
{
  int i, j;

  // Warm start
  vector<pair<int, int>> x, y;
  double of = WarmStart::compute_solution(graph, graph->getT(), x, y);

  cout << "[***] Heuristic value = " << of << endl;

  for (int i = 0; i <= graph->getN(); i++)
  {
    for (auto *arc : graph->arcs[i])
    {
      bool is_in = false;

      for (auto pair : x)
      {
        int k = pair.first, j = pair.second;
        if (k == i && j == arc->getD())
        {
          is_in = true;
          break;
        }
      }
      if (is_in)
      {
        this->x[i][arc->getD()].set(GRB_DoubleAttr_Start, 1.0);
      }
      else
        this->x[i][arc->getD()].set(GRB_DoubleAttr_Start, 0.0);
    }
  }

  model.update();

  for (auto pair : y)
  {
    i = pair.first, j = pair.second;
    this->y[i][j].set(GRB_DoubleAttr_Start, 1.0);
  }

  model.update();
  cout << "Warm Start done!" << endl;
}
*/

void DeterministicModel::objectiveFunction()
{
  GRBLinExpr objective;
  auto graph = input->getGraph();
  int i, j, n = graph->getN();

  for (i = 0; i < n; i++)
  {
    j = graph->getNode(i).first;
    for (auto b : graph->getNode(i).second)
    {
      objective += (y[j][b] * graph->getCasesPerBlock(b));
    }
  }

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();

#ifndef Silence
  cout << "[***] Obj. Function: Maximize profit" << endl;
#endif
}

void DeterministicModel::artificialNodes()
{
  int n = this->input->getGraph()->getN();
  GRBLinExpr sink, target;

  for (int i = 0; i < n; i++)
  {
    sink += x[n][i];
    target += x[i][n];
  }

  model.addConstr(sink == 1, "sink_constraint");
  model.addConstr(target == 1, "target_constraint");

#ifndef Silence
  cout << "[***] Contraint: dummy depot" << endl;
#endif
}

void DeterministicModel::flowConservation()
{
  auto graph = input->getGraph();
  int i, j, n = graph->getN();

  for (i = 0; i < n; i++)
  {
    GRBLinExpr flow_out, flow_in;

    for (auto *arc : graph->getArcs(i))
    {
      if (arc->getD() >= n)
        continue;
      flow_out += x[i][arc->getD()];
    }

    for (j = 0; j < n; j++)
    {
      for (auto *arc : graph->getArcs(j))
      {
        if (arc->getD() == i)
          flow_in += x[j][i];
      }
    }

    flow_out += x[i][n];
    flow_in += x[n][i];
    model.addConstr(flow_in - flow_out == 0, "flow_conservation_" + to_string(i));
  }

#ifndef Silence
  cout << "[***] Constraint: Flow conservation" << endl;
#endif
}

void DeterministicModel::maxAttending()
{
  auto graph = input->getGraph();
  int bl, b = graph->getB();

  for (bl = 0; bl < b; bl++)
  {
    GRBLinExpr maxServ;
    for (auto i : graph->getNodesFromBlock(bl))
      maxServ += y[i][bl];

    model.addConstr(maxServ <= 1, "max_service_block_" + to_string(bl));
  }

#ifndef Silence
  cout << "[***] Constraint: Serve each block at most once" << endl;
#endif
}

void DeterministicModel::attendingPath()
{
  auto graph = input->getGraph();
  int j, bl, n = graph->getN(), b = graph->getB();
  for (bl = 0; bl < b; bl++)
  {
    for (auto i : graph->getNodesFromBlock(bl))
    {
      GRBLinExpr served;
      for (auto *arc : graph->getArcs(i))
        served += x[i][arc->getD()];

      model.addConstr(served >= y[i][bl], "att_path_" + to_string(i) + "_" + to_string(bl));
    }
  }

#ifndef Silence
  cout << "[***] Constraint: Include node in path" << endl;
#endif
}

void DeterministicModel::timeConstraint()
{
  auto graph = input->getGraph();
  int i, j, n = graph->getN();

  GRBLinExpr arcTravel, blockTravel;
  for (i = 0; i < n; i++)
  {
    for (auto *arc : graph->getArcs(i))
    {
      j = arc->getD();
      arcTravel += x[i][j] * arc->getLength();
    }

    for (auto b : graph->getNode(i).second)
      if (b != -1)
        blockTravel += y[i][b] * graph->getTimePerBlock(b);
  }
  model.addConstr(blockTravel + arcTravel <= input->getT(), "max_time");

#ifndef Silence
  cout << "[***] Constraint: time limit" << endl;
#endif
}

void DeterministicModel::compactTimeConstraint()
{
  auto graph = input->getGraph();
  int b, i, j, k, n = graph->getN();

  for (i = 0; i <= n; i++)
  {
    if (i < n)
      model.addConstr(t[n][i] == 0);

    for (auto *arc : graph->getArcs(i))
    {
      j = arc->getD();
      if (j >= n)
        continue;

      for (auto *arcl : graph->getArcs(j))
      {
        k = arcl->getD();
        model.addConstr(t[j][k] >= t[i][j] - (2 - x[i][j] - x[j][k]) * input->getT() + arc->getLength() * x[i][j], "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }

      for (auto b : graph->getNode(i).second)
        model.addConstr(t[i][j] <= input->getT() - y[i][b] * graph->getTimePerBlock(b), "t_leq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
    }
  }
  for (i = 0; i < n; i++)
  {
    model.addConstr(t[i][n] <= input->getT(), "max_time");
  }

#ifndef Silence
  cout << "[***] Constraint: Time limit" << endl;
#endif
}

void DeterministicModel::solveCompact(string time_limit)
{
  try
  {
    model.set("TimeLimit", time_limit);
    model.set("OutputFlag", "0");
    model.update();
#ifndef Silence
    model.set("OutputFlag", "1");
    model.update();
#endif
    model.write("model.lp");
    model.optimize();
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
  }
}

void DeterministicModel::solveExponential(string time_limit, bool frac_cut)
{
  try
  {
    auto graph = input->getGraph();
    model.set("TimeLimit", time_limit);
    model.set(GRB_DoubleParam_Heuristics, 1.0);
    model.set(GRB_IntParam_LazyConstraints, 1);
    cyclecallback cb = cyclecallback(input, graph->getN(), x, y, frac_cut);
    model.setCallback(&cb);
    model.set("OutputFlag", "0");
    model.update();

#ifndef Silence
    model.set("OutputFlag", "1");
    model.update();
#endif

    model.write("model.lp");
    model.optimize();

    // Save the number of cuts
    num_lazy_cuts = cb.num_lazy_cuts, num_frac_cuts = cb.num_frac_cuts;
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
  }
}

Solution DeterministicModel::getSolution()
{
  auto graph = input->getGraph();
  double of = 0.0;
  try
  {
    of = model.get(GRB_DoubleAttr_ObjVal);
  }
  catch (GRBException &ex)
  {
    of = 0.0;
  }

  double UB = model.get(GRB_DoubleAttr_ObjBound);
  double runtime = model.get(GRB_DoubleAttr_Runtime);
  int gurobi_nodes = model.get(GRB_DoubleAttr_NodeCount);
  int num_lazy_cuts = this->num_lazy_cuts;
  int num_frac_cuts = this->num_frac_cuts;
  int time_used = 0;

  vector<vector<int>> y;
  vector<vector<int_pair>> x;
  y.push_back(vector<int>()), x.push_back(vector<int_pair>());

  for (int i = 0; i <= graph->getN(); i++)
  {
    for (auto *arc : graph->getArcs(i))
      if (this->x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
      {
        x[0].push_back(make_pair(i, arc->getD()));
        time_used += arc->getLength();
      }

    for (int b : graph->getNode(i).second)
      if (this->y[i][b].get(GRB_DoubleAttr_X) > 0.5)
      {
        y[0].push_back(b);
        time_used += graph->getTimePerBlock(b);
      }
  }

  Solution solution = Solution(of, UB, runtime, time_used, num_lazy_cuts, num_frac_cuts, gurobi_nodes, y, x);
  return solution;
}

bool DeterministicModel::checkSolution()
{
  auto graph = input->getGraph();
  int max_time = input->getT();
  int n = graph->getN();

  // Check connectivity
  vector<vector<bool>> used_arc = vector<vector<bool>>(n + 1, vector<bool>(n + 1, false));

  int start_node = n, i, j, s, target;
  bool find_next = true;
  float time = 0, insecticide = 0;

  vector<bool> visited(n + 1, false);
  vector<int> conn_comp = vector<int>(n + 1, -1);
  vector<vector<int>> conn = vector<vector<int>>(n + 1, vector<int>());

  // DFS
  deque<int> stack;
  stack.push_back(n);

  while (!stack.empty())
  {
    s = stack.front();
    stack.pop_front();

    for (auto *arc : graph->getArcs(s))
    {
      j = arc->getD();
      if (x[s][j].get(GRB_DoubleAttr_X) > 0.5)
      {
        used_arc[s][j] = true;

        if (!visited[j])
        {
          stack.push_back(j);
          visited[j] = true;
        }
      }
    }
  }

  // Check visiting
  for (i = 0; i <= n; i++)
  {
    for (auto b : graph->getNode(i).second)
    {
      if (y[i][b].get(GRB_DoubleAttr_X) > 0.5)
      {
        time += graph->getTimePerBlock(b);

        if (!visited[i])
        {
          cout << "[!!!] Not visited node!" << endl;
          return false;
        }
      }
    }

    for (auto *arc : graph->getArcs(i))
    {
      if (x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.8)
      {
        time += arc->getLength();
        if (!used_arc[i][arc->getD()])
        {
          cout << "[!!!] Not used arc!" << endl;
          cout << i << " " << arc->getD() << endl;
          return false;
        }
      }
    }
  }

  if (time > max_time)
  {
    cout << "T: " << time << " <= " << max_time << endl;
    cout << "[!!!] Resource limitation error!" << endl;
    return false;
  }

#ifndef Silence
  cout << "[***] Instance ok!!!" << endl;
#endif
  return true;
}
