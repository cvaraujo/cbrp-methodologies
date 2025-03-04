//
// Created by carlos on 03/03/25.
//

#include "StochasticModelWalk.hpp"

class cyclecallbackStochasticWalk : public GRBCallback
{

public:
  double lastiter, lastnode;
  int numvars, cuts = 0, num_frac_cuts = 0, num_lazy_cuts = 0;
  bool frac_cut = false;
  vector<vector<vector<GRBVar>>> x;
  vector<vector<GRBVar>> y;
  typedef ListDigraph G;
  typedef G::Arc Arc;
  typedef G::ArcIt ArcIt;
  typedef G::Node Node;
  typedef G::ArcMap<double> LengthMap;
  typedef G::NodeMap<bool> BoolNodeMap;
  Input *input;

  cyclecallbackStochasticWalk(Input *xinput, int xnumvars, vector<vector<vector<GRBVar>>> xx, vector<vector<GRBVar>> yy, bool frac_cut)
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

        for (int r = 0; r <= input->getS(); r++)
        {
          int i, j, s, N = graph->getN();
          vector<vector<int>> g = vector<vector<int>>(N + 2, vector<int>());
          vector<bool> used_node = vector<bool>(N + 1);

          // Create graph G'
          for (i = 0; i <= N; i++)
          {
            for (auto *arc : graph->getArcs(i))
            {
              if (getSolution(x[i][arc->getD()][r]) > 0)
              {
                g[i].push_back(arc->getD());
                used_node[i] = used_node[arc->getD()] = true;
              }
            }
          }

          vector<bool> visited(N + 1, false);
          vector<int> node_connected_component = vector<int>(N + 1, -1);
          vector<vector<int>> connected_component;
          vector<vector<int_pair>> arcs_from_component;
          int idx = 0;

          // DFS
          for (i = N; i >= 0; i--)
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
            continue;

          // Need Cuts
          is_feasible = false;

          for (i = 1; i < connected_component.size(); i++)
          {
            vector<int> s_nodes = connected_component[i];
            vector<int_pair> s_arcs = arcs_from_component[i];
            GRBLinExpr cut_arcs;

            // Arcs in the cut S
            for (int j = 0; j < N; j++)
              if (node_connected_component[j] != i)
                for (auto arc : graph->getArcs(j))
                  if (node_connected_component[arc->getD()] == i)
                    cut_arcs += x[j][arc->getD()][r];

            for (int org_1 : s_nodes)
            {
              for (int org_2 : s_nodes)
              {
                if (org_1 == org_2)
                  continue;

                for (auto a_p : graph->getArcs(org_1))
                {
                  for (auto a_pp : graph->getArcs(org_2))
                  {
                    addLazy(cut_arcs >= x[org_1][a_p->getD()][r] + x[org_2][a_pp->getD()][r] - 1);
                    num_lazy_cuts++;
                  }
                }
              }
            }
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
          for (int r = 0; r <= input->getS(); r++)
          {
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

                if (getNodeRel(x[i][j][r]) > 0)
                {
                  set_arcs.push_back(flow_graph.addArc(set_nodes[i], set_nodes[j]));
                  capacity[set_arcs[set_arcs.size() - 1]] = double(getNodeRel(x[i][j][r]));
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
              Preflow<G, LengthMap> preflow(flow_graph, capacity, set_nodes[i], set_nodes[n]);
              preflow.runMinCut();
              mincut_value = preflow.flowValue();

              if (mincut_value >= 1.0)
                continue;

              // Create basic variables
              GRBLinExpr cut_arcs;
              double cut_value = 0;

              // Get cut arcs
              for (j = 0; j < n; j++)
              {
                if (!preflow.minCut(set_nodes[j]))
                  continue;

                for (auto arc : graph->getArcs(j))
                {
                  int k = arc->getD();

                  if (preflow.minCut(set_nodes[k]))
                    continue;

                  cut_arcs += x[j][k][r];
                }
              }

              if (cut_arcs.size() > 0)
              {
                addCut(cut_arcs >= cut_value);
                num_frac_cuts++;
              }
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

Solution StochasticModelWalk::Run(bool use_warm_start, string time_limit, string model, bool use_cuts)
{
  this->createVariables();
  this->initModel(model);
  this->solveExponential(time_limit, use_cuts);

  this->checkSolution();

  return this->getSolution();
}

void StochasticModelWalk::createVariables()
{
  Graph *graph = this->input->getGraph();
  int o, d, k, n = graph->getN(), m = graph->getM(), b = graph->getB(), S = input->getS();

  try
  {
    env.set("LogFile", "MS_mip.log");
    env.start();

    x = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(n + 1, vector<GRBVar>(S + 1)));
    y = vector<vector<GRBVar>>(b, vector<GRBVar>(S + 1));
    z = vector<vector<GRBVar>>(b, vector<GRBVar>(S + 1));

    for (int r = 0; r <= S; r++)
    {
      // X
      char name[40];
      for (o = 0; o <= n; o++)
      {
        for (auto *arc : graph->getArcs(o))
        {
          d = arc->getD();
          sprintf(name, "x_%d_%d_%d", o, d, r);
          x[o][d][r] = model.addVar(0.0, GRB_INFINITY, 0, GRB_INTEGER, name);
        }
      }
      // Y
      for (int bl = 0; bl < b; bl++)
      {
        sprintf(name, "y_%d_%d", bl, r);
        y[bl][r] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
      }
      // Z
      for (int bl = 0; bl < b; bl++)
      {
        sprintf(name, "z_%d_%d", bl, r);
        z[bl][r] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
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

void StochasticModelWalk::initModel(string model)
{
#ifndef Silence
  cout << "[***] Creating " << model << " model!" << endl;
#endif

  objectiveFunction(), artificialNodes(), zValue();
  flowConservation(), attendingPath(), timeConstraint();

  this->model.update();

#ifndef Silence
  cout << "[***] Constraints and objective function created!" << endl;
#endif
}

void StochasticModelWalk::objectiveFunction()
{
  auto graph = this->input->getGraph();
  GRBLinExpr objective;
  int b, i, j, N = graph->getN(), S = input->getS(), B = graph->getB();

  for (auto b = 0; b < B; b++)
  {
    double expr = 0;
    for (int s = 0; s < S; s++)
      expr += input->getScenario(s)->getProbability() * input->getAlpha() * input->getScenario(s)->getCasesPerBlock(b);

    objective += (y[b][0] * (graph->getCasesPerBlock(b) + expr));
  }

  for (int s = 0; s < S; s++)
  {
    GRBLinExpr expr;
    for (int b = 0; b < graph->getB(); b++)
      expr += z[b][s + 1];

    objective += input->getScenario(s)->getProbability() * expr;
  }

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();

#ifndef Silence
  cout << "[***] Obj. Function: Maximize profit" << endl;
#endif
}

void StochasticModelWalk::zValue()
{
  auto graph = this->input->getGraph();
  for (int s = 1; s <= input->getS(); s++)
  {
    vector<double> cases = input->getScenario(s - 1)->getCases();

    for (int b = 0; b < graph->getB(); b++)
    {
      model.addConstr(z[b][s] <= y[b][s] * ((1 - input->getAlpha()) * cases[b]) + (1 - y[b][0]) * input->getAlpha() * cases[b], "max_z_profit");
      model.addConstr(z[b][s] <= y[b][s] * cases[b], "z_bigm_profit");
    }
  }
  model.update();
#ifndef Silence
  cout << "[***] Constraint: z value" << endl;
#endif
}

void StochasticModelWalk::artificialNodes()
{
  int n = input->getGraph()->getN();
  for (int s = 0; s <= input->getS(); s++)
  {
    GRBLinExpr sink, target;

    for (int i = 0; i < n; i++)
    {
      sink += x[n][i][s];
      target += x[i][n][s];
    }

    model.addConstr(sink == 1, "sink_constraint_" + to_string(s));
    model.addConstr(target == 1, "target_constraint_" + to_string(s));
  }
#ifndef Silence
  cout << "[***] Contraint: dummy depot" << endl;
#endif
}

void StochasticModelWalk::flowConservation()
{
  auto graph = input->getGraph();
  int i, j, n = graph->getN();

  for (int s = 0; s <= input->getS(); s++)
  {
    for (i = 0; i < n; i++)
    {
      GRBLinExpr flow_out, flow_in;

      for (auto *arc : graph->getArcs(i))
      {
        if (arc->getD() >= n)
          continue;
        flow_out += x[i][arc->getD()][s];
      }

      for (j = 0; j < n; j++)
      {
        for (auto *arc : graph->getArcs(j))
        {
          if (arc->getD() == i)
            flow_in += x[j][i][s];
        }
      }

      flow_out += x[i][n][s];
      flow_in += x[n][i][s];
      model.addConstr(flow_in - flow_out == 0, "flow_conservation_" + to_string(i));
    }
  }
#ifndef Silence
  cout << "[***] Constraint: Flow conservation" << endl;
#endif
}

void StochasticModelWalk::attendingPath()
{
  auto graph = input->getGraph();
  int j, bl, n = graph->getN(), b = graph->getB();

  for (int s = 0; s <= input->getS(); s++)
  {
    for (bl = 0; bl < b; bl++)
    {
      GRBLinExpr served;
      for (auto i : graph->getNodesFromBlock(bl))
        for (auto *arc : graph->getArcs(i))
          served += x[i][arc->getD()][s];

      model.addConstr(served >= y[bl][s], "att_path_" + to_string(bl));
    }
  }

#ifndef Silence
  cout << "[***] Constraint: Include node in path" << endl;
#endif
}

void StochasticModelWalk::timeConstraint()
{
  auto graph = input->getGraph();
  int i, j, n = graph->getN();

  for (int s = 0; s <= input->getS(); s++)
  {
    GRBLinExpr arcTravel, blockTravel;
    for (i = 0; i < n; i++)
    {
      for (auto *arc : graph->getArcs(i))
      {
        j = arc->getD();
        arcTravel += x[i][j][s] * arc->getLength();
      }
    }

    for (auto b = 0; b < graph->getB(); b++)
      if (b != -1)
        blockTravel += y[b][s] * graph->getTimePerBlock(b);

    model.addConstr(arcTravel + blockTravel <= input->getT(), "max_time");
  }

#ifndef Silence
  cout << "[***] Constraint: time limit" << endl;
#endif
}

void StochasticModelWalk::solveExponential(string time_limit, bool frac_cut)
{
  try
  {
    auto graph = input->getGraph();
    model.set("TimeLimit", time_limit);
    model.set(GRB_DoubleParam_Heuristics, 1.0);
    model.set(GRB_IntParam_LazyConstraints, 1);
    cyclecallbackStochasticWalk cb = cyclecallbackStochasticWalk(input, graph->getN(), x, y, frac_cut);
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

Solution StochasticModelWalk::getSolution()
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

  for (int s = 0; s <= input->getS(); s++)
  {
    y.push_back(vector<int>()), x.push_back(vector<int_pair>());

    for (int i = 0; i <= graph->getN(); i++)
    {
      for (auto *arc : graph->getArcs(i))
        if (this->x[i][arc->getD()][s].get(GRB_DoubleAttr_X) > 0.5)
        {
          x[s].push_back(make_pair(i, arc->getD()));
          time_used += arc->getLength();
        }
    }

    for (int b = 0; b < graph->getB(); b++)
      if (this->y[b][s].get(GRB_DoubleAttr_X) > 0.5)
      {
        y[s].push_back(b);
        time_used += graph->getTimePerBlock(b);
      }
  }

  Solution solution = Solution(this->input, of, UB, runtime, time_used, num_lazy_cuts, num_frac_cuts, gurobi_nodes, y, x);
  return solution;
}

bool StochasticModelWalk::checkSolution()
{
  auto graph = input->getGraph();
  int max_time = input->getT();
  int n = graph->getN();
  int S = input->getS();

  for (int r = 0; r <= S; r++)
  {
#ifndef Silence
    cout << "Checking Scenario " << r << endl;
#endif
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
        if (x[s][j][r].get(GRB_DoubleAttr_X) > 0.5)
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
      for (auto *arc : graph->getArcs(i))
      {
        if (x[i][arc->getD()][r].get(GRB_DoubleAttr_X) > 0.8)
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

    for (int b = 0; b < graph->getB(); b++)
    {
      if (y[b][r].get(GRB_DoubleAttr_X) > 0.5)
      {
        time += graph->getTimePerBlock(b);

        bool is_node_in_path = false;
        for (auto j : graph->getNodesFromBlock(b))
        {
          if (visited[j])
          {
            is_node_in_path = true;
            break;
          }
        }
        if (!is_node_in_path)
        {
          cout << "[!!!] Not visited node!" << endl;
          return false;
        }
      }
    }

    if (time > max_time)
    {
      cout << "T: " << time << " <= " << max_time << endl;
      cout << "[!!!] Resource limitation error!" << endl;
      return false;
    }
  }

#ifndef Silence
  cout << "[***] Instance ok!!!" << endl;
#endif
  return true;
}
