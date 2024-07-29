//
// Created by carlos on 06/07/21.
//

#include "StochasticModel.hpp"

class stochasticCyclecallback : public GRBCallback
{

public:
  double lastiter, lastnode;
  int numvars, cuts = 0, num_frac_cuts = 0, num_lazy_cuts = 0;
  bool frac_cut = false;
  vector<vector<vector<GRBVar>>> x, y;
  typedef ListDigraph G;
  typedef G::Arc Arc;
  typedef G::ArcIt ArcIt;
  typedef G::Node Node;
  typedef G::ArcMap<double> LengthMap;
  typedef G::NodeMap<bool> BoolNodeMap;
  Input *input;

  stochasticCyclecallback(Input *xinput, int xnumvars, vector<vector<vector<GRBVar>>> xx, vector<vector<vector<GRBVar>>> yy, bool frac_cut)
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
          int i, j, s, n = graph->getN();
          vector<vector<int>> g = vector<vector<int>>(n + 2, vector<int>());
          vector<bool> used_node = vector<bool>(n + 1);

          for (i = 0; i <= n; i++)
          {
            for (auto *arc : graph->getArcs(i))
            {
              if (getSolution(x[i][arc->getD()][r]) >= 0.1)
              {
                g[i].push_back(arc->getD());
                used_node[i] = used_node[arc->getD()] = true;
              }
            }
          }

          vector<bool> visited(n + 1, false);
          vector<int> conn_comp = vector<int>(n + 1, -1);
          vector<vector<int>> conn = vector<vector<int>>(n + 1, vector<int>());

          for (i = n; i >= 0; i--)
          {
            if (!used_node[i])
              continue;

            vector<int> stack;
            stack.push_back(i);

            while (!stack.empty())
            {
              s = stack.back();
              stack.pop_back();

              if (!visited[s])
              {
                conn[i].push_back(s);
                conn_comp[s] = i;
                visited[s] = true;
              }

              for (auto k : g[s])
                if (!visited[k])
                  stack.push_back(k);
            }
          }

          int num_comp = 0;
          for (i = n; i >= 0; i--)
          {
            if (conn[i].size() > 1)
              num_comp++;
            if (num_comp > 1)
              break;
          }

          if (num_comp == 1)
            continue;
          else
            is_feasible = false;

          for (i = 0; i <= n; i++)
          {
            if (!used_node[i])
              continue;

            GRBLinExpr expr = 0;
            bool has_constr = false;

            for (auto v : conn[i])
            {
              for (auto *arc : graph->getArcs(v))
              {
                j = arc->getD();
                if (conn_comp[j] != i)
                {
                  expr += x[v][j][r];
                  has_constr = true;
                }
              }
            }

            if (has_constr)
            {
              for (auto v : conn[i])
              {
                for (auto b : graph->getNode(v).second)
                {
                  if (getSolution(y[v][b][r]) >= 0.1)
                  {
                    addLazy(expr >= y[v][b][r]);
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
        cout << "aaaaaaaaa" << endl;
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
            vector<Node> setNodes = vector<Node>(n + 1);
            vector<bool> usedNode = vector<bool>(n, false);
            vector<Arc> setArcs;

            // Create the node set
            for (i = 0; i <= n; i++)
              setNodes[i] = flow_graph.addNode();

            // Create the edge set
            for (i = 0; i < n; i++)
            {
              for (auto *arc : graph->getArcs(i))
              {
                j = arc->getD();

                if (getNodeRel(x[i][j][r]) >= 0.1)
                {
                  setArcs.push_back(flow_graph.addArc(setNodes[i], setNodes[j]));
                  capacity[setArcs[setArcs.size() - 1]] = double(getNodeRel(x[i][j][r]));
                  usedNode[i] = usedNode[j] = true;
                }
              }
            }
            // Init necessary structures
            double mincut_value;
            bool source_side, has_constr, need_cut;

            // cout << "Add Cut" << endl;
            for (i = 0; i < n; i++)
            {
              // If there is no arc using this node, ignore it
              if (!usedNode[i])
                continue;

              // Lemon MaxFlow instance
              Preflow<G, LengthMap> preflow(flow_graph, capacity, setNodes[i], setNodes[n]);
              preflow.runMinCut();
              mincut_value = preflow.flowValue();

              // Create basic variables
              // BoolNodeMap bm(flow_graph);
              source_side = preflow.minCut(setNodes[i]);

              // Check the need for cutting
              need_cut = false;
              for (auto b : graph->getNode(i).second)
              {
                if (mincut_value < getNodeRel(y[i][b][r]))
                {
                  need_cut = true;
                  break;
                }
              }

              if (!need_cut)
                continue;

              GRBLinExpr expr = 0;
              has_constr = false;

              for (u = 0; u < n; u++)
              {
                if (!usedNode[u] || preflow.minCut(setNodes[u]) != source_side)
                  continue;

                for (auto *arc : graph->getArcs(u))
                {
                  v = arc->getD();
                  if (preflow.minCut(setNodes[v]) != source_side)
                  {
                    expr += x[u][v][r];
                    has_constr = true;
                  }
                }
              }

              if (has_constr)
              {
                for (u = 0; u < n; u++)
                {
                  if (usedNode[u] && preflow.minCut(setNodes[u]))
                  {
                    usedNode[u] = false;
                    for (auto b : graph->getNode(u).second)
                    {
                      if (getNodeRel(y[u][b][r]) > mincut_value)
                      {
                        addCut(expr >= y[u][b][r]);
                        num_frac_cuts++;
                      }
                    }
                  }
                }
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

Solution StochasticModel::Run(bool use_warm_start, string time_limit, string model, bool use_cuts)
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

#ifndef Silence
  cout << "[***] Model solved!" << endl;
#endif
  return this->getSolution();
}

void StochasticModel::createVariables()
{
  auto graph = this->input->getGraph();
  int o, d, k, n = graph->getN(), m = graph->getM(), b = graph->getB(), s = input->getS();
  try
  {
    env.set("LogFile", "MS_mip.log");
    env.start();

    x = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(n + 1, vector<GRBVar>(s + 1)));
    t = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(n + 1, vector<GRBVar>(s + 1)));
    y = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(b, vector<GRBVar>(s + 1)));
    z = vector<vector<GRBVar>>(b, vector<GRBVar>(s + 1));

    for (int r = 0; r <= s; r++)
    {
      // X
      char name[40];
      for (o = 0; o <= n; o++)
      {
        for (auto *arc : graph->getArcs(o))
        {
          d = arc->getD();
          sprintf(name, "x_%d_%d_%d", o, d, r);
          x[o][d][r] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
        }
      }

      // T
      for (o = 0; o <= n; o++)
      {
        for (auto *arc : graph->getArcs(o))
        {
          d = arc->getD();
          sprintf(name, "t_%d_%d_%d", o, d, r);
          t[o][d][r] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
        }
      }

      // Y
      for (o = 0; o < n; o++)
      {
        for (int bl : graph->getNode(o).second)
        {
          sprintf(name, "y_%d_%d_%d", o, bl, r);
          y[o][bl][r] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
        }
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
    cout << "Create variables" << endl;
#endif
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
    cout << ex.getErrorCode() << endl;
    exit(EXIT_FAILURE);
  }
}

void StochasticModel::initModel(string model)
{
#ifndef Silence
  cout << "[***] Creating " << model << " model!" << endl;
#endif

  objectiveFunction();
  zValue(), artificialNodes(), flowConservation();
  maxAttending(), timeConstraint(), attendingPath();

  if (model == "MTZ")
    compactTimeConstraint();

  this->model.update();

#ifndef Silence
  cout << "[***] Constraints and objective function created!" << endl;
#endif
}

void StochasticModel::objectiveFunction()
{
  auto graph = this->input->getGraph();
  GRBLinExpr objective;
  int b, i, j, N = graph->getN(), S = input->getS(), B = graph->getB();

  for (i = 0; i < N; i++)
  {
    for (auto b : graph->getNode(i).second)
    {
      double expr = 0;
      for (int s = 0; s < S; s++)
        expr += input->getScenario(s).getProbability() * input->getAlpha() * input->getScenario(s).getCasesPerBlock(b);

      objective += (y[i][b][0] * (graph->getCasesPerBlock(b) + expr));
    }
  }

  for (int s = 0; s < S; s++)
  {
    GRBLinExpr expr;
    for (int b = 0; b < graph->getB(); b++)
      expr += z[b][s + 1];

    objective += input->getScenario(s).getProbability() * expr;
  }

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();

#ifndef Silence
  cout << "[***] Obj. Function: Maximize profit" << endl;
#endif
}

void StochasticModel::zValue()
{
  auto graph = this->input->getGraph();
  for (int s = 1; s <= input->getS(); s++)
  {
    vector<double> cases = input->getScenario(s - 1).getCases();

    for (int b = 0; b < graph->getB(); b++)
    {
      GRBLinExpr rhs, rhs2;
      for (auto i : graph->getNodesFromBlock(b))
      {
        rhs += y[i][b][s];
        rhs2 += y[i][b][0];
      }

      model.addConstr(z[b][s] <= rhs * ((1 - input->getAlpha()) * cases[b]) + (1 - rhs2) * input->getAlpha() * cases[b], "max_z_profit");
      model.addConstr(z[b][s] <= rhs * cases[b], "z_bigm_profit");
    }
  }
  model.update();
#ifndef Silence
  cout << "[***] Constraint: z value" << endl;
#endif
}

void StochasticModel::artificialNodes()
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

void StochasticModel::flowConservation()
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

void StochasticModel::maxAttending()
{
  auto graph = input->getGraph();
  int bl, b = graph->getB();

  for (int s = 0; s <= input->getS(); s++)
  {
    for (bl = 0; bl < b; bl++)
    {
      GRBLinExpr maxServ;
      for (auto i : graph->getNodesFromBlock(bl))
        maxServ += y[i][bl][s];

      model.addConstr(maxServ <= 1, "max_service_block_" + to_string(bl));
    }
  }
#ifndef Silence
  cout << "[***] Constraint: Serve each block at most once" << endl;
#endif
}

void StochasticModel::attendingPath()
{
  auto graph = input->getGraph();
  int j, bl, n = graph->getN(), b = graph->getB();

  for (int s = 0; s <= input->getS(); s++)
  {
    for (bl = 0; bl < b; bl++)
    {
      for (auto i : graph->getNodesFromBlock(bl))
      {
        GRBLinExpr served;
        for (auto *arc : graph->getArcs(i))
          served += x[i][arc->getD()][s];

        model.addConstr(served >= y[i][bl][s], "att_path_" + to_string(i) + "_" + to_string(bl));
      }
    }
  }

#ifndef Silence
  cout << "[***] Constraint: Include node in path" << endl;
#endif
}

void StochasticModel::timeConstraint()
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
      for (auto b : graph->getNode(i).second)
        blockTravel += y[i][b][s] * graph->getTimePerBlock(b);
    }
    model.addConstr(arcTravel + blockTravel <= input->getT(), "max_time");
  }

#ifndef Silence
  cout << "[***] Constraint: time limit" << endl;
#endif
}

void StochasticModel::compactTimeConstraint()
{
  auto graph = input->getGraph();
  int b, i, j, k, n = graph->getN();

  for (int s = 0; s <= input->getS(); s++)
  {
    for (i = 0; i <= n; i++)
    {
      if (i < n)
        model.addConstr(t[n][i][s] == 0);

      for (auto *arc : graph->getArcs(i))
      {
        j = arc->getD();
        if (j >= n)
          continue;

        GRBLinExpr time_ij;
        time_ij += t[i][j][s] + arc->getLength() * x[i][j][s];

        for (auto b : graph->getNode(j).second)
          time_ij += y[j][b][s] * graph->getTimePerBlock(b);

        for (auto *arcl : graph->getArcs(j))
        {
          k = arcl->getD();
          model.addConstr(time_ij <= t[j][k][s] + (2 - x[i][j][s] - x[j][k][s]) * input->getT(), "t_leq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
        }
      }
    }
    for (i = 0; i < n; i++)
      model.addConstr(t[i][n][s] <= input->getT(), "max_time");
  }
#ifndef Silence
  cout << "[***] Constraint: Time limit" << endl;
#endif
}

void StochasticModel::solveCompact(string time_limit)
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

void StochasticModel::solveExponential(string time_limit, bool frac_cut)
{
  try
  {
    auto graph = input->getGraph();
    model.set("TimeLimit", time_limit);
    model.set(GRB_DoubleParam_Heuristics, 1.0);
    model.set(GRB_IntParam_LazyConstraints, 1);
    stochasticCyclecallback cb = stochasticCyclecallback(input, graph->getN(), x, y, frac_cut);
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

Solution StochasticModel::getSolution()
{
  auto graph = input->getGraph();
  double of = model.get(GRB_DoubleAttr_ObjVal);
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

      for (int b : graph->getNode(i).second)
        if (this->y[i][b][s].get(GRB_DoubleAttr_X) > 0.5)
        {
          y[s].push_back(b);
          time_used += graph->getTimePerBlock(b);
        }
    }
  }

  Solution solution = Solution(of, UB, runtime, time_used, num_lazy_cuts, num_frac_cuts, gurobi_nodes, y, x);
  solution.setS(input->getS());
  return solution;
}

bool StochasticModel::checkSolution()
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
      for (int b : graph->getNode(i).second)
      {
        if (y[i][b][r].get(GRB_DoubleAttr_X) > 0.5)
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