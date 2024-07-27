//
// Created by carlos on 06/07/21.
//

#include "DeterministicModel.hpp"

Solution DeterministicModel::Run(bool use_warm_start, string time_limit, string output_file)
{
  this->createVariables();
  this->initModelCompact(use_warm_start);
  this->solveCompact(time_limit);

#ifndef Silence
  this->checkSolution();
#endif

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

void DeterministicModel::initModelCompact(bool warm_start)
{
#ifndef Silence
  cout << "[!!!] Creating the model!" << endl;
#endif

  objectiveFunction();
  artificialNodes(), flowConservation();
  maxAttending(), attendingPath();
  timeConstraint();
  compactTimeConstraint();

  if (warm_start)
  {
#ifndef Silence
    cout << "[!!!] Calling Warm-Start function!" << endl;
#endif

    // this->WarmStart();
  }
  model.update();
#ifndef Silence
  cout << "[***] done!" << endl;
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
      objective += (y[j][b] * graph->getCasesPerBlock(b));
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

      GRBLinExpr time_ij = 0;
      time_ij += t[i][j] + arc->getLength() * x[i][j];

      for (auto b : graph->getNode(j).second)
        time_ij += y[j][b] * graph->getTimePerBlock(b);

      for (auto *arcl : graph->getArcs(j))
      {
        k = arcl->getD();
        model.addConstr(time_ij <= t[j][k] + ((2 - x[i][j] - x[j][k]) * input->getT()), "t_leq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }
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
#endif
    // model.computeIIS();
    model.write("model.lp");
    model.optimize();
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
  }
}

Solution DeterministicModel::getSolution()
{
  auto graph = input->getGraph();
  double of = model.get(GRB_DoubleAttr_ObjVal);
  double UB = model.get(GRB_DoubleAttr_ObjBound);
  double runtime = model.get(GRB_DoubleAttr_Runtime);
  int gurobi_nodes = model.get(GRB_DoubleAttr_NodeCount);
  int num_lazy_cuts = this->num_lazy_cuts;
  int num_frac_cuts = this->num_frac_cuts;
  int time_used = 0;

  vector<int> y;
  vector<int_pair> x;
  for (int i = 0; i <= graph->getN(); i++)
  {
    for (auto *arc : graph->getArcs(i))
      if (this->x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
      {
        x.push_back(make_pair(i, arc->getD()));
        time_used += arc->getLength();
      }

    for (int b : graph->getNode(i).second)
      if (this->y[i][b].get(GRB_DoubleAttr_X) > 0.5)
      {
        y.push_back(i);
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

  cout << "[***] Instance ok!!!" << endl;
  return true;
}
