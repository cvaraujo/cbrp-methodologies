//
// Created by carlos on 06/07/21.
//

#include "../headers/DeterministicModel.h"

DeterministicModel::DeterministicModel(Graph *graph)
{
  if (graph != nullptr)
    this->graph = graph;
  else
    exit(EXIT_FAILURE);
}

DeterministicModel::~DeterministicModel()
{
  x.clear(), y.clear(), t.clear();
  model.terminate();
}

void DeterministicModel::createVariables()
{
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
      for (auto *arc : graph->arcs[o])
      {
        d = arc->getD();
        sprintf(name, "x_%d_%d", o, d);
        x[o][d] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
      }
    }

    // Y
    for (int i = 0; i < n; i++)
    {
      o = graph->nodes[i].first;
      for (int bl = 0; bl < graph->getB(); bl++)
      {
        sprintf(name, "y_%d_%d", o, bl);
        y[o][bl] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
      }
    }

    // T
    for (o = 0; o <= n; o++)
    {
      for (auto *arc : graph->arcs[o])
      {
        d = arc->getD();
        sprintf(name, "t_%d_%d", o, d);
        t[o][d] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
      }
    }

    model.update();
    cout << "Create variables" << endl;
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
  cout << "[!!!] Creating the model!" << endl;
  objectiveFunction();
  artificialNodes(), flowConservation();
  maxAttending(), timeConstraint();
  compactTimeConstraint();
  attendingPath();

  if (warm_start)
  {
    cout << "[!!!] Calling Warm-Start function!" << endl;
    this->WarmStart();
  }
  model.update();
  cout << "[***] done!" << endl;
}

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

void DeterministicModel::objectiveFunction()
{
  GRBLinExpr objective;
  int i, j, n = graph->getN();

  for (i = 0; i < n; i++)
  {
    j = graph->nodes[i].first;
    for (auto b : graph->nodes[i].second)
    {
      if (b == -1)
        break;

      objective += (y[j][b] * graph->cases_per_block[b]);
    }
  }

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();
  cout << "[***] Obj. Function: Maximize profit" << endl;
}

void DeterministicModel::artificialNodes()
{
  int n = graph->getN();
  GRBLinExpr sink, target;

  for (int i = 0; i < n; i++)
  {
    sink += x[n][i];
    target += x[i][n];
  }

  model.addConstr(sink == 1, "sink_constraint");
  model.addConstr(target == 1, "target_constraint");

  cout << "[***] Contraint: dummy depot" << endl;
}

void DeterministicModel::flowConservation()
{
  int i, j, n = graph->getN();

  for (i = 0; i < n; i++)
  {
    GRBLinExpr flow_out, flow_in;

    for (auto *arc : graph->arcs[i])
    {
      if (arc->getD() >= n)
        continue;
      flow_out += x[i][arc->getD()];
    }

    for (j = 0; j < n; j++)
    {
      for (auto *arc : graph->arcs[j])
      {
        if (arc->getD() == i)
          flow_in += x[j][i];
      }
    }

    flow_out += x[i][n];
    flow_in += x[n][i];
    model.addConstr(flow_in - flow_out == 0, "flow_conservation_" + to_string(i));
  }

  cout << "[***] Constraint: Flow conservation" << endl;
}

void DeterministicModel::maxAttending()
{
  int bl, b = graph->getB();

  for (bl = 0; bl < b; bl++)
  {
    GRBLinExpr maxServ;
    for (auto i : graph->nodes_per_block[bl])
      maxServ += y[i][bl];

    model.addConstr(maxServ <= 1, "max_service_block_" + to_string(bl));
  }
  cout << "[***] Constraint: Serve each block at most once" << endl;
}

void DeterministicModel::attendingPath()
{
  int j, bl, n = graph->getN(), b = graph->getB();
  for (bl = 0; bl < b; bl++)
  {
    for (auto i : graph->nodes_per_block[bl])
    {
      GRBLinExpr served;
      for (auto *arc : graph->arcs[i])
        served += x[i][arc->getD()];

      model.addConstr(served >= y[i][bl], "att_path_" + to_string(i) + "_" + to_string(bl));
    }
  }

  cout << "[***] Constraint: Include node in path" << endl;
}

void DeterministicModel::timeConstraint()
{
  int i, j, n = graph->getN();

  GRBLinExpr arcTravel, blockTravel;
  for (i = 0; i < n; i++)
  {
    for (auto *arc : graph->arcs[i])
    {
      j = arc->getD();
      arcTravel += x[i][j] * arc->getLength();
    }

    for (auto b : graph->nodes[i].second)
      if (b != -1)
        blockTravel += y[i][b] * graph->time_per_block[b];
  }
  model.addConstr(arcTravel + blockTravel <= graph->getT(), "max_time");

  cout << "[***] Constraint: time limit" << endl;
}

void DeterministicModel::compactTimeConstraint()
{
  int b, i, j, k, n = graph->getN();

  for (i = 0; i <= n; i++)
  {
    if (i < n)
      model.addConstr(t[n][i] == 0);

    for (auto *arc : graph->arcs[i])
    {
      j = arc->getD();
      if (j >= n)
        continue;

      GRBLinExpr time_blocks_j = 0;
      for (auto b : graph->nodes[j].second)
        if (b != -1)
          time_blocks_j += y[j][b] * graph->time_per_block[b];

      for (auto *arcl : graph->arcs[j])
      {
        k = arcl->getD();
        model.addConstr(t[j][k] >= t[i][j] + time_blocks_j + x[j][k] * arc->getLength() - ((2 - x[i][j] - x[j][k]) * graph->getT()), "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }
    }
  }
  for (i = 0; i < n; i++)
    model.addConstr(t[i][n] <= graph->getT() * x[i][n], "max_time");

  cout << "[***] Constraint: Time limit" << endl;
}

void DeterministicModel::solveCompact(string timeLimit)
{
  try
  {
    model.set("TimeLimit", timeLimit);

    model.update();
    // model.set("OutputFlag", "0");
    // model.computeIIS();
    model.write("model.lp");
    model.optimize();
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
  }
}

void DeterministicModel::writeSolution(string result)
{
  try
  {
    ofstream output;
    output.open(result);

    int j, n = graph->getN(), b = graph->getB();

    output << "Nodes: " << n << endl;
    output << "Arcs: " << graph->getM() << endl;
    output << "Blocks: " << b << endl;
    output << "LB: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    output << "UB: " << model.get(GRB_DoubleAttr_ObjBound) << endl;
    output << "Gurobi Nodes: " << model.get(GRB_DoubleAttr_NodeCount) << endl;
    output << "LAZY_CUTS: " << this->num_lazy_cuts << endl;
    output << "FRAC_CUTS: " << this->num_frac_cuts << endl;
    output << "Runtime: " << model.get(GRB_DoubleAttr_Runtime) << endl;

    float timeUsed = 0, insecUsed = 0;
    for (int i = 0; i <= n; i++)
    {
      for (auto *arc : graph->arcs[i])
      {
        j = arc->getD();
        if (x[i][j].get(GRB_DoubleAttr_X) > 0.5)
        {
          timeUsed += arc->getLength();
          output << "X: " << i << " " << j << endl;
        }
      }
    }

    for (int i = 0; i < n; i++)
    {
      for (auto b : graph->nodes[i].second)
      {
        if (y[i][b].get(GRB_DoubleAttr_X) > 0.5)
        {
          timeUsed += graph->time_per_block[b];
          output << "Y: " << i << " " << b << " = " << graph->cases_per_block[b] << endl;
        }
      }
    }

    output << "Route Time: " << timeUsed << endl;
    output << "Insecticide Used: " << insecUsed << endl;
    cout << "OF: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
  }
  catch (GRBException &ex)
  {
    ofstream output;
    output.open(result);

    int n = graph->getN(), b = graph->getB(), j;
    output << "Nodes: " << n << endl;
    output << "Arcs: " << graph->getM() << endl;
    output << "Blocks: " << b << endl;
    output << "LB: 0" << endl;
    output << "UB: " << model.get(GRB_DoubleAttr_ObjBound) << endl;
    output << "N. Nodes: " << model.get(GRB_DoubleAttr_NodeCount) << endl;
    output << "Runtime: " << model.get(GRB_DoubleAttr_Runtime) << endl;
    cout << "OF: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    cout << ex.getMessage() << endl;
  }
}

bool DeterministicModel::check_solution(float max_time, float max_insecticide)
{
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

    for (auto *arc : graph->arcs[s])
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
    for (auto b : graph->nodes[i].second)
    {
      if (y[i][b].get(GRB_DoubleAttr_X) > 0.5)
      {
        time += graph->time_per_block[b];
        insecticide += 0;

        if (!visited[i])
        {
          cout << "[!!!] Not visited node!" << endl;
          return false;
        }
      }
    }

    for (auto *arc : graph->arcs[i])
    {
      if (x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.5)
      {
        time += arc->getLength();
        if (!used_arc[i][arc->getD()] && insecticide > 0)
        {
          cout << "[!!!] Not used arc" << endl;
          return false;
        }
      }
    }
  }

  if (time > max_time || insecticide > max_insecticide)
  {
    cout << "T: " << time << " <= " << max_time << ", I: " << insecticide << " <= " << max_insecticide << endl;
    cout << "[!!!] Resource limitation error!" << endl;
    return false;
  }

  cout << "[***] Instance ok!!!" << endl;
  return true;
}
