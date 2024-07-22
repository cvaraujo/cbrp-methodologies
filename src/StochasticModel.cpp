//
// Created by carlos on 06/07/21.
//

#include "../headers/StochasticModel.h"

StochasticModel::StochasticModel(Graph *graph)
{
  if (graph != nullptr)
    this->graph = graph;
  else
    exit(EXIT_FAILURE);
}

void StochasticModel::createVariables()
{
  int o, d, k, n = graph->getN(), m = graph->getM(), b = graph->getB(), s = graph->getS();
  try
  {
    env.set("LogFile", "MS_mip.log");
    env.start();

    x = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(n + 1, vector<GRBVar>(s + 1)));
    y = vector<vector<vector<GRBVar>>>(n, vector<vector<GRBVar>>(b, vector<GRBVar>(s + 1)));
    t = vector<vector<vector<GRBVar>>>(n + 1, vector<vector<GRBVar>>(n + 1, vector<GRBVar>(s + 1)));
    z = vector<vector<GRBVar>>(b, vector<GRBVar>(s + 1));

    for (int r = 0; r <= s; r++)
    {
      // X
      char name[40];
      for (o = 0; o <= n; o++)
      {
        for (auto *arc : graph->arcs[o])
        {
          d = arc->getD();
          sprintf(name, "x_%d_%d_%d", o, d, r);
          x[o][d][r] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
        }
      }

      // Y
      for (int i = 0; i < n; i++)
      {
        o = graph->nodes[i].first;
        for (int bl = 0; bl < graph->getB(); bl++)
        // for (auto bl : graph->nodes[i].second)
        {
          sprintf(name, "y_%d_%d_%d", o, bl, r);
          y[o][bl][r] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
        }
      }

      // T
      for (o = 0; o <= n; o++)
      {
        for (auto *arc : graph->arcs[o])
        {
          d = arc->getD();
          sprintf(name, "t_%d_%d_%d", o, d, r);
          t[o][d][r] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, name);
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
    cout << "Create variables" << endl;
  }
  catch (GRBException &ex)
  {
    cout << ex.getMessage() << endl;
    cout << ex.getErrorCode() << endl;
    exit(EXIT_FAILURE);
  }
}

void StochasticModel::initModelCompact(bool warm_start)
{
  cout << "[!!!] Creating the model!" << endl;
  objectiveFunction();
  zValue(), artificialNodes(), flowConservation();
  maxAttending(), timeConstraint();
  compactTimeConstraint();
  attendingPath();

  if (warm_start)
  {
    cout << "[!!!] Calling Warm-Start function!" << endl;
    StochasticWarmStart();
  }
  model.update();
  cout << "[***] done!" << endl;
}

void StochasticModel::StochasticWarmStart()
{
  int i, j;
  Graph *g1 = new Graph(*graph);
  Graph *g2 = new Graph(*graph);

  for (int b = 0; b < graph->getB(); b++)
    for (int r = 0; r < graph->getS(); r++)
      g1->cases_per_block[b] += this->alpha * graph->scenarios[r].cases_per_block[b];

  // Warm start first stage
  vector<pair<int, int>> x, y;
  double of = WarmStart::compute_solution(g1, graph->getT(), x, y);
  setStartSolution(0, x, y);

  for (int r = 0; r < graph->getS(); r++)
  {
    g2->cases_per_block = g1->scenarios[r].cases_per_block;
    for (auto pair : y)
      g2->cases_per_block[pair.second] = (1 - this->alpha) * g1->scenarios[r].cases_per_block[pair.second];

    // Warm start second stage
    vector<pair<int, int>> x2, y2;
    of = WarmStart::compute_solution(g2, graph->getT(), x2, y2);
    setStartSolution(r + 1, x2, y2);
  }

  cout << "Warm Start done!" << endl;
  // getchar();
}

void StochasticModel::setStartSolution(int s, vector<pair<int, int>> x, vector<pair<int, int>> y)
{
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
        this->x[i][arc->getD()][s].set(GRB_DoubleAttr_Start, 1.0);
        // model.addConstr(this->x[i][arc->getD()][s] == 1);
        // cout << "X[" << s << "] " << i << " -> " << arc->getD() << endl;
      }
      else
      {
        this->x[i][arc->getD()][s].set(GRB_DoubleAttr_Start, 0.0);
        // model.addConstr(this->x[i][arc->getD()][s] == 0);
      }
    }
  }
  model.update();

  int i, j;
  for (auto pair : y)
  {
    i = pair.first, j = pair.second;
    this->y[i][j][s].set(GRB_DoubleAttr_Start, 1.0);
    // model.addConstr(this->y[i][j][s] == 1);
    // cout << "Y[" << s << "] " << i << " -> " << j << endl;
  }
  model.update();
}

void StochasticModel::objectiveFunction()
{
  GRBLinExpr objective;
  int i, j, n = graph->getN(), s = graph->getS();
  vector<float> cases = graph->cases_per_block;
  vector<Scenario> scenarios = graph->scenarios;

  for (i = 0; i < n; i++)
  {
    j = graph->nodes[i].first;
    for (auto b : graph->nodes[i].second)
    {
      if (b == -1)
        break;

      double expr = 0.0;
      for (int r = 0; r < s; r++)
        expr += scenarios[r].probability * scenarios[r].cases_per_block[b];

      objective += (y[j][b][0] * (cases[b] + this->alpha * expr));
    }
  }

  for (int r = 0; r < s; r++)
  {
    GRBLinExpr expr;
    for (int b = 0; b < graph->getB(); b++)
      expr += z[b][r + 1];

    objective += scenarios[r].probability * expr;
  }

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();
  cout << "[***] Obj. Function: Maximize profit" << endl;
}

void StochasticModel::zValue()
{
  for (int s = 1; s <= graph->getS(); s++)
  {
    vector<float> cases = graph->scenarios[s - 1].cases_per_block;

    for (int b = 0; b < graph->getB(); b++)
    {
      GRBLinExpr rhs, rhs2;
      for (auto i : graph->nodes_per_block[b])
      {
        rhs += y[i][b][s];
        rhs2 += y[i][b][0];
      }
      model.addConstr(z[b][s] <= rhs * ((1.0 - this->alpha) * cases[b]) + (1 - rhs2) * this->alpha * cases[b], "max_z_profit");
      model.addConstr(z[b][s] <= rhs * cases[b], "z_bigm_profit");
    }
  }
  model.update();
  cout << "[***] Constraint: z value" << endl;
}

void StochasticModel::artificialNodes()
{
  int n = graph->getN();
  for (int s = 0; s <= graph->getS(); s++)
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
  cout << "[***] Contraint: dummy depot" << endl;
}

void StochasticModel::flowConservation()
{
  int i, j, n = graph->getN();

  for (int s = 0; s <= graph->getS(); s++)
  {
    for (i = 0; i < n; i++)
    {
      GRBLinExpr flow_out, flow_in;

      for (auto *arc : graph->arcs[i])
      {
        if (arc->getD() >= n)
          continue;
        flow_out += x[i][arc->getD()][s];
      }

      for (j = 0; j < n; j++)
      {
        for (auto *arc : graph->arcs[j])
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

  cout << "[***] Constraint: Flow conservation" << endl;
}

void StochasticModel::maxAttending()
{
  int bl, b = graph->getB();

  for (int s = 0; s <= graph->getS(); s++)
  {
    for (bl = 0; bl < b; bl++)
    {
      GRBLinExpr maxServ;
      for (auto i : graph->nodes_per_block[bl])
      {
        maxServ += y[i][bl][s];
      }

      model.addConstr(maxServ <= 1, "max_service_block_" + to_string(bl));
    }
  }
  cout << "[***] Constraint: Serve each block at most once" << endl;
}

void StochasticModel::attendingPath()
{
  int j, bl, n = graph->getN(), b = graph->getB();
  for (int s = 0; s <= graph->getS(); s++)
  {
    for (bl = 0; bl < b; bl++)
    {
      for (auto i : graph->nodes_per_block[bl])
      {
        GRBLinExpr served;
        for (auto *arc : graph->arcs[i])
          served += x[i][arc->getD()][s];

        model.addConstr(served >= y[i][bl][s], "att_path_" + to_string(i) + "_" + to_string(bl));
      }
    }
  }

  cout << "[***] Constraint: Include node in path" << endl;
}

void StochasticModel::timeConstraint()
{
  int i, j, n = graph->getN();

  for (int s = 0; s <= graph->getS(); s++)
  {
    GRBLinExpr arcTravel, blockTravel;
    for (i = 0; i < n; i++)
    {
      for (auto *arc : graph->arcs[i])
      {
        j = arc->getD();
        arcTravel += x[i][j][s] * arc->getLength();
      }

      for (auto b : graph->nodes[i].second)
        if (b != -1)
          blockTravel += y[i][b][s] * graph->time_per_block[b];
    }
    model.addConstr(arcTravel + blockTravel <= graph->getT(), "max_time");
  }

  cout << "[***] Constraint: time limit" << endl;
}

void StochasticModel::compactTimeConstraint()
{
  int b, i, j, k, n = graph->getN();

  for (int s = 0; s <= graph->getS(); s++)
  {
    for (i = 0; i <= n; i++)
    {
      if (i < n)
        model.addConstr(t[n][i][s] == 0);

      for (auto *arc : graph->arcs[i])
      {
        j = arc->getD();
        if (j >= n)
          continue;

        GRBLinExpr time_blocks_j = 0;
        for (auto b : graph->nodes[j].second)
          if (b != -1)
            time_blocks_j += y[j][b][s] * graph->time_per_block[b];

        for (auto *arcl : graph->arcs[j])
        {
          k = arcl->getD();
          model.addConstr(t[j][k][s] >= t[i][j][s] + time_blocks_j + x[j][k][s] * arc->getLength() - ((2 - x[i][j][s] - x[j][k][s]) * graph->getT()), "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
        }
      }
    }
    for (i = 0; i < n; i++)
      model.addConstr(t[i][n][s] <= graph->getT() * x[i][n][s], "max_time");
  }
  cout << "[***] Constraint: Time limit" << endl;
}

void StochasticModel::solveCompact(string timeLimit)
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

void StochasticModel::writeSolution(string result)
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

    for (int r = 0; r <= graph->getS(); r++)
    {
      // cout << "Scenario: " << r << endl;
      output << "S: " << r << endl;

      float timeUsed = 0;
      for (int i = 0; i <= n; i++)
      {
        for (auto *arc : graph->arcs[i])
        {
          j = arc->getD();
          if (x[i][j][r].get(GRB_DoubleAttr_X) > 0.5)
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
          if (y[i][b][r].get(GRB_DoubleAttr_X) > 0.5)
          {
            timeUsed += graph->time_per_block[b];
            if (r == 0)
              output << "Y: " << i << " " << b << " " << r << " = " << graph->cases_per_block[b] << endl;
            else
              output << "Y: " << i << " " << b << " " << r << " = " << graph->scenarios[r - 1].cases_per_block[b] << endl;
          }
        }
      }

      for (int b = 0; b < graph->getB(); b++)
      {
        if (z[b][r].get(GRB_DoubleAttr_X) > 0.0)
        {
          output << "Z: " << b << " " << r << " = " << z[b][r].get(GRB_DoubleAttr_X) << endl;
        }
      }

      output << "Route Time: " << timeUsed << endl;
    }
    cout << "OF: " << model.get(GRB_DoubleAttr_ObjVal) << endl;
    cout << "Finish write!" << endl;
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

bool StochasticModel::check_solution(float max_time, float max_insecticide)
{
  int n = graph->getN();

  for (int r = 0; r <= graph->getS(); r++)
  {
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
      for (auto b : graph->nodes[i].second)
      {
        if (y[i][b][r].get(GRB_DoubleAttr_X) > 0.5)
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
        if (x[i][arc->getD()][r].get(GRB_DoubleAttr_X) > 0.5)
        {
          time += arc->getLength();
          if (!used_arc[i][arc->getD()] && insecticide > 0)
          {
            cout << "[!!!] Not used arc on scenario: " << r << endl;
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
  }

  cout << "[***] Instance ok!!!" << endl;
  return true;
}
