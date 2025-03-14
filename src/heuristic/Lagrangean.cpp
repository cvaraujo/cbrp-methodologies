//
// Created by carlos on 06/07/21.
//

#include "Lagrangean.hpp"

Lagrangean::Lagrangean(Input *input)
{
  if (input != nullptr)
  {
    this->input = input;
    this->boost = new BoostLibrary(input);
  }
  else
    exit(EXIT_FAILURE);
}

pair<int, double> Lagrangean::runSolverERCSPP(set<pair<int, int>> &x)
{
  Graph *graph = this->input->getGraph();
  int N = graph->getN(), o, d, i, j, k, b;
  int T = input->getT(), B = graph->getB();

  GRBEnv env = GRBEnv();
  env.set("LogFile", "MS_mip.log");
  env.start();

  GRBModel model = GRBModel(env);
  vector<vector<GRBVar>> X = vector<vector<GRBVar>>(N + 2, vector<GRBVar>(N + 2));
  vector<vector<GRBVar>> W = vector<vector<GRBVar>>(N + 2, vector<GRBVar>(N + 2));

  // Creating Variables
  for (o = 0; o <= N; o++)
  {
    for (auto *arc : graph->getArcs(o))
    {
      d = arc->getD();
      X[o][d] = model.addVar(0.0, GRB_INFINITY, 0, GRB_INTEGER, "x_" + to_string(o) + "_" + to_string(d));
      W[o][d] = model.addVar(0.0, T, 0, GRB_CONTINUOUS, "w");
    }
  }

  // OF
  GRBLinExpr of;
  for (i = 0; i <= graph->getN(); i++)
  {
    for (auto arc : graph->getArcs(i))
    {
      j = arc->getD();
      of += X[i][j] * mult_time * arc->getLength();
      // if (mult_time != 0)
      // {
      //   cout << "[1 -->] i: " << i << ", j: " << j << ", mult: " << mult_time * arc->getLength() << endl;
      //   getchar();
      // }
    }
  }

  for (b = 0; b < B; b++)
  {
    for (int i : graph->getNodesFromBlock(b))
    {
      for (auto arc : graph->getArcs(i))
      {
        j = arc->getD();
        of += -1 * X[i][j] * mult_conn[b];
        // if (mult_conn[b] != 0)
        // {
        //   cout << "[2 -->] i: " << i << ", j: " << j << ", b: " << b << ", mult: " << mult_conn[b] << endl;
        //   getchar();
        // }
      }
    }
  }

  model.setObjective(of, GRB_MINIMIZE);
  model.update();

  // Sink/Depot constraint
  GRBLinExpr sink, target;
  for (int i = 0; i < N; i++)
  {
    sink += X[N][i];
    target += X[i][N];
  }

  model.addConstr(sink == 1, "sink_constraint");
  model.addConstr(target == 1, "target_constraint");
  cout << "[***] Contraint: dummy depot" << endl;

  // Flow Conservation Constraint
  for (i = 0; i < N; i++)
  {
    GRBLinExpr flow_out, flow_in;

    for (auto *arc : graph->getArcs(i))
    {
      if (arc->getD() >= N)
        continue;
      flow_out += X[i][arc->getD()];
    }

    for (j = 0; j < N; j++)
    {
      for (auto *arc : graph->getArcs(j))
      {
        if (arc->getD() == i)
          flow_in += X[j][i];
      }
    }

    flow_out += X[i][N];
    flow_in += X[N][i];
    model.addConstr(flow_in - flow_out == 0, "flow_conservation_" + to_string(i));
  }
  cout << "[***] Constraint: Flow conservation" << endl;

  // Time Constraints
  for (i = 0; i <= N; i++)
  {
    if (i < N)
      model.addConstr(W[N][i] == 0);

    for (auto *arc : graph->getArcs(i))
    {
      j = arc->getD();
      if (j >= N)
        continue;

      for (auto *arcl : graph->getArcs(j))
      {
        k = arcl->getD();
        model.addConstr(W[j][k] >= W[i][j] + (X[i][j] * arc->getLength()) - ((2 - X[i][j] - X[j][k]) * T), "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }
    }
  }

  for (i = 0; i < N; i++)
    model.addConstr(W[i][N] <= T * X[i][N], "max_time_arrives_" + to_string(i));
  cout << "[***] Constraint: Time limit" << endl;

  X[N][0].set(GRB_DoubleAttr_Start, 1.0);
  X[0][N].set(GRB_DoubleAttr_Start, 1.0);

  model.set("TimeLimit", "300");
  // model.set("OutputFlag", "0");
  model.update();
  model.write("model.lp");
  model.optimize();

  int route_time = 0.0;
  for (i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->getArcs(i))
      if (X[i][arc->getD()].get(GRB_DoubleAttr_X) > 0)
      {
        int ip = i;
        if (i == N)
          ip = N + 1;
        int_pair p = make_pair(ip, arc->getD());
        route_time += arc->getLength();
        x.insert(p);
      }

  return make_pair(route_time, model.get(GRB_DoubleAttr_ObjVal));
}

pair<int, double> Lagrangean::runSHPRC(set<pair<int, int>> &x)
{
  // Update arcs costs
  Graph *graph = input->getGraph();
  int i, j, b, N = graph->getN();
  double arc_cost;

  // Update arcs
  // cout << "[*] Updating arcs" << endl;
  for (i = 0; i < graph->getN(); i++)
  {
    // Connectors multipliers
    double block_cost = 0.0;
    for (auto b : graph->getNode(i).second)
    {
      // cout << "[-->]" << b << " " << mult_conn[b] << endl;
      block_cost += mult_conn[b];
    }
    // getchar();

    for (auto arc : graph->getArcs(i))
    {
      j = arc->getD();

      // Time Multipliers
      arc_cost = (mult_time * arc->getLength() - block_cost);

      this->boost->update_arc_cost(i, j, arc_cost);
    }
  }

  return this->boost->run_spprc(x);
}

double Lagrangean::solve_ppl(set<pair<int, int>> &x, vector<int> &y)
{
  Graph *graph = input->getGraph();
  int i, b, j, T = input->getT();
  double arc_cost, of = mult_time * T;

  pair<int, double> x_result = runSHPRC(x);
  // pair<int, double> x_result = runSolverERCSPP(x);
  double route_time = x_result.first, route_cost = x_result.second;

  if (route_cost >= numeric_limits<int>::max())
  {
    cout << "[!!!] No feasible route!" << endl;
    return numeric_limits<int>::max();
  }

  cout << "[*] Route Cost: " << route_cost << endl;
  cout << "[*] Route Time: " << route_time << endl;
  this->curr_route_time = route_time;

  // Update Blocks profit
  vector<int> blocks, times, y_aux;
  vector<double> profit;
  for (b = 0; b < graph->getB(); b++)
  {
    int time_block = graph->getTimePerBlock(b);
    double coef = graph->getCasesPerBlock(b) - mult_conn[b] - (mult_time * (1.0 * time_block));

    if (coef > 0)
    {
      blocks.push_back(b);
      profit.push_back(coef);
      times.push_back(time_block);
    }
  }

  double knapsack_cost = Knapsack::Run(y_aux, profit, times, T);

  for (auto b : y_aux)
  {
    auto it = blocks.begin();
    std::advance(it, b);
    y.push_back(*it);
  }

  // cout << "[*] Knapsack Profit: " << knapsack_cost << endl;
  // for (auto b : y)
  //   cout << b << ", ";
  // cout << endl;

  cout << "[*] PPL OF: " << (of - route_cost + knapsack_cost) << endl
       << "\t[-] OF: " << of << endl
       << "\t[-] Route Cost: " << route_cost << endl
       << "\t[-] Knapsack Cost: " << knapsack_cost << endl;
  return of - route_cost + knapsack_cost;
}

int Lagrangean::bestAttendFromRoute(const set<pair<int, int>> &x, vector<int> &y)
{
  Graph *graph = input->getGraph();
  set<int> route_blocks = graph->getBlocksFromRoute(x);
  vector<int> time, y_aux;
  vector<double> cases;
  int i, j, avail_time = this->input->getT();

  if (!route_blocks.size())
    return 0;

  // Get route time
  for (auto p : x)
  {
    i = p.first, j = p.second;
    if (i >= graph->getN() || j >= graph->getN())
      continue;

    auto arc = graph->getArc(i, j);
    avail_time -= arc->getLength();
  }

  // Get blocks to Knapsack
  vector<int> blocks, times;
  vector<double> profit;
  for (int b : route_blocks)
  {
    int time_block = graph->getTimePerBlock(b);
    double cases = graph->getCasesPerBlock(b);
    if (cases > 0)
    {
      blocks.push_back(b);
      profit.push_back(cases);
      times.push_back(time_block);
    }
  }

  int of = Knapsack::Run(y_aux, profit, times, avail_time);

  for (int b : y_aux)
  {
    auto it = route_blocks.begin();
    std::advance(it, b);
    y.push_back(*it);
  }

  cout << "[*] Heuristic: " << of << ", With Time: " << avail_time << endl;
  // for (auto b : y)
  //   cout << "B" << b << ", ";
  // cout << endl;

  return of;
}

int Lagrangean::getOriginalObjValue(vector<int> y)
{
  int profit = 0;
  Graph *graph = input->getGraph();
  for (int b : y)
    profit += graph->getCasesPerBlock(b);
  return profit;
}

int Lagrangean::getGradientTime(set<pair<int, int>> x, vector<int> y)
{
  Graph *graph = input->getGraph();
  int N = graph->getN();
  double gradient_time = input->getT();

  // int i, j;
  // for (auto p : x)
  // {
  //   i = p.first, j = p.second;
  //   if (i > N)
  //     i = N;

  //   auto arc = graph->getArc(i, j);
  //   if (arc == nullptr)
  //   {
  //     cout << "[Error!] Arc not found" << endl;
  //     exit(EXIT_FAILURE);
  //   }

  //   gradient_sigma += arc->getLength();
  // }
  gradient_time -= this->curr_route_time;

  for (int b : y)
    gradient_time -= graph->getTimePerBlock(b);

  if (gradient_time < 0)
    is_feasible = false;

  return gradient_time;
}

void Lagrangean::getGradientConnection(vector<double> &gradient_lambda, set<pair<int, int>> x, vector<int> y)
{
  Graph *graph = input->getGraph();
  int i, j, b, B = graph->getB(), N = graph->getN();
  map<pair<int, int>, bool> arc_used;

  for (auto arc : x)
    arc_used[arc] = true;

  for (b = 0; b < B; b++)
  {
    gradient_lambda[b] = 0;
    if (find(y.begin(), y.end(), b) != y.end())
      gradient_lambda[b] = -1;

    set<int> nodes = graph->getNodesFromBlock(b);

    for (int i : nodes)
      for (auto arc : graph->getArcs(i))
        if (arc_used[make_pair(i, arc->getD())])
          gradient_lambda[b]++;

    if (gradient_lambda[b] < 0)
      is_feasible = false;
  }
}

double Lagrangean::getNorm(vector<double> &gradient)
{
  Graph *graph = input->getGraph();
  int B = graph->getB();
  double sum = 0;

  for (int b = 0; b < B; b++)
  {
    if (gradient[b] != 0)
      sum += pow(gradient[b], 2);
  }
  return sqrt(sum);
}

bool Lagrangean::isFeasible()
{
  if (is_feasible)
    return true;

  is_feasible = true;
  return false;
}

int Lagrangean::lagrangean_relax()
{
  Graph *graph = input->getGraph();
  int progress = 0, iter = 0, N = graph->getN(), B = graph->getB(), max_iter = 1000, reduce = 10;
  double theta_time, norm_time, theta_conn, norm_conn, obj_ppl, original_obj, route_obj, heuristic_obj;
  double lambda = 1.0;
  this->T = input->getT();
  is_feasible = true;

  vector<double> gradient_conn = vector<double>(B, 0);
  double gradient_time = 0.0;

  mult_conn = vector<double>(B, 0);
  mult_time = 0;
  UB = 0, LB = 0;

  for (int b = 0; b < B; b++)
    UB += graph->getCasesPerBlock(b);

  cout << "Initial UB: " << UB << endl;

  while (iter < max_iter)
  {
    set<pair<int, int>> x;
    vector<int> y, y_aux;

    cout << "[*] Solving PPL in iteration " << iter << "..." << endl;
    obj_ppl = solve_ppl(x, y);

    // cout << "[*] Solving PPL in iteration " << iter << " finished!" << endl;
    if (obj_ppl < numeric_limits<int>::max())
    {
      // cout << "[*] Gradients..." << endl;
      gradient_time = getGradientTime(x, y);
      // cout << "[*] Gradient Time: " << gradient_time << endl;

      getGradientConnection(gradient_conn, x, y);

      cout << "[*] Updating bounds..." << endl;
      if (obj_ppl < UB)
      {
        UB = obj_ppl;
        progress = 0;
      }
      else
      {
        progress++;
        if (progress == reduce)
        {
          lambda *= 0.9;
          progress = 0;
        }
      }

      // cout << "[*] Get Original Objective..." << endl;
      original_obj = getOriginalObjValue(y);

      // cout << "[*] Get Heuristic Objective..." << endl;
      heuristic_obj = bestAttendFromRoute(x, y_aux);

      for (auto p : x)
        cout << p.first << " -> " << p.second << ", ";
      cout << endl;

      for (auto b : y)
        cout << b << ", ";
      cout << endl;

      cout << "[*] Original Obj: " << original_obj << ", Heuristic: " << heuristic_obj << ", Relax Obj: " << obj_ppl << endl;
      bool feasible = isFeasible();

      if ((feasible && original_obj > LB) || heuristic_obj > LB)
      {
        LB = heuristic_obj;
        if (feasible)
          LB = max(original_obj, heuristic_obj);
      }

      if ((UB - LB) < 1)
      {
        cout << "[!!!] Found optimal solution!" << endl
             << "(Feasible) Lower Bound = " << LB << ", (Relaxed) Upper Bound = " << UB << endl;
        return LB;
      }

      norm_conn = getNorm(gradient_conn);
      norm_time = sqrt(pow(gradient_time, 2));

      // cout << "[*] NormConn: " << norm_conn << ", NomTime: " << norm_time << endl;

      // cout << "[*} Lambda: " << lambda << endl;
      // cout << "OBJ: " << obj_ppl << ", LB: " << LB << ", |NormConn|: " << pow(norm_conn, 2) << ", |Normtime|: " << pow(norm_conn, 2) << endl;
      if (norm_time == 0)
        theta_time = 0;
      else
        theta_time = lambda * (float(obj_ppl - LB) / pow(norm_time, 2));

      // cout << "[*] Theta Time: " << theta_time << endl;

      if (norm_conn == 0)
        theta_conn = 0;
      else
        theta_conn = lambda * (float(obj_ppl - LB) / pow(norm_conn, 2));

      // cout << "[*] Theta Conn: " << theta_conn << endl;

      for (int b = 0; b < B; b++)
      {

        mult_conn[b] = max(0.0, mult_conn[b] - (gradient_conn[b] * theta_conn));
        // cout << "mult_conn[" << b << "]: " << mult_conn[b] << ", gradient_conn[" << b << "]: " << gradient_conn[b] << " = " << mult_conn[b] << endl;
      }

      mult_time = max(0.0, mult_time - (gradient_time * theta_time));
      // cout << "mult_time: " << mult_time << endl;

      cout << "(Feasible) Lower Bound = " << LB << ", (Relaxed) Upper Bound = " << UB << endl;
      getchar();
    }
    else
      return LB;
    iter++;
  }

  cout << "[!!!] (Feasible) Lower Bound = " << LB << ", (Relaxed) Upper Bound = " << UB << endl;
  return LB;
}