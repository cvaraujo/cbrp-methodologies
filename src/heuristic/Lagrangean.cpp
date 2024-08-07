//
// Created by carlos on 06/07/21.
//

#include "../headers/Lagrangean.h"

using namespace std;

Lagrangean::Lagrangean(Graph *graph)
{
  if (graph != nullptr)
    this->graph = graph;
  else
    exit(EXIT_FAILURE);
}

double Lagrangean::runSolverERCSPP(set<pair<int, int>> &x)
{
  int N = graph->getN(), o, d, i, j, k, b;
  GRBEnv env = GRBEnv();
  env.set("LogFile", "MS_mip.log");
  env.start();

  GRBModel model = GRBModel(env);

  vector<vector<GRBVar>> X = vector<vector<GRBVar>>(N + 2, vector<GRBVar>(N + 2));
  vector<vector<GRBVar>> W = vector<vector<GRBVar>>(N + 2, vector<GRBVar>(N + 2));

  // Creating Variables
  for (o = 0; o <= N; o++)
  {
    for (auto *arc : graph->arcs[o])
    {
      d = arc->getD();
      X[o][d] = model.addVar(0.0, 1.0, 0, GRB_CONTINUOUS, "x");
      W[o][d] = model.addVar(0.0, this->max_time, 0, GRB_CONTINUOUS, "w");
    }
  }

  // OF
  GRBLinExpr of;
  for (i = 0; i <= graph->getN(); i++)
  {
    for (auto arc : graph->arcs[i])
    {
      j = arc->getD();
      of += (-1 * X[i][j]) * (multTime * arc->getLength());

      // Connectors multipliers
      for (b = 0; b < graph->getPB(); b++)
        if (find(graph->nodes_per_block[b].begin(), graph->nodes_per_block[b].end(), j) != graph->nodes_per_block[b].end())
          of += X[i][j] * multConn[b];
    }
  }

  model.setObjective(of, GRB_MAXIMIZE);
  model.update();

  // Sink/Depot constraint
  GRBLinExpr sink, target;
  for (int i = 0; i < N; i++)
  {
    sink += X[N][i];
    target += X[i][N + 1];
  }

  model.addConstr(sink == 1, "sink_constraint");
  model.addConstr(target == 1, "target_constraint");
  cout << "[***] Contraint: dummy depot" << endl;

  // Flow Conservation Constraint
  for (i = 0; i < N; i++)
  {
    GRBLinExpr flow_out, flow_in;

    for (auto *arc : graph->arcs[i])
    {
      if (arc->getD() >= N)
        continue;
      flow_out += X[i][arc->getD()];
    }

    for (j = 0; j < N; j++)
    {
      for (auto *arc : graph->arcs[j])
      {
        if (arc->getD() == i)
          flow_in += X[j][i];
      }
    }

    flow_out += X[i][N + 1];
    flow_in += X[N][i];
    model.addConstr(flow_in - flow_out == 0, "flow_conservation_" + to_string(i));
  }
  cout << "[***] Constraint: Flow conservation" << endl;

  // Time Constraints
  for (i = 0; i <= N; i++)
  {
    if (i < N)
      model.addConstr(W[N][i] == 0);

    for (auto *arc : graph->arcs[i])
    {
      j = arc->getD();
      if (j >= N)
        continue;

      for (auto *arcl : graph->arcs[j])
      {
        k = arcl->getD();
        model.addConstr(W[j][k] >= W[i][j] + X[i][j] * arc->getLength() - ((2 - X[i][j] - X[j][k]) * max_time), "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }
    }
  }

  for (i = 0; i < N; i++)
    model.addConstr(W[i][N + 1] <= max_time * X[i][N + 1], "max_time_arrives");
  cout << "[***] Constraint: Time limit" << endl;

  //
  model.set("TimeLimit", "300");
  // model.set("OutputFlag", "0");
  model.update();
  model.write("model.lp");
  model.optimize();

  for (i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->arcs[i])
      if (X[i][arc->getD()].get(GRB_DoubleAttr_X) > 0)
        x.insert(make_pair(i, arc->getD()));

  return model.get(GRB_DoubleAttr_ObjVal);
}

double Lagrangean::solve_ppl(set<pair<int, int>> &x, vector<int> &y)
{
  int i, b, j;
  double arc_cost, of = multTime * graph->getT();

  // Update arcs costs
  // for (i = 0; i <= graph->getN(); i++)
  // {
  //   for (auto arc : graph->arcs[i])
  //   {
  //     j = arc->getD();

  //     // Time Multipliers
  //     arc_cost = multTime * arc->getLength();

  //     // Connectors multipliers
  //     for (b = 0; b < graph->getPB(); b++)
  //       if (find(graph->nodes_per_block[b].begin(), graph->nodes_per_block[b].end(), j) != graph->nodes_per_block[b].end())
  //         arc_cost -= multConn[b];

  //     graph->updateBoostArcCost(i, j, arc_cost);
  //   }
  // }

  cout << "Run RCSPP" << endl;
  // double route_cost = graph->run_spprc(x);
  double route_cost = runSolverERCSPP(x);

  if (route_cost >= numeric_limits<int>::max())
  {
    cout << "[!!!] No feasible route!" << endl;
    return numeric_limits<int>::max();
  }

  // cout << "[*] Route Cost: " << route_cost << endl;
  // for (auto p : x)
  //   cout << p.first << " -> " << p.second << ", ";
  // cout << endl;

  // Update Blocks profit
  vector<int> blocks, times, y_aux;
  vector<double> profit;
  for (b = 0; b < graph->getPB(); b++)
  {
    double coef = graph->cases_per_block[b] - multConn[b] - multTime * graph->time_per_block[b];
    if (coef > 0)
    {
      blocks.push_back(b);
      profit.push_back(coef);
      times.push_back(graph->time_per_block[b]);
    }
  }

  double knapsack_cost = graph->knapsack(y_aux, profit, times, graph->getT());

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

  std::cout << "[*] PPL OF: " << (of + route_cost + knapsack_cost) << endl;
  return of + route_cost + knapsack_cost;
}

int Lagrangean::bestAttendFromRoute(set<pair<int, int>> &x, vector<int> &y)
{

  set<int> route_blocks = graph->getBlocksFromRoute(x);
  vector<int> time;
  vector<double> cases;
  int i, j, avail_time = graph->getT();

  for (auto p : x)
  {
    i = p.first, j = p.second;
    auto arc = graph->getArc(i, j);
    avail_time -= arc->getLength();
  }

  graph->populateKnapsackVectors(route_blocks, cases, time);

  // cout << "----------" << endl;
  // for (auto b : route_blocks)
  //   cout << "B" << b << ", ";
  // cout << endl;
  // for (auto c : cases)
  //   cout << "C" << c << ", ";
  // cout << endl;
  // cout << "----------" << endl;

  vector<int> y_aux;
  int of = graph->knapsack(y_aux, cases, time, avail_time);

  for (int b : y_aux)
  {
    auto it = route_blocks.begin();
    std::advance(it, b);
    y.push_back(*it);
  }

  cout << "[*] Heuristic: " << of << ", With Res:" << avail_time << endl;
  // for (auto b : y)
  //   cout << "B" << b << ", ";
  // cout << endl;

  return of;
}

int Lagrangean::getOriginalObjValue(vector<int> y)
{
  int profit = 0;
  for (int b : y)
    profit += graph->cases_per_block[b];
  return profit;
}

void Lagrangean::getGradientTime(double &gradient_sigma, set<pair<int, int>> x, vector<int> y)
{
  gradient_sigma = -max_time;

  int i, j;
  for (auto p : x)
  {
    i = p.first, j = p.second;
    auto arc = graph->getArc(i, j);

    if (arc == nullptr)
      continue;

    gradient_sigma += arc->getLength();
  }

  for (int b : y)
    gradient_sigma += graph->time_per_block[b];

  if (gradient_sigma > 0)
    feasible = false;
}

void Lagrangean::getGradientConnection(vector<double> &gradient_lambda, set<pair<int, int>> x, vector<int> y)
{

  for (int b = 0; b < graph->getPB(); b++)
  {
    int delta_plus = 0;

    for (auto p : x)
    {
      int i = p.first, j = p.second;
      if (find(graph->nodes_per_block[b].begin(), graph->nodes_per_block[b].end(), j) != graph->nodes_per_block[b].end())
        delta_plus++;
    }

    gradient_lambda[b] = -delta_plus;
    if (find(y.begin(), y.end(), b) != y.end())
      gradient_lambda[b] += 1;

    if (gradient_lambda[b] > 0)
      feasible = false;
  }
}

double Lagrangean::getNorm(vector<double> &gradient)
{
  double sum = 0;
  for (int b = 0; b < graph->getPB(); b++)
    sum += pow(gradient[b], 2);
  return sqrt(sum);
}

bool Lagrangean::isFeasible()
{
  if (feasible)
    return true;
  feasible = true;
  return false;
}

int Lagrangean::lagrangean_relax()
{
  int progress = 0, iter = 0, N = graph->getN(), B = graph->getPB(), max_iter = 1000, reduce = 50;
  double thetaTime, normTime, thetaConn, normConn, objPpl, originalObj, routeObj;
  double lambda = 1.5;
  this->max_time = graph->getT();

  vector<double> gradientConn = vector<double>(B);
  double gradientTime = 0;

  multConn = vector<double>(B, 0);
  multTime = 0;
  UB = 0, LB = 0;

  for (auto cases : graph->cases_per_block)
    UB += cases;

  // cout << "Initial UB: " << UB << endl;

  while (iter < max_iter)
  {
    set<pair<int, int>> x;
    vector<int> y, y_aux;

    objPpl = solve_ppl(x, y);
    if (objPpl < numeric_limits<int>::max())
    {
      getGradientTime(gradientTime, x, y);
      getGradientConnection(gradientConn, x, y);

      if (objPpl < UB)
      {
        UB = objPpl;
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

      originalObj = getOriginalObjValue(y);
      double blocksFromRouteObj = bestAttendFromRoute(x, y_aux);

      cout << "Original Obj: " << originalObj << ", Heuristic: " << blocksFromRouteObj << ", Relax Obj: " << objPpl << endl;
      bool feasible = isFeasible();

      if ((feasible && originalObj > LB) || blocksFromRouteObj > LB)
      {
        LB = blocksFromRouteObj;
        if (feasible)
        {
          // cout << "[!] Found Feasible!" << endl;
          LB = max(originalObj, blocksFromRouteObj);
        }

        if ((UB - LB) < 1)
          return LB;
      }

      normConn = getNorm(gradientConn);
      normTime = sqrt(pow(gradientTime, 2));

      if (normTime == 0)
        thetaTime = 0;
      else
        thetaTime = lambda * ((objPpl - LB) / pow(normTime, 2));

      if (normConn == 0)
        thetaConn = 0;
      else
        thetaConn = lambda * ((objPpl - LB) / pow(normConn, 2));

      for (int b = 0; b < B; b++)
        multConn[b] = max(0.0, multConn[b] + (gradientConn[b] * thetaConn));

      multTime = max(0.0, multTime + (gradientTime * thetaTime));

      // cout << "Mult: " << multTime << endl;
      cout << "(Feasible) Lower Bound = " << LB << ", (Relaxed) Upper Bound = " << UB << endl;
      // getchar();
    }
    else
      return LB;
    iter++;
  }

  return LB;
}