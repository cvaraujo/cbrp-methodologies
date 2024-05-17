//
// Created by carlos on 06/07/21.
//

#include "../headers/Lagrangean.h"
#include "../headers/WarmStart.h"

using namespace std;

Lagrangean::Lagrangean(Graph *graph, float default_vel, float spraying_vel, float insecticide_ml_min)
{
  if (graph != nullptr)
  {
    this->graph = graph;
    this->default_vel = default_vel;
    this->spraying_vel = spraying_vel;
    this->insecticide_ml_min = insecticide_ml_min;
  }
  else
    exit(EXIT_FAILURE);
}

void Lagrangean::createVariables()
{
  int o, d, k, n = graph->getN(), m = graph->getM(), b = graph->getB();
  try
  {
    env.set("LogFile", "MS_mip.log");
    env.start();

    x = vector<vector<GRBVar>>(n + 1, vector<GRBVar>(n + 1));
    y = vector<GRBVar>(b);
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
    for (int bl = 0; bl < graph->getB(); bl++)
    {
      sprintf(name, "y_%d", bl);
      y[bl] = model.addVar(0.0, 1.0, 0, GRB_BINARY, name);
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

void Lagrangean::initModelCompact(float maxTime, float maxInsecticide, bool warm_start)
{
  cout << "[!!!] Creating the model!" << endl;
  createVariables();
  lagrange_of();
  artificialNodes(), flowConservation();
  timeConstraint(maxTime), compactTimeConstraint(maxTime);

  cout << "[***] done!" << endl;
}

void Lagrangean::artificialNodes()
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

void Lagrangean::flowConservation()
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

void Lagrangean::timeConstraint(float maxTime)
{
  int i, j, n = graph->getN();
  GRBLinExpr arcTravel, blockTravel;

  for (i = 0; i < n; i++)
  {
    for (auto *arc : graph->arcs[i])
    {
      j = arc->getD();
      arcTravel += x[i][j] * timeArc(arc->getLength(), this->default_vel);
    }
  }

  for (int b = 0; b < graph->getB(); b++)
    blockTravel += y[b] * timeBlock(b, this->spraying_vel);

  model.addConstr(arcTravel <= maxTime, "max_travel_time");
  // model.addConstr(blockTravel <= maxTime, "max_block_time");

  std::cout << "[***] Constraint: time limit" << endl;
}

void Lagrangean::compactTimeConstraint(float maxTime)
{
  int b, i, j, k, n = graph->getN();
  vector<float> block_time = vector<float>(graph->getB(), -1);

  for (i = 0; i <= n; i++)
  {
    if (i < n)
      model.addConstr(t[n][i] == 0);

    for (auto *arc : graph->arcs[i])
    {
      j = arc->getD();
      if (j >= n)
        continue;

      GRBLinExpr time_ij = 0;
      time_ij += t[i][j] + (timeArc(arc->getLength(), this->default_vel) * x[i][j]);

      for (auto *arcl : graph->arcs[j])
      {
        k = arcl->getD();
        model.addConstr(t[j][k] >= t[i][j] + x[i][j] * timeArc(arc->getLength(), this->default_vel) - ((2 - x[i][j] - x[j][k]) * maxTime), "t_geq_" + to_string(i) + "_" + to_string(j) + "_" + to_string(k));
      }
    }
  }

  for (i = 0; i < n; i++)
    model.addConstr(t[i][n] <= maxTime * x[i][n], "max_time_arrives");
  cout << "[***] Constraint: Time limit" << endl;
}

float Lagrangean::timeArc(float distance, float speed)
{
  return distance > 0 ? distance / ((speed * 1000) / 60) : 0;
}

float Lagrangean::timeBlock(int block, float speed)
{
  if (block == -1)
    return 0;

  float time = 0;
  for (auto *arc : graph->arcsPerBlock[block])
  {
    time += timeArc(arc->getLength(), speed);
  }
  return time;
}

float Lagrangean::inseticideBlock(int block, float perMeter)
{
  float consumed = 0;
  for (auto *arc : graph->arcsPerBlock[block])
  {
    consumed += timeArc(arc->getLength(), this->spraying_vel) * perMeter;
  }
  return consumed;
}

bool Lagrangean::solveCompact(string timeLimit)
{
  try
  {
    model.set("TimeLimit", timeLimit);
    model.update();
    // model.set("OutputFlag", "0");
    model.write("model.lp");
    model.optimize();
    return true;
  }
  catch (GRBException &ex)
  {
    std::cout << ex.getMessage() << endl;
  }
  return false;
}

void Lagrangean::lagrange_of()
{
  GRBLinExpr objective, arc_time, blocks_time, dependency;
  int i, j, b, N = graph->getN(), B = graph->getB();

  for (i = 0; i < N; i++)
  {
    for (auto arc : graph->arcs[i])
    {
      if (arc->getD() >= N)
        continue;

      j = arc->getD();
      arc_time += x[i][j] * timeArc(arc->getLength(), default_vel);
    }
  }

  for (b = 0; b < B; b++)
  {
    GRBLinExpr delta_minus;
    for (int k : graph->nodesPerBlock[b])
      for (auto arc : graph->arcs[k])
        delta_minus += x[arc->getO()][arc->getD()];

    dependency += multConn[b] * (delta_minus - y[b]);
    blocks_time += y[b] * timeBlock(b, spraying_vel);
  }

  for (b = 0; b < B; b++)
    objective += y[b] * graph->cases_per_block[b] + multTime * (max_time - (arc_time + blocks_time)) + dependency;

  model.setObjective(objective, GRB_MAXIMIZE);
  model.update();

  std::cout << "[***] Obj. Function: Maximize profit" << endl;
}

int Lagrangean::getOriginalObjValue()
{
  int blocks = 0;
  for (int b = 0; b < graph->getB(); b++)
    if (y[b].get(GRB_DoubleAttr_X) > 0.1)
      blocks += graph->cases_per_block[b];

  return blocks;
}

void Lagrangean::getGradientTime(double &gradient_sigma)
{
  gradient_sigma = -max_time;
  // cout << "Tot. Time: " << gradient_sigma << endl;

  for (int i = 0; i <= graph->getN(); i++)
    for (auto arc : graph->arcs[i])
      if (x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.1)
        gradient_sigma += timeArc(arc->getLength(), default_vel);

  // cout << "Route Time: " << gradient_sigma << endl;
  for (int b = 0; b < graph->getB(); b++)
    if (y[b].get(GRB_DoubleAttr_X) > 0.1)
      gradient_sigma += timeBlock(b, spraying_vel);

  // cout << "Block + Route Time: " << gradient_sigma << endl;

  if (gradient_sigma > 0)
    feasible = false;
}

void Lagrangean::getGradientConnection(vector<double> &gradient_lambda)
{
  for (int b = 0; b < graph->getB(); b++)
  {
    int delta_minus = 0;

    for (auto i : graph->nodesPerBlock[b])
      for (auto arc : graph->arcs[i])
        if (x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.1)
          delta_minus += 1;

    gradient_lambda[b] = y[b].get(GRB_DoubleAttr_X) - delta_minus;

    if (gradient_lambda[b] > 0)
      feasible = false;
  }
}

double Lagrangean::getNorm(vector<double> &gradient)
{
  double sum = 0;
  for (int b = 0; b < graph->getB(); b++)
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
  int progress = 0, iter = 0, N = graph->getN(), B = graph->getB(), max_iter = 1000;
  double thetaTime, normTime, thetaConn, normConn, objPpl, originalObj;
  double lambda = 0.15;
  this->max_time = 120;

  vector<double> gradientConn = vector<double>(B);
  double gradientTime = 0;

  multConn = vector<double>(B, 0);
  multTime = 0;

  UB = 0, LB = 0;

  for (auto cases : graph->cases_per_block)
    UB += cases;

  while (iter < max_iter)
  {
    initModelCompact(max_time, 0, false);

    if (solveCompact("3600"))
    {
      getGradientTime(gradientTime);
      getGradientConnection(gradientConn);

      // cout << "Solution" << endl;
      // for (int b = 0; b < B; b++)
      //   if (y[b].get(GRB_DoubleAttr_X) > 0.1)
      //     cout << "Y" << b << endl;

      // for (int i = 0; i <= N; i++)
      //   for (auto arc : graph->arcs[i])
      //     if (x[i][arc->getD()].get(GRB_DoubleAttr_X) > 0.1)
      //       cout << "X" << i << "-" << arc->getD() << endl;

      // cout << "Gradient Time: " << gradientTime << endl;
      for (int b = 0; b < B; b++)
        // cout << "Gradiente Conn: " << gradientConn[b] << endl;

        objPpl = model.get(GRB_DoubleAttr_ObjVal);

      if (objPpl < UB)
        UB = objPpl, progress = 0;
      else
      {
        progress++;
        if (progress == 5)
        {
          lambda *= 0.9;
          progress = 0;
        }
      }

      originalObj = getOriginalObjValue();
      cout << "Original Obj: " << originalObj << ", Computed Obj: " << objPpl << endl;

      if (isFeasible() && originalObj > LB)
      {
        LB = originalObj;
        if ((UB - LB) < 1)
        {
          cout << "Result: " << LB << " - " << UB << endl;
          return LB;
        }
      }

      normConn = getNorm(gradientConn);
      normTime = sqrt(pow(gradientTime, 2));

      // cout << "Norm: " << normTime << " - " << normConn << endl;

      if (normTime == 0)
        thetaTime = 0;
      else
        thetaTime = lambda * ((objPpl - LB) / pow(normTime, 2));

      if (normConn == 0)
        thetaConn = 0;
      else
        thetaConn = lambda * ((objPpl - LB) / pow(normConn, 2));

      // cout << "Theta: " << thetaTime << " - " << thetaConn << endl;
      for (int b = 0; b < B; b++)
      {
        multConn[b] = max(0.0, multConn[b] - (gradientConn[b] * thetaConn));
        // cout << "Mult Conn: " << multConn[b] << endl;
      }

      multTime = max(0.0, multTime - (gradientTime * thetaTime));
      // cout << "Mult Time: " << multTime << endl;

      cout << "(Feasible) Lower Bound = " << LB << ", (Relaxed) Upper Bound = " << UB << endl;
      model.reset(0);
      getchar();
    }
    iter++;
  }
  return LB;
}