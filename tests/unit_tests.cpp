#include "../src/common/BlockConnection.hpp"
#include "../src/common/Knapsack.hpp"
#include "../src/classes/Graph.hpp"
#include "../src/classes/Input.hpp"
#include "../src/exact/DeterministicModel.hpp"
#include "../src/exact/StochasticModel.hpp"

#define BOOST_TEST_MODULE your_test_module
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(testGraph)
{
  Graph graph("/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-graph.txt", 20, 10);

  // Basics
  BOOST_TEST(graph.getB() == 3);
  BOOST_TEST(graph.getN() == 3);
  BOOST_TEST(graph.getM() == 6);

  // Nodes from block
  set<int> nodes_0{0, 1, 2}, nodes_1{2}, nodes_2{2};
  BOOST_TEST(graph.getNodesFromBlock(0) == nodes_0);
  BOOST_TEST(graph.getNodesFromBlock(1) == nodes_1);
  BOOST_TEST(graph.getNodesFromBlock(2) == nodes_2);

  // Blocks from route
  set<int> blocks{0, 1, 2};
  BOOST_TEST(graph.getBlocksFromRoute({make_pair(3, 2), make_pair(2, 3)}) == blocks);

  // Arcs
  Arc *arc1 = graph.getArc(0, 1);
  Arc *arc2 = graph.getArc(3, 4);
  BOOST_TEST(arc1->getO() == 0);
  BOOST_TEST(arc1->getD() == 1);
  BOOST_TEST(arc2 == nullptr);

  // Cases per block
  BOOST_TEST(graph.getCasesPerBlock(0) == 20);
  BOOST_TEST(graph.getCasesPerBlock(1) == 10);
  BOOST_TEST(graph.getCasesPerBlock(2) == 0);

  // Time per block
  BOOST_TEST(graph.getTimePerBlock(0) == 180);
  BOOST_TEST(graph.getTimePerBlock(1) == 0);
  BOOST_TEST(graph.getTimePerBlock(2) == 0);
}

BOOST_AUTO_TEST_CASE(testKnapsack)
{
  vector<double> cases{1, 2, 3, 4};
  vector<int> time{5, 2, 3, 4}, y;
  int MT = 5;

  double result = Knapsack::Run(y, cases, time, MT);
  vector<int> correct{2, 1};

  BOOST_TEST(result == 5);
  BOOST_TEST(y == correct);
}

BOOST_AUTO_TEST_CASE(testInput)
{
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 1200;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  // Basics
  BOOST_TEST(input->getAlpha() == alpha);
  BOOST_TEST(input->getT() == T);
  BOOST_TEST(input->getS() == 4);

  // Scenarios
  vector<double> cases_0{10, 21, 100};
  vector<double> cases_1{10, 21, 0};
  BOOST_TEST(input->getScenario(0).getCases() == cases_0);
  BOOST_TEST(input->getScenario(1).getCases() == cases_1);
  BOOST_TEST(input->getScenario(0).getProbability() == 0.25);
}

BOOST_AUTO_TEST_CASE(testShortestPath)
{
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 1200;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  ShortestPath *sp = input->getShortestPath();

  // Shortest path ST
  vector<int> path;
  int cost = sp->ShortestPathST(0, 2, path);
  vector<int> correct_path{0, 2};

  BOOST_TEST(cost == 30);
  BOOST_TEST(path == correct_path);

  // Djikstra layered DAG
  vector<int> pred;
  vector<vector<Arc>> dag;
  dag.push_back(vector<Arc>({Arc{0, 1, 0, 0}, Arc{0, 2, 0, 0}}));
  dag.push_back(vector<Arc>({Arc{1, 3, 10, 0}}));
  dag.push_back(vector<Arc>({Arc{2, 3, 10, 0}}));
  dag.push_back(vector<Arc>({Arc{3, 4, 0, 0}}));
  dag.push_back(vector<Arc>());

  correct_path = {0, 0, 0, 1, 3};
  cost = sp->DijkstraLayeredDAG(dag, 5, 0, 4, pred);

  BOOST_TEST(cost == 10);
  BOOST_TEST(pred == correct_path);

  //
  set<int> nodes;
  map<int, map<int, bool>> arcs;
  cost = sp->SHPBetweenBlocks(2, 0, nodes, arcs);
  set<int> correct_nodes{0};

  BOOST_TEST(cost == 0);
  BOOST_TEST(arcs.empty());

  BOOST_TEST(nodes == correct_nodes);
}

BOOST_AUTO_TEST_CASE(testBlockConnection)
{
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/tests/test_instance/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 1200;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  BlockConnection *bc = input->getBlockConnection();

  // Static function
  vector<int> a{1, 2, 3, 4};
  string correct = "1234";
  string wrong = "123";
  string result = bc->GenerateStringFromIntVector(a);

  BOOST_TEST(result == correct);
  BOOST_TEST(result != wrong);

  // Heuristic Block Connection
  string key = "012";
  vector<int> blocks = {0, 1, 2};
  vector<int> corect_path = {1, 0, 1};
  int cost = bc->HeuristicBlockConnection(input->getGraph(), input->getShortestPath(), blocks, key);

  BOOST_TEST(cost == 0);
  BOOST_TEST(bc->getBlocksAttendCost(key) == 0);
  BOOST_TEST(bc->getBlocksAttendPath(key) == corect_path);
}

BOOST_AUTO_TEST_CASE(testDeterministicModelCompact)
{
  cout << "Deterministic Model Compact" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  DeterministicModel *dm = new DeterministicModel(input);
  Solution sol = dm->Run(false, "60", "MTZ", false);

  sol.WriteSolution("result_deterministic_mtz.txt");

  BOOST_TEST(sol.getOf() == 80);
}

BOOST_AUTO_TEST_CASE(testDeterministicModelExp)
{
  cout << "Deterministic Model Exponential" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 1000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  DeterministicModel *dm = new DeterministicModel(input);
  Solution sol = dm->Run(false, "60", "EXP", false);

  sol.WriteSolution("result_deterministic_exp.txt");

  BOOST_TEST(sol.getOf() == 68);
}

BOOST_AUTO_TEST_CASE(testDeterministicModelExpFrac)
{
  cout << "Deterministic Model Exponential Frac. Cuts" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  DeterministicModel *dm = new DeterministicModel(input);
  Solution sol = dm->Run(false, "60", "EXP", true);

  sol.WriteSolution("result_deterministic_exp_frac.txt");

  BOOST_TEST(sol.getOf() == 80);
}

BOOST_AUTO_TEST_CASE(testStochasticModelCompact)
{
  cout << "Stochastic Model Compact" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  StochasticModel *sm = new StochasticModel(input);
  Solution sol = sm->Run(false, "60", "MTZ", false);

  sol.WriteSolution("result_stochastic_mtz.txt");

  BOOST_TEST(sol.getOf() == 241);
}

BOOST_AUTO_TEST_CASE(testStochasticModelExp)
{
  cout << "Stochastic Model Exponential" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  StochasticModel *sm = new StochasticModel(input);
  Solution sol = sm->Run(false, "60", "EXP", false);

  sol.WriteSolution("result_stochastic_mtz.txt");

  BOOST_TEST(sol.getOf() == 241);
}

BOOST_AUTO_TEST_CASE(testStochasticModelExpFrac)
{
  cout << "Stochastic Model Exponential Frac. Cuts" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-graph.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/instances/test/test-scenarios.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  StochasticModel *sm = new StochasticModel(input);
  Solution sol = sm->Run(false, "60", "EXP", true);

  sol.WriteSolution("result_stochastic_mtz.txt");

  BOOST_TEST(sol.getOf() == 241);
}

BOOST_AUTO_TEST_CASE(checkStochasticModels)
{
  cout << "Stochastic Model Exponential Frac. Cuts" << endl;
  string file_graph = "/home/araujo/Documents/cbrp-methodologies/instances/simulated-alto-santo/alto-santo-200-1.txt";
  string file_scenarios = "/home/araujo/Documents/cbrp-methodologies/instances/simulated-alto-santo/scenarios-alto-santo-200-1.txt";
  int graph_adapt = 1, default_vel = 20, neblize_vel = 10, T = 12000;
  double alpha = 0.8;

  Input *input = new Input(file_graph, file_scenarios, graph_adapt, default_vel, neblize_vel, T, alpha);

  StochasticModel *sm = new StochasticModel(input);
  StochasticModel *sm2 = new StochasticModel(input);
  StochasticModel *sm3 = new StochasticModel(input);

  Solution sol = sm->Run(false, "600", "MTZ", false);
  Solution sol2 = sm2->Run(false, "600", "EXP", false);
  Solution sol3 = sm3->Run(false, "600", "EXP", true);

  sol.WriteSolution("result_stochastic_mtz.txt");
  sol2.WriteSolution("result_stochastic_exp.txt");
  sol3.WriteSolution("result_stochastic_exp_frac.txt");

  cout << sol.getOf() << ", " << sol2.getOf() << ", " << sol3.getOf() << endl;

  BOOST_TEST(round(sol.getOf()) == round(sol2.getOf()));
  BOOST_TEST(round(sol3.getOf()) == round(sol2.getOf()));
}