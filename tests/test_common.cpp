#include "../src/common/BlockConnection.hpp"
#include "../src/common/Knapsack.hpp"
#include "../src/classes/Graph.hpp"

#define BOOST_TEST_MODULE your_test_module
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(testGraph)
{
  Graph graph("/home/araujo/Documents/cbrp-lagrangean/tests/test_instance/test-graph.txt", 20, 10);

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

BOOST_AUTO_TEST_CASE(generateStringFromIntVector)
{
  vector<int> a{1, 2, 3, 4};
  string correct = "1234";
  string wrong = "123";
  string result = BlockConnection::GenerateStringFromIntVector(a);

  BOOST_TEST(result == correct);
  BOOST_TEST(result != wrong);
}

BOOST_AUTO_TEST_CASE(knapsack)
{
  vector<double> cases{1, 2, 3, 4};
  vector<int> time{5, 2, 3, 4}, y;
  int MT = 5;

  double result = Knapsack::Run(y, cases, time, MT);
  vector<int> correct{2, 1};

  BOOST_TEST(result == 5);
  BOOST_TEST(y == correct);
}