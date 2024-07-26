#include <string>
#include <iostream>

#define BOOST_TEST_MODULE your_test_module
#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_CASE(your_test_case)
{
  std::vector<int> a{1, 2};
  std::vector<int> b{1, 2};
  BOOST_TEST(a == b);
}

// int main(int argc, const char *argv[])
// {
//   std::cout << "Hello, World!" << std::endl;
//   return 0;
// }
