//
// Created by carlos on 06/07/21.
//

#ifndef DPARP_INCLUDE_H
#define DPARP_INCLUDE_H

using namespace std;

#include <iostream>
#include <vector>
#include "string"
#include <iomanip>
#include <bits/ios_base.h>
#include <algorithm>
#include <fstream>
#include <deque>
#include <limits>
#include <random>

#include <boost/config.hpp>

#ifdef BOOST_MSVC
#pragma warning(disable : 4267)
#endif

#include "boost/algorithm/string.hpp"
#include "boost/algorithm/string/trim.hpp"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>

#include <lemon/list_graph.h>
#include <lemon/preflow.h>

const int INF = INT_MAX;

typedef pair<int, int> int_pair;

#endif // DPARP_INCLUDE_H
