// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/graph/graph.hpp"

#include <iostream>
#include <vector>

#define BOOST_TEST_MODULE connectedComponent

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace lemon;

BOOST_AUTO_TEST_CASE(Empty) {
  lemon::ListGraph graph;

  const int connectedComponentCount = lemon::countConnectedComponents(graph);
  BOOST_CHECK_EQUAL(0, connectedComponentCount);
}

BOOST_AUTO_TEST_CASE(OneCC) {
  lemon::ListGraph graph;
  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  const int connectedComponentCount = lemon::countConnectedComponents(graph);
  BOOST_CHECK_EQUAL(1, connectedComponentCount);
}

BOOST_AUTO_TEST_CASE(TwoCC) {
  lemon::ListGraph graph;

  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  lemon::ListGraph::Node a2 = graph.addNode(), b2 = graph.addNode();
  graph.addEdge(a2,b2);
  const int connectedComponentCount = lemon::countConnectedComponents(graph);
  BOOST_CHECK_EQUAL(2, connectedComponentCount);
}


BOOST_AUTO_TEST_CASE(TwoCC_Parsing) {
  lemon::ListGraph graph;

  lemon::ListGraph::Node a = graph.addNode(), b = graph.addNode();
  graph.addEdge(a,b);
  lemon::ListGraph::Node a2 = graph.addNode(), b2 = graph.addNode();
  graph.addEdge(a2,b2);

  typedef ListGraph::NodeMap<size_t> IndexMap;
  IndexMap connectedNodeMap(graph);
  const int connectedComponentCount =  lemon::connectedComponents(graph, connectedNodeMap);
  BOOST_CHECK_EQUAL(2, connectedComponentCount);
  for (IndexMap::MapIt it(connectedNodeMap); it != INVALID; ++it)
  {
    ALICEVISION_LOG_DEBUG(*it << "\t" << graph.id(it));
  }
}


/// Test to get back node id of each CC
// a
//
// b-c
//
// d-g
// | |
// e-f
//
// h-i-j-k
//   |/
//   l
BOOST_AUTO_TEST_CASE(exportGraphToMapSubgraphs_CC_Subgraph) {
  lemon::ListGraph graph;

  // single
  lemon::ListGraph::Node a = graph.addNode();

  // two
  lemon::ListGraph::Node b = graph.addNode(), c = graph.addNode();
  graph.addEdge(b,c);

  // four
  lemon::ListGraph::Node d = graph.addNode(), e = graph.addNode(),
    f = graph.addNode(), g = graph.addNode();
  graph.addEdge(d,e);
  graph.addEdge(e,f);
  graph.addEdge(f,g);
  graph.addEdge(g,d);

  // five
  lemon::ListGraph::Node h = graph.addNode(), i = graph.addNode(),
    j = graph.addNode(), k = graph.addNode(),l = graph.addNode();
  graph.addEdge(h,i);
  graph.addEdge(i,j);
  graph.addEdge(j,k);
  graph.addEdge(i,l);
  graph.addEdge(j,l);

  const std::map<size_t, std::set<lemon::ListGraph::Node> > map_subgraphs =
    aliceVision::graph::exportGraphToMapSubgraphs<lemon::ListGraph, size_t>(graph);

  BOOST_CHECK_EQUAL(4, map_subgraphs.size());
  BOOST_CHECK_EQUAL(5, map_subgraphs.at(0).size());
  BOOST_CHECK_EQUAL(4, map_subgraphs.at(1).size());
  BOOST_CHECK_EQUAL(2, map_subgraphs.at(2).size());
  BOOST_CHECK_EQUAL(1, map_subgraphs.at(3).size());
}
