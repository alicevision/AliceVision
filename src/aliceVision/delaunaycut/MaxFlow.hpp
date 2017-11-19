// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <iostream>


class MaxFlow
{
public:
    using NodeType = int;
    using ValueType = float;

    using edge_descriptor = typename boost::compressed_sparse_row_graph<boost::directedS>::edge_descriptor;

    struct Vertex {
        boost::default_color_type color{};
        ValueType distance{};
        edge_descriptor predecessor{};
    };
    struct Edge {
        Edge(ValueType pCapacity, ValueType pResidual)
            : capacity(pCapacity)
            , residual(pResidual)
        {}
        Edge(ValueType pCapacity, ValueType pResidual, edge_descriptor pReverse)
            : capacity(pCapacity)
            , residual(pResidual)
            , reverse(pReverse)
        {}
        Edge(){}

        ValueType capacity{};
        ValueType residual{};
        edge_descriptor reverse{};
    };
    using Graph = boost::compressed_sparse_row_graph<
        boost::directedS,
        Vertex, // VertexProperty
        Edge // EdgeProperty
        >;
    using vertex_descriptor = typename Graph::vertex_descriptor;
    using vertex_size_type = typename Graph::vertices_size_type;
    using edges_size_type = typename Graph::edges_size_type;
    // using VertexIterator = typename Graph::vertex_iterator;

public:
    MaxFlow(size_t numNodes)
        // : _graph(numNodes+2)
        : _numNodes(numNodes)
        , _S(NodeType(numNodes))
        , _T(NodeType(numNodes+1))
    {
        std::cout << "MaxFlow constructor" << std::endl;
        const std::size_t nbEdgesEstimation = numNodes * 9 + numNodes * 2;
        _edges.reserve(nbEdgesEstimation);
        _edgesData.reserve(nbEdgesEstimation);
    }

    inline void addNode(NodeType n, ValueType source, ValueType sink)
    {
        assert(source >= 0 && sink >= 0);
        ValueType score = source - sink;
        if(score > 0)
        {
            this->addEdge(_S, n, score, score);
        }
        else //if(score <= 0)
        {
            this->addEdge(n, _T, -score, -score);
        }
    }

    inline void addEdge(NodeType n1, NodeType n2, ValueType capacity, ValueType reverseCapacity)
    {
        assert(capacity >= 0 && reverseCapacity >= 0);
        
        int edgeIndex = _edges.size();
        _edges.push_back(std::make_pair(n1, n2)); // edge

        int reverseEdgeIndex = _edges.size();
        _edges.push_back(std::make_pair(n2, n1)); // reverse edge

        ValueType defaultResidual = 0.0;
        _edgesData.push_back(Edge(capacity, defaultResidual)); //, reverseEdgeIndex));
        _edgesData.push_back(Edge(reverseCapacity, defaultResidual)); //, edgeIndex));
    }

    void printStats() const;
    void printColorStats() const;

    inline ValueType compute()
    {
        printStats();
        std::cout << "Compute boykov_kolmogorov_max_flow" << std::endl;

        Graph graph(boost::edges_are_unsorted_multi_pass, _edges.begin(), _edges.end(), _numNodes);

        const vertex_size_type nbVertices = boost::num_vertices(graph);
        const edges_size_type nbEdges = boost::num_edges(graph);

        std::cout << "nbVertices: " << nbVertices << ", nbEdges: " << nbEdges << std::endl;

        Graph::edge_iterator edgeIt, edgeItEnd;
        for(boost::tie(edgeIt, edgeItEnd) = boost::edges(graph);
            edgeIt != edgeItEnd;)
        {
          Graph::vertex_descriptor v1 = boost::source(*edgeIt, graph);
          Graph::vertex_descriptor v2 = boost::target(*edgeIt, graph);
          Graph::edge_iterator edgeItA = edgeIt++;
          Graph::edge_iterator edgeItB = edgeIt++;

          graph[*edgeItA].reverse = *edgeItB;
          graph[*edgeItB].reverse = *edgeItA;
        }

        ValueType v = boost::boykov_kolmogorov_max_flow(graph,
            boost::get(&Edge::capacity, graph), // edge_capacity: The edge capacity property map.
            boost::get(&Edge::residual, graph), // edge_residual_capacity: The edge residual capacity property map.
            boost::get(&Edge::reverse, graph), // edge_reverse: An edge property map that maps every edge (u,v) in the graph to the reverse edge (v,u).
            boost::get(&Vertex::predecessor, graph), // vertex_predecessor: A vertex property map that stores the edge to the vertex' predecessor.
            boost::get(&Vertex::color, graph), // vertex_color
            boost::get(&Vertex::distance, graph), // vertex_distance
            boost::get(boost::vertex_index, graph), // this is not bundled, get it from graph
            _S, _T
            );

        printColorStats();
        _isTarget.resize(nbVertices);
        for(std::size_t vi = 0; vi < nbVertices; ++vi)
        {
            boost::default_color_type color =  graph[vi].color;
            _isTarget[vi] = (color == boost::white_color);
        }
        return v;
    }

    /// is empty
    inline bool isSource(NodeType n) const
    {
        return !_isTarget[n];
    }
    /// is full
    inline bool isTarget(NodeType n) const
    {
        return _isTarget[n];
    }

protected:
    std::size_t _numNodes;
    std::vector<std::pair<std::size_t, std::size_t>> _edges;
    std::vector<Edge> _edgesData;
    std::vector<bool> _isTarget;
    const NodeType _S;  //< emptyness
    const NodeType _T;  //< fullness
};
