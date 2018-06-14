// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>

#include <iostream>

namespace aliceVision {
namespace fuseCut {

/**
 * @brief Maxflow computation based on a standard Adjacency List graph reprensentation.
 *
 * @see MaxFlow_CSR which use less memory.
 */
class MaxFlow_AdjList
{
public:
    using NodeType = int;
    using ValueType = float;

    using Traits = boost::adjacency_list_traits<
                boost::vecS,  // OutEdgeListS
                boost::vecS,  // VertexListS
                boost::directedS,
                boost::vecS   // EdgeListS
                >;
    using edge_descriptor = typename Traits::edge_descriptor;
    using vertex_descriptor = typename Traits::vertex_descriptor;
    using vertex_size_type = typename Traits::vertices_size_type;
    struct Edge {
        ValueType capacity{};
        ValueType residual{};
        edge_descriptor reverse;
    };
    using Graph = boost::adjacency_list<boost::vecS,        // OutEdgeListS
                                        boost::vecS,        // VertexListS
                                        boost::directedS,
                                        boost::no_property, // VertexProperty
                                        Edge,               // EdgeProperty
                                        boost::no_property, // GraphProperty
                                        boost::vecS         // EdgeListS
                                        >;
    using VertexIterator = typename boost::graph_traits<Graph>::vertex_iterator;

public:
    explicit MaxFlow_AdjList(size_t numNodes)
        : _graph(numNodes+2)
        , _S(NodeType(numNodes))
        , _T(NodeType(numNodes+1))
    {
        VertexIterator vi, vi_end;
        for(boost::tie(vi, vi_end) = vertices(_graph); vi != vi_end; ++vi)
        {
            _graph.m_vertices[*vi].m_out_edges.reserve(9);
        }
        _graph.m_vertices[numNodes].m_out_edges.reserve(numNodes);
        _graph.m_vertices[numNodes+1].m_out_edges.reserve(numNodes);
    }

    inline void addNode(NodeType n, ValueType source, ValueType sink)
    {
        assert(source >= 0 && sink >= 0);
        ValueType score = source - sink;
        if(score > 0)
        {
            edge_descriptor edge(boost::add_edge(_S, n, _graph).first);
            edge_descriptor reverseEdge(boost::add_edge(n, _S, _graph).first);

            _graph[edge].capacity = score;
            _graph[edge].reverse = reverseEdge;
            _graph[reverseEdge].reverse = edge;
            _graph[reverseEdge].capacity = score;
        }
        else //if(score <= 0)
        {
            edge_descriptor edge(boost::add_edge(n, _T, _graph).first);
            edge_descriptor reverseEdge(boost::add_edge(_T, n, _graph).first);
            _graph[edge].capacity = -score;
            _graph[edge].reverse = reverseEdge;
            _graph[reverseEdge].reverse = edge;
            _graph[reverseEdge].capacity = -score;
        }
    }

    inline void addEdge(NodeType n1, NodeType n2, ValueType capacity, ValueType reverseCapacity)
    {
        assert(capacity >= 0 && reverseCapacity >= 0);

        edge_descriptor edge(boost::add_edge(n1, n2, _graph).first);
        edge_descriptor reverseEdge(boost::add_edge(n2, n1, _graph).first);
        _graph[edge].capacity = capacity;
        _graph[edge].reverse = reverseEdge;

        _graph[reverseEdge].capacity = reverseCapacity;
        _graph[reverseEdge].reverse = edge;
    }

    void printStats() const;
    void printColorStats() const;

    inline ValueType compute()
    {
        printStats();
        ALICEVISION_LOG_INFO("Compute boykov_kolmogorov_max_flow.");

        vertex_size_type nbVertices(boost::num_vertices(_graph));
        _color.resize(nbVertices, boost::white_color);
        std::vector<edge_descriptor> pred(nbVertices);
        std::vector<vertex_size_type> dist(nbVertices);

        ValueType v = boost::boykov_kolmogorov_max_flow(_graph,
            boost::get(&Edge::capacity, _graph),
            boost::get(&Edge::residual, _graph),
            boost::get(&Edge::reverse, _graph),
            &pred[0],
            &_color[0],
            &dist[0],
            boost::get(boost::vertex_index, _graph),
            _S, _T
            );

        printColorStats();

        return v;
    }

    /// is empty
    inline bool isSource(NodeType n) const
    {
        return (_color[n] == boost::black_color);
    }
    /// is full
    inline bool isTarget(NodeType n) const
    {
        return (_color[n] == boost::white_color);
    }

protected:
    Graph _graph;
    std::vector<boost::default_color_type> _color;
    const NodeType _S;  //< emptyness
    const NodeType _T;  //< fullness
};

} // namespace fuseCut
} // namespace aliceVision
