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
#include <boost/graph/compressed_sparse_row_graph.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/container/flat_map.hpp>

#include <iostream>

namespace aliceVision {
namespace fuseCut {

/**
 * @brief Maxflow computation based on a compressed sparse row graph reprensentation.
 *
 * @note: The graph itself consumes much less memory than AdjList, but this improvement is
 * limited by the creation of the reverse edges which requires the creation of a temporary map.
 * The memory peak is still slightly better than AdjList version, but it would be great to find
 * another way to retrieve the reverse edges, as it could reduce a lot the required amount of memory.
 */
class MaxFlow_CSR
{
public:
    using NodeType = unsigned int;
    using ValueType = float;

    using edge_descriptor = typename boost::compressed_sparse_row_graph<
        boost::directedS,
        boost::no_property, // VertexProperty
        boost::no_property, // EdgeProperty
        unsigned int,       // Vertex
        unsigned int        // EdgeIndex
        >::edge_descriptor;

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
        Vertex,       // VertexProperty
        Edge,         // EdgeProperty
        unsigned int, // Vertex
        unsigned int  // EdgeIndex
        >;
    using vertex_descriptor = typename Graph::vertex_descriptor;
    using vertex_size_type = typename Graph::vertices_size_type;
    using edges_size_type = typename Graph::edges_size_type;

public:
    explicit MaxFlow_CSR(size_t numNodes)
        : _numNodes(numNodes+2)
        , _S(NodeType(numNodes))
        , _T(NodeType(numNodes+1))
    {
        ALICEVISION_LOG_INFO("MaxFlow constructor.");
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
        
        //int edgeIndex = _edges.size();
        _edges.push_back(std::make_pair(n1, n2)); // edge

        //int reverseEdgeIndex = _edges.size();
        _edges.push_back(std::make_pair(n2, n1)); // reverse edge

        ValueType defaultResidual = 0.0;
        _edgesData.push_back(Edge(capacity, defaultResidual)); //, reverseEdgeIndex));
        _edgesData.push_back(Edge(reverseCapacity, defaultResidual)); //, edgeIndex));
    }

    inline ValueType compute()
    {
        ALICEVISION_LOG_INFO("Compute boykov_kolmogorov_max_flow.");

        Graph graph(boost::edges_are_unsorted_multi_pass, _edges.begin(), _edges.end(), _edgesData.begin(), _numNodes);
        // Graph graph(boost::edges_are_unsorted, _edges.begin(), _edges.end(), _edgesData.begin(), _numNodes);
        // Graph graph(boost::edges_are_sorted, _edges.begin(), _edges.end(), _edgesData.begin(), _numNodes);

        const vertex_size_type nbVertices = boost::num_vertices(graph);
        const edges_size_type nbEdges = boost::num_edges(graph);

        ALICEVISION_LOG_INFO("# vertices: " << nbVertices);
        ALICEVISION_LOG_INFO("# edges: " << nbEdges);

        /*
        // Cannot be used as it is far too compute intensive
        Graph::edge_iterator ei, ee;
        for(boost::tie(ei, ee) = boost::edges(graph); ei != ee; ++ei)
        {
            Graph::vertex_descriptor v1 = boost::source(*ei, graph);
            Graph::vertex_descriptor v2 = boost::target(*ei, graph);
            std::pair<Graph::edge_descriptor, bool> opp_edge = boost::edge(v2, v1, graph); // VERY compute intensive

            graph[opp_edge.first].reverse = *ei; // and edge_reverse of *ei will be (or already have been) set by the opp_edge
        }
        */
        {
            // flat_map use less memory than std::map
            // This step represent the memory peak, would be great to find a way to reduce it.
            boost::container::flat_map<std::pair<Graph::vertex_descriptor, Graph::vertex_descriptor>, Graph::edge_descriptor> edgeDescriptors;
            Graph::edge_iterator ei, ee;
            for(boost::tie(ei, ee) = boost::edges(graph); ei != ee; ++ei)
            {
                Graph::vertex_descriptor v1 = boost::source(*ei, graph);
                Graph::vertex_descriptor v2 = boost::target(*ei, graph);
                edgeDescriptors[std::make_pair(v1, v2)] = *ei;
            }
            for(boost::tie(ei, ee) = boost::edges(graph); ei != ee; ++ei)
            {
                Graph::vertex_descriptor v1 = boost::source(*ei, graph);
                Graph::vertex_descriptor v2 = boost::target(*ei, graph);
                graph[*ei].reverse = edgeDescriptors[std::make_pair(v2, v1)];
            }
        }
        ALICEVISION_LOG_INFO("boykov_kolmogorov_max_flow: start.");
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
        ALICEVISION_LOG_INFO("boykov_kolmogorov_max_flow: done.");

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

} // namespace fuseCut
} // namespace aliceVision
