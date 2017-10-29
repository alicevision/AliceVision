#pragma once

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>


template <typename NType, typename VType>
class MaxFlow
{
public:
    typedef NType node_type;
    typedef VType value_type;

    typedef boost::vecS out_edge_list_t;
    typedef boost::vecS vertex_list_t;
    typedef boost::adjacency_list_traits<out_edge_list_t, vertex_list_t, boost::directedS> graph_traits;
    typedef typename graph_traits::edge_descriptor edge_descriptor;
    typedef typename graph_traits::vertex_descriptor vertex_descriptor;
    typedef typename graph_traits::vertices_size_type vertex_size_type;
    struct Edge {
        value_type capacity;
        value_type residual;
        edge_descriptor reverse;
    };
    typedef boost::adjacency_list<out_edge_list_t, vertex_list_t, boost::directedS, size_t, Edge> graph_type;
    typedef typename boost::graph_traits<graph_type>::edge_iterator edge_iterator;
    typedef typename boost::graph_traits<graph_type>::out_edge_iterator out_edge_iterator;

public:
    MaxFlow(size_t numNodes)
        : _graph(numNodes+2)
        , _S(node_type(numNodes))
        , _T(node_type(numNodes+1))
    {}

    inline void addNode(node_type n, value_type source, value_type sink)
    {
        assert(source >= 0 && sink >= 0);
        if(source > 0)
        {
            edge_descriptor edge(boost::add_edge(_S, n, _graph).first);
            edge_descriptor reverseEdge(boost::add_edge(n, _S, _graph).first);

            _graph[edge].capacity = source;
            _graph[edge].reverse = reverseEdge;
            _graph[reverseEdge].reverse = edge;
        }
        if(sink > 0)
        {
            edge_descriptor edge(boost::add_edge(n, _T, _graph).first);
            edge_descriptor reverseEdge(boost::add_edge(_T, n, _graph).first);
            _graph[edge].capacity = sink;
            _graph[edge].reverse = reverseEdge;
            _graph[reverseEdge].reverse = edge;
        }
    }

    inline void addEdge(node_type n1, node_type n2, value_type capacity, value_type reverseCapacity)
    {
        assert(capacity >= 0 && reverseCapacity >= 0);

        edge_descriptor edge(boost::add_edge(n1, n2, _graph).first);
        edge_descriptor reverseEdge(boost::add_edge(n2, n1, _graph).first);
        _graph[edge].capacity = capacity;
        _graph[edge].reverse = reverseEdge;

        _graph[reverseEdge].capacity = reverseCapacity;
        _graph[reverseEdge].reverse = edge;
    }

    inline value_type compute()
    {
        std::cout << "Compute boost::boykov_kolmogorov_max_flow" << std::endl;

        vertex_size_type nbVertices(boost::num_vertices(_graph));
        _color.resize(nbVertices);
        std::vector<edge_descriptor> pred(nbVertices);
        std::vector<vertex_size_type> dist(nbVertices);

        return boost::boykov_kolmogorov_max_flow(_graph,
            boost::get(&Edge::capacity, _graph),
            boost::get(&Edge::residual, _graph),
            boost::get(&Edge::reverse, _graph),
            &pred[0],
            &_color[0],
            &dist[0],
            boost::get(boost::vertex_index, _graph),
            _S, _T
        );
    }

    inline bool isSource(node_type n) const
    {
        return (_color[n] != boost::white_color);
    }

protected:
    graph_type _graph;
    std::vector<boost::default_color_type> _color;
    const node_type _S;
    const node_type _T;
};
