# graph

This module provides graph data structures and algorithms used throughout AliceVision, including indexed graphs, connected component analysis, and triplet enumeration.

## Overview

The `graph` module wraps the [Boost.Graph](https://www.boost.org/doc/libs/release/libs/graph/) library with AliceVision-specific types and adds higher-level algorithms commonly needed in Structure-from-Motion pipelines.

## IndexedGraph

The `IndexedGraph` class is a graph where nodes are identified by integer indices (`IndexT`). It supports directed and undirected edges.

## Connected Components

The `connectedComponent.hpp` header provides functions to compute and filter connected components of a graph:

```cpp
// Get all connected components of the graph
std::set<std::set<IndexT>> components = aliceVision::graph::graphToConnectedComponents(graph);
```

## Triplets

The `Triplet` struct and associated utilities enumerate all triplets (sets of three mutually connected nodes) in a graph. Triplets are used in Structure-from-Motion for rotation averaging and loop consistency checks:

```cpp
struct Triplet {
    IndexT i, j, k;
};
std::vector<Triplet> triplets;
aliceVision::graph::ListTriplets(graph, triplets);
```

## Graphviz Export

The `indexedGraphGraphvizExport.hpp` header provides a function to export an `IndexedGraph` to the Graphviz DOT format for visualization.
