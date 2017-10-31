#pragma once

typedef struct
{
    int rank;
    int p;
    int size;
} uni_elt;

/**
 * @brief Allows to perform labelling by creating node and connecting them.
 */
class mv_universe
{
public:
    mv_universe(int elements);
    ~mv_universe();
    /// Initialize all elements to the default values
    void initialize();
    /// Retrieve the smallest index of the elements connected to x.
    /// @warning: it updates the local indexes along the way
    int find(int x);
    void join(int x, int y);
    void addEdge(int x, int y);
    int size(int x) const { return elts[x].size; }
    int num_sets() const { return num; }

public:
    uni_elt* elts;
    int num, allelems;
};
