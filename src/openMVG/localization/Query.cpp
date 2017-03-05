#include "Query.h"
#include "KDTree.h"
#include <queue>
#include <tbb/tbb.h>
#undef min
#undef max

#define DISTANCE_CHECK(a, b) L2DistanceSquared(a, b);

namespace popsift {
namespace kdtree {

// 2-NN query ///////////////////////////////////////////////////////////////

class Q2NNquery
{
    struct PQEntry {
        unsigned distance;
        unsigned tree;
        unsigned node;
        friend bool operator<(const PQEntry& e1, const PQEntry& e2) {
            return e1.distance > e2.distance;   // Reverse heap ordering; smallest on top
        }
    };

    const std::vector<KDTreePtr>& _trees;
    const U8Descriptor& _descriptor;
    const size_t _max_descriptors;

    std::priority_queue<PQEntry> _pq;
    size_t _found_descriptors;
    Q2NNAccumulator _result;

    void TraverseToLeaf(PQEntry pqe);
    void ProcessLeaf(const KDTree& tree, unsigned node);

    
public:
    Q2NNquery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors);
    std::pair<const DescriptorAssociation*, const DescriptorAssociation*> Run();
};

Q2NNquery::Q2NNquery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors) :
    _trees(trees), _descriptor(descriptor), _max_descriptors(max_descriptors)
{
    _found_descriptors = 0;
}

std::pair<const DescriptorAssociation*, const DescriptorAssociation*> Q2NNquery::Run()
{
#if 1
    for (unsigned i = 0; i < _trees.size(); ++i) {
        unsigned d = DISTANCE_CHECK(_descriptor, _trees[i]->BB(0));
        _pq.push(PQEntry{ d, i, 0 });
    }
#else

#endif

    while (_found_descriptors < _max_descriptors && !_pq.empty()) {
        PQEntry pqe = _pq.top();
        _pq.pop();
        if (pqe.distance <= _result.distance[1])    // We're searching 2NN, so test 2nd-best distance
            TraverseToLeaf(pqe);
    }

    return std::make_pair(_result.index[0], _result.index[1]);
}

void Q2NNquery::TraverseToLeaf(PQEntry pqe)
{
    const KDTree& tree = *_trees[pqe.tree];
    unsigned node = pqe.node;

    while (!tree.IsLeaf(node)) {
        unsigned l = tree.Left(node), dl = DISTANCE_CHECK(_descriptor, tree.BB(l));
        unsigned r = tree.Right(node), dr = DISTANCE_CHECK(_descriptor, tree.BB(r));

        if (dl <= dr) {
            node = l;
            pqe.node = r; pqe.distance = dr;
            _pq.push(pqe);
        }
        else {
            node = r;
            pqe.node = l; pqe.distance = dl;
            _pq.push(pqe);
        }
    }
    ProcessLeaf(tree, node);
}

void Q2NNquery::ProcessLeaf(const KDTree& tree, unsigned node)
{
    auto list = tree.List(node);

    _found_descriptors += list.second - list.first;
    for (; list.first != list.second; ++list.first) {
        unsigned d = DISTANCE_CHECK(_descriptor, tree.Descriptors()[list.first->global_index]);
        _result.Update(d, list.first);
    }
}

QueryResult Query2NN(const std::vector<KDTreePtr>& trees, size_t max_candidates,
    const U8Descriptor* queries, size_t query_count)
{
    // Sanity check: all trees must be built with the same descriptors.
    {
        const U8Descriptor* tree_descriptors = trees.front()->Descriptors();
        for (const auto& t : trees) POPSIFT_KDASSERT(t->Descriptors() == tree_descriptors);
    }

    QueryResult result(query_count);
    tbb::parallel_for(size_t(0), query_count, [&](size_t i) {
        Q2NNquery q(trees, queries[i], max_candidates);
        auto r = q.Run();
        result[i] = std::make_tuple(i, *r.first, *r.second);
    });
    return result;
}

}
}
