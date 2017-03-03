#include "KDTree.h"
#include <queue>
#include <tbb/tbb.h>
#undef min
#undef max

#define DISTANCE_CHECK(a, b) L2DistanceSquared(a, b);

namespace popsift {
namespace kdtree {

struct NodeDistance {
    unsigned distance;
    unsigned tree;
    unsigned node;
    friend bool operator<(const NodeDistance& e1, const NodeDistance& e2) {   // Reverse heap ordering; smallest on top
        return e1.distance > e2.distance;
    }
};

class NNQuery {
    const std::vector<KDTreePtr>& _trees;
    const U8Descriptor& _descriptor;
    const size_t _max_candidates;

    std::priority_queue<NodeDistance> _pq;
    size_t _found_candidates = 0;
    unsigned _best_distance = -1U;
    KDTree::DescriptorAssociation _result;

    void TraverseToLeaf(NodeDistance nd);
    void ProcessLeaf(const KDTree& tree, unsigned node);
    
public:
    NNQuery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_candidates);
    KDTree::DescriptorAssociation Run();
};

NNQuery::NNQuery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_candidates) :
    _trees(trees), _descriptor(descriptor), _max_candidates(max_candidates)
{
}

KDTree::DescriptorAssociation NNQuery::Run()
{
    // Initialize distances from each root node.
    for (unsigned i = 0; i < _trees.size(); ++i) {
        unsigned d = DISTANCE_CHECK(_descriptor, _trees[i]->BB(0));
        _pq.push(NodeDistance{ d, i, 0 });
    }

    while (_found_candidates < _max_candidates && !_pq.empty()) {
        NodeDistance nd = _pq.top();
        _pq.pop();
        if (nd.distance <= _best_distance)
            TraverseToLeaf(nd);
    }

    return _result;
}

void NNQuery::TraverseToLeaf(NodeDistance nd)
{
    const KDTree& tree = *_trees[nd.tree];
    unsigned node = nd.node;

    while (!tree.IsLeaf(node)) {
        unsigned l = tree.Left(node), dl = DISTANCE_CHECK(_descriptor, tree.BB(l));
        unsigned r = tree.Right(node), dr = DISTANCE_CHECK(_descriptor, tree.BB(r));

        if (dl <= dr) {
            node = l;
            nd.node = r;
            nd.distance = dr;
            _pq.push(nd);
        }
        else {
            node = r;
            nd.node = l;
            nd.distance = dl;
            _pq.push(nd);
        }
    }
    ProcessLeaf(tree, node);
}

void NNQuery::ProcessLeaf(const KDTree& tree, unsigned node)
{
    auto list = tree.List(node);

    _found_candidates += list.second - list.first;
    for (; list.first != list.second; ++list.first) {
        unsigned d = DISTANCE_CHECK(_descriptor, tree.Descriptors()[list.first->global_index]);
        if (d < _best_distance) {
            _best_distance = d;
            _result = *list.first;
        }
    }
}

std::vector<KDTree::DescriptorAssociation>
Query(const std::vector<KDTreePtr>& trees, size_t max_candidates,
    const U8Descriptor* descriptors, size_t descriptor_count)
{
    // Sanity check: all trees refer to the same descriptors.
    {
        const U8Descriptor* descriptors = trees.front()->Descriptors();
        for (const auto& t : trees) POPSIFT_KDASSERT(t->Descriptors() == descriptors);
    }
    
    std::vector<KDTree::DescriptorAssociation> result(descriptor_count);
    tbb::parallel_for(size_t(0), descriptor_count, [&](unsigned int i) {
        NNQuery q(trees, descriptors[i], max_candidates);
        result[i] = q.Run();
    });

    return result;
}

}
}
