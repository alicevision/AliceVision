#include "Query.h"
#include "KDTree.h"
#include <boost/container/flat_set.hpp>
#include <tbb/tbb.h>
#undef min
#undef max

#define DISTANCE_CHECK(a, b) L2DistanceSquared(a, b);

namespace popsift {
namespace kdtree {

Q2NNAccumulator Q2NNAccumulator::Combine(const Q2NNAccumulator& other) const
{
    Q2NNAccumulator r;

    if (distance[0] == other.distance[0]) {
        r.distance[0] = distance[0];
        r.index[0] = index[0];

        if (distance[1] < other.distance[1]) {
            r.distance[1] = distance[1];
            r.index[1] = index[1];
        }
        else {
            r.distance[1] = other.distance[1];
            r.index[1] = other.index[1];
        }
    }
    else if (distance[0] < other.distance[0]) {
        r.distance[0] = distance[0];
        r.index[0] = index[0];

        if (other.distance[0] < distance[1]) {
            r.distance[1] = other.distance[0];
            r.index[1] = other.index[0];
        }
        else {
            r.distance[1] = distance[1];
            r.index[1] = index[1];
        }
    }
    else {
        r.distance[0] = other.distance[0];
        r.index[0] = other.index[0];

        if (distance[0] < other.distance[1]) {
            r.distance[1] = distance[0];
            r.index[1] = index[0];
        }
        else {
            r.distance[1] = other.distance[1];
            r.index[1] = other.index[1];
        }
    }

    r.Validate();
    return r;
}

class Q2NNpq    // std::priority_queue doesn't support preallocation
{
public:
    struct Entry {
        unsigned distance;
        unsigned tree;
        unsigned node;
        friend bool operator<(const Entry& e1, const Entry& e2) {
            return e1.distance > e2.distance;   // Reverse heap ordering; smallest on top
        }
    };

    Q2NNpq()
    {
        _pq.reserve(4096);  // Should be more than #trees * #levels to avoid allocations on Push/Pop
    }

    template<typename Mutex>
    void Push(const Entry& e, Mutex& mtx)
    {
        Mutex::scoped_lock lk(mtx);
        Push(e);
    }

    template<typename Mutex>
    bool Pop(Entry& e, Mutex& mtx)
    {
        Mutex::scoped_lock lk(mtx);
        return Pop(e);
    }

private:
    void Push(const Entry& e)
    {
        _pq.push_back(e);
        std::push_heap(_pq.begin(), _pq.end());
    }

    bool Pop(Entry& e)
    {
        if (_pq.empty())
            return false;
        e = _pq.front();
        std::pop_heap(_pq.begin(), _pq.end());
        _pq.pop_back();
        return true;
    }

    std::vector<Entry> _pq;
};

class Q2NNquery
{
    const std::vector<KDTreePtr>& _trees;
    const U8Descriptor& _descriptor;
    const size_t _max_descriptors;

    Q2NNpq _pq;
    tbb::null_mutex _pqmtx;
    size_t _found_descriptors;
    std::vector<unsigned> _leaf_new_descriptors;
    Q2NNAccumulator _result;

    void TraverseToLeaf(Q2NNpq::Entry pqe);
    void ProcessLeaf(const KDTree& tree, unsigned node);

    
public:
    Q2NNquery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors);
    std::pair<unsigned, unsigned> Run();
};

Q2NNquery::Q2NNquery(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors) :
    _trees(trees), _descriptor(descriptor), _max_descriptors(max_descriptors)
{
    _found_descriptors = 0;
    _leaf_new_descriptors.reserve(2048);
}

std::pair<unsigned, unsigned> Q2NNquery::Run()
{
#if 1
    for (unsigned i = 0; i < _trees.size(); ++i) {
        unsigned d = DISTANCE_CHECK(_descriptor, _trees[i]->BB(0));
        _pq.Push(Q2NNpq::Entry{ d, i, 0 }, _pqmtx);
    }
#else

#endif

    Q2NNpq::Entry pqe;
    while (_found_descriptors < _max_descriptors && _pq.Pop(pqe, _pqmtx))
    if (pqe.distance <= _result.distance[1])    // We're searching 2NN, so test 2nd-best distance
        TraverseToLeaf(pqe);

    return std::make_pair(_result.index[0], _result.index[1]);
}

void Q2NNquery::TraverseToLeaf(Q2NNpq::Entry pqe)
{
    const KDTree& tree = *_trees[pqe.tree];
    unsigned node = pqe.node;

    while (!tree.IsLeaf(node)) {
        unsigned l = tree.Left(node), dl = DISTANCE_CHECK(_descriptor, tree.BB(l));
        unsigned r = tree.Right(node), dr = DISTANCE_CHECK(_descriptor, tree.BB(r));

        if (dl <= dr) {
            node = l;
            pqe.node = r; pqe.distance = dr;
            _pq.Push(pqe, _pqmtx);
        }
        else {
            node = r;
            pqe.node = l; pqe.distance = dl;
            _pq.Push(pqe, _pqmtx);
        }
    }
    ProcessLeaf(tree, node);
}

void Q2NNquery::ProcessLeaf(const KDTree& tree, unsigned node)
{
    auto list = tree.List(node);

    _found_descriptors += list.second - list.first;
    for (; list.first != list.second; ++list.first) {
        unsigned d = DISTANCE_CHECK(_descriptor, tree.Descriptors()[*list.first]);
        _result.Update(d, *list.first);
    }
}

std::pair<unsigned, unsigned> Query2NN(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors)
{
    const U8Descriptor* descriptors = trees.front()->Descriptors();
    for (const auto& t : trees) POPSIFT_KDASSERT(t->Descriptors() == descriptors);
    Q2NNquery q(trees, descriptor, max_descriptors);
    return q.Run();
}

}
}
