#pragma once
#include <immintrin.h>
#include <array>
#include <vector>
#include <random>
#include <utility>
#include <string>
#include <memory>
#include <limits>
#undef min
#undef max

#ifdef _MSC_VER
#define ALIGNED32 __declspec(align(32))
#else
#define ALIGNED32 __attribute__((aligned(32)))
#endif

#define POPSIFT_KDASSERT(x) if (!(x)) ::popsift::kdtree::assert_fail(#x, __FILE__, __LINE__)

/////////////////////////////////////////////////////////////////////////////

namespace popsift {
namespace kdtree {

inline void assert_fail(const char* expr, const char* file, int line) {
    throw std::logic_error(std::string("KDTree assertion failed: ") + expr + " @ " + file + std::to_string(line));
}

ALIGNED32 struct U8Descriptor {
    union {
        __m256i features[4];
        std::array<unsigned char, 128> ufeatures;
    };
};

struct BoundingBox {
    U8Descriptor min;
    U8Descriptor max;
};

constexpr int SPLIT_DIMENSION_COUNT = 5;    // Count of dimensions with highest variance to randomly split against
using SplitDimensions = std::array<unsigned char, SPLIT_DIMENSION_COUNT>;

// The code crashes unless this is correct.
static_assert(sizeof(unsigned) == 4, "Unsupported unsigned int size.");
static_assert(alignof(U8Descriptor) >= 32 && alignof(BoundingBox) >= 32, "Invalid alignment.");
static_assert(sizeof(U8Descriptor) == 128 && sizeof(BoundingBox) == 256, "Invalid size.");

/////////////////////////////////////////////////////////////////////////////

//! KDTree.  Node 0 is the root node.
class KDTree {
    friend std::unique_ptr<KDTree> Build(const U8Descriptor* descriptors, size_t dcount, unsigned leaf_size);
public:
    KDTree(const KDTree&) = delete;
    KDTree& operator=(const KDTree&) = delete;

    using Leaf = std::pair<const unsigned*, const unsigned*>;

    unsigned Root() const
    {
        return 0;
    }

    unsigned NodeCount() const
    {
        return static_cast<unsigned>(_nodes.size());
    }

    bool IsLeaf(unsigned n) const
    {
        return  _nodes.at(n).leaf;
    }

    unsigned Left(unsigned n) const
    {
        POPSIFT_KDASSERT(!_nodes.at(n).leaf);
        return n + 1;
    }

    unsigned Right(unsigned n) const
    {
        POPSIFT_KDASSERT(!_nodes.at(n).leaf);
        return _nodes.at(n).index;
    }

    unsigned Dim(unsigned n) const
    {
        POPSIFT_KDASSERT(!_nodes.at(n).leaf);
        return _nodes.at(n).dim();
    }

    unsigned Val(unsigned n) const
    {
        POPSIFT_KDASSERT(!_nodes.at(n).leaf);
        return _nodes.at(n).val();
    }
    
    const BoundingBox& BB(unsigned n) const
    {
        return _bb.at(n);
    }

    Leaf List(unsigned n) const {
        POPSIFT_KDASSERT(_nodes.at(n).leaf && _nodes.at(n).end <= _list.size());
        return List(_nodes.at(n).index, _nodes.at(n).end);
    }
    
    const U8Descriptor* Descriptors() const
    {
        return _descriptors;
    }

    size_t DescriptorCount() const
    {
        return _dcount;
    }

private:
    // There's no left link: if the parent is at index i, the left child is always at i+1 due to the way we build the tree.
    struct Node {
        unsigned index : 31;        // right link or begin list index if leaf == 1
        unsigned leaf : 1;          // 1 for leaf nodes
        union {
            unsigned char dv[2];    // splitting dimension and value if internal node
            unsigned end;           // end list index if leaf node
        };

        unsigned char& dim() { return dv[0]; }
        unsigned char& val() { return dv[1]; }

        unsigned char dim() const { return dv[0]; }
        unsigned char val() const { return dv[1]; }
    };

    static_assert(sizeof(Node) == 8, "Invalid size.");

    const std::uniform_int_distribution<int> _split_dim_gen;
    const U8Descriptor *_descriptors;   // Descriptor data
    const unsigned _dcount;             // Count of descriptors
    std::vector<BoundingBox> _bb;       // BBs of all nodes; packed linearly to not waste cache lines
    std::vector<Node> _nodes;           // Link nodes
    std::vector<unsigned> _list;        // Elements in leaf nodes; consecutive in range [left,right)

    // Used by Build
    unsigned _leaf_size;

    KDTree(const U8Descriptor* descriptors, size_t dcount);
    void Build(unsigned leaf_size);
    void Build(unsigned node_index,  unsigned lelem, unsigned relem);
    unsigned Partition(Node& node, unsigned lelem, unsigned relem);

    std::pair<unsigned*, unsigned*> List(unsigned l, unsigned r) const {
        return std::make_pair(
            const_cast<unsigned*>(_list.data() + l),
            const_cast<unsigned*>(_list.data() + r));
    }
};

using KDTreePtr = std::unique_ptr<KDTree>;

/////////////////////////////////////////////////////////////////////////////

//! Used by 2NN queries.
struct Q2NNAccumulator
{
    unsigned distance[2];
    unsigned index[2];

    Q2NNAccumulator()
    {
        distance[0] = distance[1] = std::numeric_limits<unsigned>::max();
        index[0] = index[1] = -1;
    }

    void Update(unsigned d, unsigned i)
    {
        if (d < distance[0]) {
            distance[1] = distance[0]; distance[0] = d;
            index[1] = index[0]; index[0] = i;
        }
        else if (d != distance[0] && d < distance[1]) {
            distance[1] = d;
            index[1] = i;
        }
        Validate();
    }
    
    Q2NNAccumulator Combine(const Q2NNAccumulator& other) const;

    void Validate() const
    {
        POPSIFT_KDASSERT(distance[0] < distance[1]);
        POPSIFT_KDASSERT(index[0] != index[1]);
    }
};

/////////////////////////////////////////////////////////////////////////////

unsigned L1Distance(const U8Descriptor&, const U8Descriptor&);
unsigned L1Distance(const U8Descriptor&, const BoundingBox&);
unsigned L2DistanceSquared(const U8Descriptor&, const U8Descriptor&);
unsigned L2DistanceSquared(const U8Descriptor&, const BoundingBox&);

void VerifyL2DistanceAVX();

std::pair<SplitDimensions, SplitDimensions> GetSplitDimensions(const U8Descriptor* descriptors, const unsigned* indexes, size_t count);
BoundingBox GetBoundingBox(const U8Descriptor* descriptors, const unsigned* indexes, size_t count);
BoundingBox Union(const BoundingBox& a, const BoundingBox& b);
KDTreePtr Build(const U8Descriptor* descriptors, size_t dcount, unsigned leaf_size);
std::vector<KDTreePtr> Build(const U8Descriptor* descriptors, size_t descriptor_count, size_t tree_count, unsigned leaf_size);
std::pair<unsigned, unsigned> Query2NN(const std::vector<KDTreePtr>& trees, const U8Descriptor& descriptor, size_t max_descriptors);

}   // kdtree
}   // popsift
