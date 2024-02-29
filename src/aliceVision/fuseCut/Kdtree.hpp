// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "nanoflann.hpp"

namespace aliceVision {
namespace fuseCut {

static const std::size_t MAX_LEAF_ELEMENTS = 10;

struct PointVectorAdaptator
{
    using Derived = PointVectorAdaptator;  //!< In this case the dataset class is myself.
    using T = double;

    const std::vector<Point3d>& _data;
    PointVectorAdaptator(const std::vector<Point3d>& data)
      : _data(data)
    {}

    /// CRTP helper method
    inline const Derived& derived() const { return *static_cast<const Derived*>(this); }
    /// CRTP helper method
    inline Derived& derived() { return *static_cast<Derived*>(this); }

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return _data.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const { return _data.at(idx).m[dim]; }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template<class BBOX>
    bool kdtree_get_bbox(BBOX& bb) const
    {
        return false;
    }
};

typedef nanoflann::L2_Simple_Adaptor<double, PointVectorAdaptator> KdTreeDist;
typedef nanoflann::KDTreeSingleIndexAdaptor<KdTreeDist, PointVectorAdaptator, 3> KdTree;

template<typename DistanceType, typename IndexType = size_t>
class SmallerPixSizeInRadius
{
  public:
    const DistanceType radius;

    const std::vector<double>& m_pixSizePrepare;
    const std::vector<float>& m_simScorePrepare;
    size_t m_result = 0;
    const int m_i;
    bool found = false;

    inline SmallerPixSizeInRadius(DistanceType radius_, const std::vector<double>& pixSizePrepare, const std::vector<float>& simScorePrepare, int i)
      : radius(radius_),
        m_pixSizePrepare(pixSizePrepare),
        m_simScorePrepare(simScorePrepare),
        m_i(i)
    {
        init();
    }

    inline void init() { clear(); }
    inline void clear() { m_result = 0; }

    inline size_t size() const { return m_result; }

    inline bool full() const { return found; }

    /**
     * Called during search to add an element matching the criteria.
     * @return true if the search should be continued, false if the results are sufficient
     */
    inline bool addPoint(DistanceType dist, IndexType index)
    {
        if (dist < radius)
        {
            ++m_result;
            if (m_simScorePrepare[index] * m_pixSizePrepare[index] * m_pixSizePrepare[index] <
                m_simScorePrepare[m_i] * m_pixSizePrepare[m_i] * m_pixSizePrepare[m_i])
            {
                found = true;
                return false;
            }
        }
        return true;
    }

    inline DistanceType worstDist() const { return radius; }
};

class Tree
{
    std::unique_ptr<KdTree> _tree;
    std::unique_ptr<PointVectorAdaptator> _pointCloudRef;

  public:
    Tree(const std::vector<Point3d>& verticesCoords) { initKdTree(verticesCoords); }

    void initKdTree(const std::vector<Point3d>& verticesCoords)
    {
        ALICEVISION_LOG_INFO("Build nanoflann KdTree index.");
        _pointCloudRef = std::make_unique<PointVectorAdaptator>(verticesCoords);
        _tree = std::make_unique<KdTree>(3 /*dim*/, *_pointCloudRef.get(), nanoflann::KDTreeSingleIndexAdaptorParams(MAX_LEAF_ELEMENTS));
        _tree->buildIndex();
        ALICEVISION_LOG_INFO("KdTree created for " << verticesCoords.size() << " points.");
    }

    bool locateNearestVertex(const Point3d& p, std::size_t& index, double& sq_dist) const
    {
        index = std::numeric_limits<std::size_t>::max();
        sq_dist = std::numeric_limits<double>::max();

        nanoflann::KNNResultSet<double, std::size_t> resultSet(1);
        resultSet.init(&index, &sq_dist);
        if (!_tree->findNeighbors(resultSet, p.m, nanoflann::SearchParameters()))
        {
            return false;
        }
        return true;
    }
};

}
}