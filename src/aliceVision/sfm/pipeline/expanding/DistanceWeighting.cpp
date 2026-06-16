// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/expanding/DistanceWeighting.hpp>
#include "nanoflann.hpp"

#include <algorithm>
#include <numeric>
#include <cmath>

namespace aliceVision {
namespace sfm {

struct PointCloud2D
{
    std::vector<sfmData::Observation *> pts;

    // ── mandatory nanoflann interface ──────────

    // Total number of points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Value of the d-th coordinate of the i-th point
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        return dim == 0 ? pts[idx]->getX() : pts[idx]->getY();
    }

    // Optional bounding-box computation (return false = let nanoflann compute it)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};

bool weightObservationsFromDistance(sfmData::SfMData & sfmData, size_t neighboorRank, double ratioDefaultArea)
{
    const int K = static_cast<int>(neighboorRank);

    sfmData::Landmarks & landmarks = sfmData.getLandmarks();

    std::map<IndexT, PointCloud2D> perViewObservations;

    // Make list of landmark per view
    for (auto & [_, landmark] : landmarks)
    {
        for (auto & [idView, obs] : landmark.getObservations())
        {
            perViewObservations[idView].pts.push_back(&obs);
        }
    }

    // Loop per view
    for (auto & [idView, pointCloud] : perViewObservations)
    {
        int N = pointCloud.pts.size();

        if (N == 0)
        {
            continue;
        }

        const sfmData::View & v = sfmData.getView(idView);
        const double viewArea = v.getImage().getWidth() * v.getImage().getHeight();


        // average area per feature
        const double featArea = viewArea / N;
        const double minSqDist = 1.0;
        const double maxSqDist = ratioDefaultArea * featArea;

        if (N < 10)
        {
            for (size_t i = 0; i < N; ++i)
            {
                pointCloud.pts[i]->setWeight(1.0);
            }
            continue;
        }

        using KDTree2D = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>;

        KDTree2D index(2, pointCloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index.buildIndex();

        std::vector<double> knnSquaredDistance(N);

        // Find the distance to the Kth nearest neighbor
        #pragma omp parallel
        {
            std::vector<unsigned int> indices(K);
            std::vector<double> sq_dists(K);

            #pragma omp for schedule(static)
            for (size_t i = 0; i < N; ++i)
            {
                const double query[2] = { pointCloud.pts[i]->getX(), pointCloud.pts[i]->getY() };
                index.knnSearch(query, K, indices.data(), sq_dists.data());
                knnSquaredDistance[i] = std::clamp(sq_dists[K - 1], minSqDist, maxSqDist);
            }
        }

        // Normalize so the mean is 1 (overall influence is 1).what is
        const double meanNormalizedDistance = std::accumulate(knnSquaredDistance.begin(), knnSquaredDistance.end(), 0.0) / static_cast<double>(N);

        for (size_t i = 0; i < N; ++i)
        {
            knnSquaredDistance[i] /= meanNormalizedDistance;
            pointCloud.pts[i]->setWeight(knnSquaredDistance[i]);
        }
    }

    return true;
}

}
}
