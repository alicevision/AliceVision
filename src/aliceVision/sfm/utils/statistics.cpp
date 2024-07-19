// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "statistics.hpp"

#include <aliceVision/sfmData/SfMData.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

namespace aliceVision {
namespace sfm {

double RMSE(const sfmData::SfMData& sfmData)
{
    // Compute residuals for each observation
    std::vector<double> vec;
    for (sfmData::Landmarks::const_iterator iterTracks = sfmData.getLandmarks().begin(); iterTracks != sfmData.getLandmarks().end(); ++iterTracks)
    {
        const sfmData::Observations& obs = iterTracks->second.getObservations();
        for (sfmData::Observations::const_iterator itObs = obs.begin(); itObs != obs.end(); ++itObs)
        {
            const sfmData::View* view = sfmData.getViews().find(itObs->first)->second.get();
            const geometry::Pose3 pose = sfmData.getPose(*view).getTransform();
            const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId());
            const Vec2 residual = intrinsic->residual(pose, iterTracks->second.X.homogeneous(), itObs->second.getCoordinates());
            vec.push_back(residual(0));
            vec.push_back(residual(1));
        }
    }
    if (vec.empty())
        return -1.0;
    const Eigen::Map<Eigen::RowVectorXd> residuals(&vec[0], vec.size());
    const double RMSE = std::sqrt(residuals.squaredNorm() / vec.size());
    return RMSE;
}

double computeAreaScore(const std::vector<Eigen::Vector2d>& refPts, const std::vector<Eigen::Vector2d>& nextPts, size_t refWidth, size_t refHeight, size_t nextWidth, size_t nextHeight)
{
    namespace bg = boost::geometry;

    const double refArea = refWidth * refHeight;
    const double nextArea = nextWidth * nextHeight;

    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
    typedef boost::geometry::model::multi_point<point_t> mpoint_t;
    typedef boost::geometry::model::polygon<point_t> polygon;
    mpoint_t mpt1, mpt2;

    for (int idx = 0; idx < refPts.size(); idx++)
    {
        const auto& refPt = refPts[idx];
        const auto& nextPt = nextPts[idx];

        boost::geometry::append(mpt1, point_t(refPt(0), refPt(1)));
        boost::geometry::append(mpt2, point_t(nextPt(0), nextPt(1)));
    }

    polygon hull1, hull2;
    boost::geometry::convex_hull(mpt1, hull1);
    boost::geometry::convex_hull(mpt2, hull2);
    const double area1 = boost::geometry::area(hull1);
    const double area2 = boost::geometry::area(hull1);
    const double score = (area1 + area2) / (refArea + nextArea);

    return score;
}

}  // namespace sfm
}  // namespace aliceVision
