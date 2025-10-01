// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/bootstrapping/EstimateAngle.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

#include <algorithm>

namespace aliceVision {
namespace sfm {

bool estimatePairAngle(const sfmData::SfMData & sfmData, 
                       const IndexT referenceViewId,
                       const IndexT otherViewId,
                       const geometry::Pose3 & otherTreference,
                       const double & errorMax,
                       const track::TracksMap& tracksMap, 
                       const track::TracksPerView & tracksPerView, 
                       double & resultAngle, 
                       std::vector<size_t> & usedTracks)
{
    usedTracks.clear();

    const sfmData::View& refView = sfmData.getView(referenceViewId);
    const sfmData::View& nextView = sfmData.getView(otherViewId);

    std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
    std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
    
    aliceVision::track::TracksMap mapTracksCommon;
    track::getCommonTracksInImagesFast({referenceViewId, otherViewId}, tracksMap, tracksPerView, mapTracksCommon);

    const Mat4 T1 = Eigen::Matrix4d::Identity();
    Mat4 T2 = otherTreference.getHomogeneous();
    const Mat3 R2 = otherTreference.rotation();
    const Mat3 E = CrossProductMatrix(otherTreference.translation()) * R2;
    
    const Eigen::Vector3d c = otherTreference.center();

    size_t count = 0;
    std::vector<double> angles;
    for(const auto& commonItem : mapTracksCommon)
    {
        const track::Track& track = commonItem.second;
        
        const IndexT refFeatureId = track.featPerView.at(referenceViewId).featureId;
        const IndexT nextfeatureId = track.featPerView.at(otherViewId).featureId;

        const Vec2 refpt = track.featPerView.at(referenceViewId).coords;
        const Vec2 nextpt = track.featPerView.at(otherViewId).coords;
        
        const Vec3 pt3d1 = refIntrinsics->toUnitSphere(refIntrinsics->removeDistortion(refIntrinsics->ima2cam(refpt)));
        const Vec3 pt3d2 = nextIntrinsics->toUnitSphere(nextIntrinsics->removeDistortion(nextIntrinsics->ima2cam(nextpt)));

        const Vec3 x = (E * pt3d1).normalized();
        double error = std::abs(std::asin(pt3d2.dot(x)));
        if (error > errorMax)
        {
            continue;
        }

        
        Vec3 X;
        multiview::TriangulateSphericalDLT(T1, pt3d1, T2, pt3d2, X);
        
        //Make sure 
        Eigen::Vector3d dirX1 = (T1 * X.homogeneous()).head(3).normalized();
        Eigen::Vector3d dirX2 = (T2 * X.homogeneous()).head(3).normalized();
        if (!(dirX1.dot(pt3d1) > 0.0 && dirX2.dot(pt3d2)  > 0.0))
        {
            continue;
        }

        const Vec3 ray1 = - X;
        const Vec3 ray2 = c - X;
        const double cangle = clamp(ray1.normalized().dot(ray2.normalized()), -1.0, 1.0);
        const double angle = std::acos(cangle);
        angles.push_back(angle);

        usedTracks.push_back(commonItem.first);
    }

    if (angles.size() == 0)
    {
        resultAngle = 0.0;
        return false;
    }

    const unsigned medianIndex = angles.size() / 2;
    std::nth_element(angles.begin(), angles.begin() + medianIndex, angles.end());
    resultAngle = angles[medianIndex];
    return true;
}

}
}