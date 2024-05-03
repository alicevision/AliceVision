// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmTriangulation.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>
#include <aliceVision/multiview/triangulation/Triangulation.hpp>
#include <aliceVision/multiview/triangulation/TriangulationSphericalKernel.hpp>

#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>

#include <aliceVision/camera/camera.hpp>

namespace aliceVision {
namespace sfm {




bool SfmTriangulation::process(
            const sfmData::SfMData & sfmData,
            const track::TracksMap & tracks,
            const track::TracksPerView & tracksPerView, 
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> &viewIds,
            std::set<IndexT> & evaluatedTracks,
            std::map<IndexT, sfmData::Landmark> & outputLandmarks
        )
{
    evaluatedTracks.clear();
    outputLandmarks.clear();

    // Get all tracks id which are visible in views
    std::set<IndexT> viewTracks;
    track::getTracksInImagesFast(viewIds, tracksPerView, viewTracks);

    std::vector<IndexT> viewTracksVector;
    std::copy(viewTracks.begin(), viewTracks.end(), std::back_inserter(viewTracksVector));

    const std::set<IndexT>& validViews = sfmData.getValidViews();

    // Get a set of all views to consider
    std::set<IndexT> allInterestingViews;
    allInterestingViews.insert(viewIds.begin(), viewIds.end());
    allInterestingViews.insert(validViews.begin(), validViews.end());


    #pragma omp parallel for
    for(int pos = 0; pos < viewTracksVector.size(); pos++)
    {
        const std::size_t trackId = viewTracksVector[pos];
        const track::Track& track = tracks.at(trackId);

        // Get all views observing the current track (Keeping their Id)
        std::set<IndexT> trackViews;
        std::transform(track.featPerView.begin(), track.featPerView.end(),
                       std::inserter(trackViews, trackViews.begin()), stl::RetrieveKey());

        // Intersect with list of interesting views
        std::set<IndexT> trackViewsFiltered;
        std::set_intersection(trackViews.begin(), trackViews.end(), 
                            allInterestingViews.begin(), allInterestingViews.end(), 
                            std::inserter(trackViewsFiltered, trackViewsFiltered.begin()));
    
        if(trackViewsFiltered.size() < _minObservations)
        {
            continue;
        }

        #pragma omp critical
        {
            
            evaluatedTracks.insert(trackId);
        }

        sfmData::Landmark result;
        if (!processTrack(sfmData, track, randomNumberGenerator, trackViewsFiltered, result))
        {
            continue;
        }

        #pragma omp critical
        {
            outputLandmarks[trackId] = result;
        }
    }

    return true;
}

bool SfmTriangulation::processTrack(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        )
{
    feature::EImageDescriberType descType = track.descType;

    std::vector<Vec2> observations;
    std::vector<std::shared_ptr<camera::IntrinsicBase>> intrinsics;
    std::vector<Eigen::Matrix4d> poses;
    std::vector<IndexT> indexedViewIds;

    for (auto viewId : viewIds)
    {   
        //Retrieve pose and feature coordinates for this observation
        const sfmData::View & view = sfmData.getView(viewId);
        const std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicSharedPtr(view.getIntrinsicId());
        const Eigen::Matrix4d pose = sfmData.getPose(view).getTransform().getHomogeneous();

        const auto  & trackItem = track.featPerView.at(viewId);

        //Lift the coordinates to metric unit sphere
        const Vec2 coords = trackItem.coords;

        observations.push_back(coords);
        intrinsics.push_back(intrinsic);
        poses.push_back(pose);
        
        indexedViewIds.push_back(viewId);
    }



    robustEstimation::MatrixModel<Vec4> model;
    std::vector<std::size_t> inliers;
    robustEstimation::ScoreEvaluator<multiview::TriangulationSphericalKernel> scorer(_maxError);
    multiview::TriangulationSphericalKernel kernel(observations, poses, intrinsics);

    if (observations.size() <= 0)
    {
    }
    else 
    {
        model = robustEstimation::LO_RANSAC(kernel, scorer, randomNumberGenerator, &inliers);
    }
    
    Vec4 X = model.getMatrix();

    Vec3 X_euclidean;
    homogeneousToEuclidean(X, X_euclidean); 

    //Create landmark from result
    result.X = X_euclidean;
    result.descType = track.descType;

    for (const std::size_t & i : inliers)
    {   
        //Inlier to view index
        IndexT viewId = indexedViewIds[i];
        
        sfmData::Observation & o = result.getObservations()[viewId];

        //Retrieve observation data
        const auto  & trackItem = track.featPerView.at(viewId);

        o.setFeatureId(trackItem.featureId);
        o.setScale(trackItem.scale);
        o.setCoordinates(trackItem.coords);
    }

    return true;
}

bool SfmTriangulation::checkChierality(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark)
{
    for (const auto & pRefObs : landmark.getObservations())
    {
        IndexT refViewId = pRefObs.first;
        const sfmData::View & refView = sfmData.getView(refViewId);
        const sfmData::Observation & obs = pRefObs.second;
        const camera::IntrinsicBase * intrinsic = sfmData.getIntrinsicPtr(refView.getIntrinsicId());
        const sfmData::CameraPose refCameraPose = sfmData.getPose(refView);
        const geometry::Pose3 & refPose = refCameraPose.getTransform();

        const Vec3 dir = intrinsic->toUnitSphere(intrinsic->removeDistortion(intrinsic->ima2cam(obs.getCoordinates())));
        const Vec3 ldir = refPose(landmark.X).normalized();

        if (dir.dot(ldir) < 0.0)
        {
            return false;
        }
    }

    return true;
}

double SfmTriangulation::getMaximalAngle(const sfmData::SfMData & sfmData, const sfmData::Landmark & landmark)
{
    double max = 0.0;

    for (const auto & pRefObs : landmark.getObservations())
    {
        IndexT refViewId = pRefObs.first;

        const sfmData::View & refView = sfmData.getView(refViewId);
        const sfmData::CameraPose & refCameraPose = sfmData.getPoses().at(refView.getPoseId());
        const geometry::Pose3 & refPose = refCameraPose.getTransform();

        for (const auto & pNextObs : landmark.getObservations())
        {
            IndexT nextViewId = pNextObs.first;
            if (refViewId > nextViewId)
            {
                continue;
            }

            const sfmData::View & nextView = sfmData.getView(nextViewId);
            const sfmData::CameraPose & nextCameraPose = sfmData.getPoses().at(nextView.getPoseId());
            const geometry::Pose3 & nextPose = nextCameraPose.getTransform();
            double angle_deg = camera::angleBetweenRays(refPose, nextPose, landmark.X);

            max = std::max(max, angle_deg);
        }
    }

    return max;
}

} // namespace sfm
} // namespace aliceVision