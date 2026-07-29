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

#include <aliceVision/numeric/projection.hpp>
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
            std::map<IndexT, sfmData::Landmark> & outputLandmarks,
            bool useDepthPrior
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
    
        if(trackViewsFiltered.size() < _minObservations && !useDepthPrior)
        {
            continue;
        }

        #pragma omp critical
        {
            evaluatedTracks.insert(trackId);
        }

        sfmData::Landmark result;
        if (useDepthPrior)
        {
            if (_pointFetcherHandler)
            {
                if (!processTrackWithPointFetcher(sfmData, track, randomNumberGenerator, trackViewsFiltered, result))
                {
                    continue;
                }
            }
            else 
            {
                if (!processTrackWithPrior(sfmData, track, randomNumberGenerator, trackViewsFiltered, result))
                {
                    continue;
                }
            }
        }
        else 
        {
            if (!processTrack(sfmData, track, randomNumberGenerator, trackViewsFiltered, result))
            {
                continue;
            }
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

    std::vector<double> weights;
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
        weights.push_back(1.0);
        //TODO, check how to use scale correctly
        //weights.push_back(1.0 / trackItem.scale);
        
        indexedViewIds.push_back(viewId);
    }



    robustEstimation::MatrixModel<Vec4> model;
    std::vector<std::size_t> inliers;
    robustEstimation::ScoreEvaluator<multiview::TriangulationSphericalKernel> scorer(_maxError);
    multiview::TriangulationSphericalKernel kernel(observations, weights, poses, intrinsics);

    if (observations.size() <= 0)
    {
        return false;
    }
    else 
    {
        model = robustEstimation::LO_RANSAC(kernel, scorer, randomNumberGenerator, &inliers);
    }
    
    Vec4 X = model.getMatrix();

    Vec3 X_euclidean;
    homogeneousToEuclidean(X, X_euclidean); 

    //Create landmark from result
    result.setX(X_euclidean);
    result.setDescType(track.descType);

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

bool SfmTriangulation::processTrackWithPrior(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        )
{
    size_t bestInliersCount = 0;

    //For each observed view in the track
    for (auto referenceViewId : viewIds)
    {   
        if (track.featPerView.find(referenceViewId) == track.featPerView.end())
        {
            continue;
        }

        //Look if this observation has an associated depth
        const auto & refTrackItem = track.featPerView.at(referenceViewId);
        if (refTrackItem.depth < 0.0)
        {
            continue;
        }

        //Retrieve pose and feature coordinates for this observation
        const sfmData::View & rView = sfmData.getView(referenceViewId);
        const camera::IntrinsicBase & rIntrinsic = sfmData.getIntrinsic(rView.getIntrinsicId());
        const geometry::Pose3 rPose = sfmData.getPose(rView).getTransform();        

        //Compute 3D point in camera space
        const double Z = refTrackItem.depth;
        const Vec2 meters = rIntrinsic.removeDistortion(rIntrinsic.ima2cam(refTrackItem.coords.cast<double>()));
        Vec3 cX = Z * meters.homogeneous();

        //Transform 3D point in world space
        const Vec3 oX = rPose.inverse()(cX);
        
        //Make sure this point is not dependent on parallax 
        //As it does not need parallax to estimate its depth
        sfmData::Landmark landmark;
        landmark.setParallaxRobust(true);
        landmark.setX(oX);
        landmark.setDescType(track.descType);

        //Compute consensus for this depth
        for (auto viewId : viewIds)
        {
            if (track.featPerView.find(viewId) == track.featPerView.end())
            {
                continue;
            }

            const auto & trackItem = track.featPerView.at(viewId);

            const sfmData::View & view = sfmData.getView(viewId);
            const camera::IntrinsicBase & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();    

            const Vec2 est = intrinsic.transformProject(pose, oX.homogeneous(), true);
            const Vec2 mes = trackItem.coords.cast<double>();
            double err = (est - mes).norm() / trackItem.scale;

            //Use _maxError as threshold for inliers/outlier detection
            if (err > _maxError)
            {
                continue;
            }

            //Create and associated an observation to the landmark
            sfmData::Observation & o = landmark.getObservations()[viewId];
            o.setFeatureId(trackItem.featureId);
            o.setScale(trackItem.scale);
            o.setCoordinates(trackItem.coords);
            o.setDepth(trackItem.depth);
        }

        //Store the landmark if it's better than the previously estimated one
        int count = landmark.getObservations().size();
        if (count > bestInliersCount)
        {
            bestInliersCount = count;
            result = landmark;
        }
    }

    //One inlier is the reference, so we need at least 2 inliers
    if (bestInliersCount < 2)
    {
        return false;
    }

    return true;
}

bool SfmTriangulation::processTrackWithPointFetcher(
            const sfmData::SfMData & sfmData,
            const track::Track & track,
            std::mt19937 &randomNumberGenerator,
            const std::set<IndexT> & viewIds,
            sfmData::Landmark & result
        )
{
    size_t bestInliersCount = 0;
    std::map<IndexT, std::pair<Vec3, Vec3>> possibleParameters;

    //For each observed view in the track
    for (auto referenceViewId : viewIds)
    {   
        if (track.featPerView.find(referenceViewId) == track.featPerView.end())
        {
            continue;
        }

        //Look if this observation has an associated depth
        const auto & refTrackItem = track.featPerView.at(referenceViewId);

        const sfmData::View & v = sfmData.getView(referenceViewId);
        const sfmData::CameraPose cp = sfmData.getPose(v);
        const camera::IntrinsicBase & intrinsics = *sfmData.getIntrinsics().at(v.getIntrinsicId());

        _pointFetcherHandler->setPose(cp.getTransform());

        Vec3 point, normal;
        if (!_pointFetcherHandler->pickPointAndNormal(point, normal, intrinsics, refTrackItem.coords))
        {
            continue;
        }

        possibleParameters[referenceViewId] = std::make_pair(point, normal);
    }
    
    //Consider each point
    for (const auto & [refViewId, pair] : possibleParameters)
    {
        const Vec3 & refpt = pair.first;

        //Make sure this point is not dependent on parallax 
        //As it does not need parallax to estimate its depth
        sfmData::Landmark landmark;
        landmark.setParallaxRobust(true);
        landmark.setX(refpt);
        landmark.setDescType(track.descType);
        landmark.setReferenceView(sfmData.getViews().at(refViewId));
        landmark.setPointFetcher(_pointFetcherHandler);

        //For each observed view in the track
        for (auto viewId: viewIds)
        {   
            if (track.featPerView.find(viewId) == track.featPerView.end())
            {
                continue;
            }

            const auto & trackItem = track.featPerView.at(viewId);

            const sfmData::View & view = sfmData.getView(viewId);
            const camera::IntrinsicBase & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();    

            const Vec2 est = intrinsic.transformProject(pose, refpt.homogeneous(), true);
            const Vec2 mes = trackItem.coords.cast<double>();
            double err = (est - mes).norm() / trackItem.scale;

            //Use _maxError as threshold for inliers/outlier detection
            if (err > _maxError)
            {
                continue;
            }

            //Create and associated an observation to the landmark
            sfmData::Observation & o = landmark.getObservations()[viewId];
            o.setFeatureId(trackItem.featureId);
            o.setScale(trackItem.scale);
            o.setCoordinates(trackItem.coords);
            o.setDepth(trackItem.depth);
        }

        //Store the landmark if it's better than the previously estimated one
        int count = landmark.getObservations().size();
        if (count > bestInliersCount)
        {
            bestInliersCount = count;
            result = landmark;
        }
    }


    if (bestInliersCount == 0)
    {
        return false;
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
        const Vec3 ldir = refPose(landmark.getX()).normalized();

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
        const sfmData::CameraPose refCameraPose = sfmData.getPose(refView);
        const geometry::Pose3 & refPose = refCameraPose.getTransform();

        for (const auto & pNextObs : landmark.getObservations())
        {
            IndexT nextViewId = pNextObs.first;
            if (refViewId > nextViewId)
            {
                continue;
            }

            const sfmData::View & nextView = sfmData.getView(nextViewId);
            const sfmData::CameraPose nextCameraPose = sfmData.getPose(nextView);
            const geometry::Pose3 & nextPose = nextCameraPose.getTransform();
            double angle_deg = camera::angleBetweenRays(refPose, nextPose, landmark.getX());

            max = std::max(max, angle_deg);
        }
    }

    return max;
}

} // namespace sfm
} // namespace aliceVision