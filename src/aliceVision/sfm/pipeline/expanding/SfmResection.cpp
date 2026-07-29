// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfmResection.hpp"

#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/stl/mapUtils.hpp>

#include <aliceVision/robustEstimation/NACRansac.hpp>
#include <aliceVision/numeric/Container.hpp>
#include <aliceVision/multiview/resection/ResectionSphericalKernel.hpp>
#include <aliceVision/multiview/resection/ResectionNodalSphericalKernel.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>

namespace aliceVision {
namespace sfm {


bool SfmResection::processView(
                        const sfmData::SfMData & sfmData,
                        const track::TracksMap & tracks,
                        const track::TracksPerView & tracksPerView, 
                        std::mt19937 &randomNumberGenerator,
                        const IndexT viewId,
                        Eigen::Matrix4d & updatedPose,
                        double & updatedThreshold,
                        size_t & inliersCount
                        )
{
    if (!_validationPolicy)
    {
        ALICEVISION_LOG_ERROR("Validation policy is not set.");
        return false;
    }
    
    ALICEVISION_LOG_INFO("SfmResection::processView start " << viewId);

    // A. Compute 2D/3D matches
    // A1. list tracks ids used by the view
    const aliceVision::track::TrackIdSet & viewTracksIds = tracksPerView.at(viewId);

    // A2. Each landmark's id is equal to the associated track id
    // Get list of landmarks = get list of reconstructed tracks
    std::set<std::size_t> reconstructedTrackId;
    std::transform(sfmData.getLandmarks().begin(), sfmData.getLandmarks().end(),
                 std::inserter(reconstructedTrackId, reconstructedTrackId.begin()),
                 stl::RetrieveKey());

    // Remove all reconstructed tracks which were not observed in the view to resect.
    // --> Intersection of tracks observed in this view and tracks reconstructed.
    std::set<std::size_t> trackIds;
    std::set_intersection(viewTracksIds.begin(), viewTracksIds.end(),
                        reconstructedTrackId.begin(),
                        reconstructedTrackId.end(),
                        std::inserter(trackIds, trackIds.begin()));

    if (trackIds.size() < 3)
    {
        
        // If less than 3 points, the resection is theoretically impossible.
        // Let ignore this view.
        return false;
    }


    //Get information about this view
    const sfmData::View & view = sfmData.getView(viewId);
    std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicSharedPtr(view.getIntrinsicId());

    // Retrieve the image group And associated information

    bool isNodal = false;
    Vec3 center = Vec3::Zero();
    if (view.getImageGroupId() != UndefinedIndexT)
    { 
        const auto & group = *sfmData.getImageGroups().at(view.getImageGroupId());
        isNodal = group.isNodalCamera();
        center = group.getCenter();
    }

    //Loop over tracks to build data needed by resection process
    std::vector<Eigen::Vector3d> structure;
    std::vector<Eigen::Vector2d> observations;
    std::vector<feature::EImageDescriberType> featureTypes;
    for (const auto & trackId : trackIds)
    {
        const auto & track = tracks.at(trackId);

        const feature::EImageDescriberType descType = track.descType;        
        const Eigen::Vector3d X = sfmData.getLandmarks().at(trackId).getX() - center;
        const Eigen::Vector2d x = track.featPerView.at(viewId).coords;

        structure.push_back(X);
        observations.push_back(x);
        featureTypes.push_back(descType);
    }

    //Compute a first estimation of the pose
    Eigen::Matrix4d pose;
    std::vector<size_t> inliers;
    double errorMax = 0.0;
    if (isNodal)
    {
        if (!internalNodal(intrinsic, randomNumberGenerator, structure, observations, featureTypes, pose, inliers, errorMax))
        {
            ALICEVISION_LOG_INFO("SfmResection::processView internalNodal failed " << viewId);
            return false;
        }
    }
    else
    {
        if (!internalResection(intrinsic, randomNumberGenerator, structure, observations, featureTypes, pose, inliers, errorMax))
        {
            ALICEVISION_LOG_INFO("SfmResection::processView internalResection failed " << viewId);
            return false;
        }
    }
    

    inliersCount = inliers.size();
    ALICEVISION_LOG_INFO("Resection for view " << viewId << " had " << inliersCount << " inliers.");

    //Refine the pose
    if (!internalRefinement(structure, observations, inliers, pose, intrinsic, isNodal, errorMax))
    {
        ALICEVISION_LOG_INFO("SfmResection::processView internalRefinemanet failed " << viewId);
        return false;
    }
    

    updatedThreshold = errorMax;
    updatedPose = pose;

    ALICEVISION_LOG_INFO("SfmResection::processView end " << viewId);

    return true;
}

bool SfmResection::internalResection(
            std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            std::mt19937 &randomNumberGenerator,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<feature::EImageDescriberType> & featureTypes,
            Eigen::Matrix4d & pose,
            std::vector<size_t> & inliers,
            double & errorMax
        )
{
    multiview::resection::ResectionSphericalKernel kernel(intrinsic, structure, observations);

    Eigen::Matrix4d model;

    inliers.clear();
    auto pairResult = robustEstimation::NACRANSAC(kernel, randomNumberGenerator, inliers, _maxIterations, &model, _precision);

    errorMax = pairResult.first;
    const bool resection = _validationPolicy->validate(*intrinsic, structure, observations, featureTypes, model, inliers);

    if (resection)
    {
        pose = model;
        return true;
    }

    return false;
}

bool SfmResection::internalNodal(
            std::shared_ptr<camera::IntrinsicBase> & intrinsic,
            std::mt19937 &randomNumberGenerator,
            const std::vector<Eigen::Vector3d> & structure,
            const std::vector<Eigen::Vector2d> & observations,
            const std::vector<feature::EImageDescriberType> & featureTypes,
            Eigen::Matrix4d & pose,
            std::vector<size_t> & inliers,
            double & errorMax
        )
{
    multiview::resection::ResectionNodalSphericalKernel kernel(intrinsic, structure, observations);

    robustEstimation::Mat3Model model;

    inliers.clear();
    auto pairResult = robustEstimation::NACRANSAC(kernel, randomNumberGenerator, inliers, _maxIterations, &model, _precision);

    Eigen::Matrix4d model4;
    model4.setIdentity();
    model4.topLeftCorner(3, 3) = model.getMatrix();
    
    errorMax = pairResult.first;

    const bool resection = _validationPolicy->validate(*intrinsic, structure, observations, featureTypes, model4, inliers);

    if (resection)
    {
        pose = model4;
        return true;
    }

    return false;
}

bool SfmResection::internalRefinement(
        const std::vector<Eigen::Vector3d> & structure,
        const std::vector<Eigen::Vector2d> & observations,
        const std::vector<size_t> & inliers,
        Eigen::Matrix4d & pose, 
        std::shared_ptr<camera::IntrinsicBase> & intrinsics,
        bool isNodal,
        double & errorMax)
{
    // Setup a tiny SfM scene with the corresponding 2D-3D data
    sfmData::SfMData tinyScene;


    // view
    std::shared_ptr<sfmData::View> view = std::make_shared<sfmData::View>("", 0, 0, 0);
    tinyScene.getViews().insert(std::make_pair(0, view));

    // pose
    tinyScene.setPose(*view, sfmData::CameraPose(geometry::Pose3(pose)));

    // Intrinsics
    tinyScene.getIntrinsics().emplace(0, intrinsics);

    const double unknownScale = 1.0;

    // structure data (2D-3D correspondences)
    for(std::size_t i = 0; i < inliers.size(); ++i)
    {
        const std::size_t idx = inliers[i];

        sfmData::Landmark landmark;
        landmark.setX(structure[idx]);
        landmark.getObservations()[0] = sfmData::Observation(observations[idx], UndefinedIndexT, unknownScale);
        tinyScene.getLandmarks()[i] = std::move(landmark);
    }

    BundleAdjustmentCeres BA;
    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION;
    if (!isNodal)
    {
        refineOptions |= BundleAdjustment::REFINE_CENTER;
    }

    const bool success = BA.adjust(tinyScene, refineOptions);
    if(!success)
    {
        return false;
    }

    pose = tinyScene.getPose(*view).getTransform().getHomogeneous();

    // Compute errorMax
    errorMax = 0.0;
    for(std::size_t i = 0; i < inliers.size(); ++i)
    {
        const std::size_t idx = inliers[i];

        Vec2 est = intrinsics->transformProject(pose, structure[idx].homogeneous(), true);
        Vec2 diff = observations[idx] - est;

        errorMax = std::max(errorMax, diff.norm());
    }

    const bool resection = _validationPolicy->validateRefinement(tinyScene);
    if (!resection)
    {
        return false;
    }

    return true;
}

} // namespace sfm
} // namespace aliceVision
