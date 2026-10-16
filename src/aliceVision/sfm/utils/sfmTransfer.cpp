// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "sfmTransfer.hpp"

#include <aliceVision/sfmData/Rig.hpp>
#include <aliceVision/system/Logger.hpp>

#include <map>
#include <memory>
#include <set>

namespace aliceVision {
namespace sfm {

void transferMatchingPoses(sfmData::SfMData& sfmData,
                           const sfmData::SfMData& sfmDataRef,
                           const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Poses");

    // Destination rig pose ID -> source rig pose ID, to detect inconsistent frame matching
    std::map<IndexT, IndexT> transferredRigPoses;

    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        if (!sfmDataRef.isPoseAndIntrinsicDefined(sourceViewId))
        {
            continue;
        }

        sfmData::View& destinationView = sfmData.getView(destinationViewId);
        const sfmData::View& sourceView = sfmDataRef.getView(sourceViewId);
        destinationView.setResectionId(sourceView.getResectionId());

        const bool sourceInRig = sourceView.isPartOfRig() && !sourceView.isPoseIndependant();
        const bool destinationRigExists = destinationView.isPartOfRig() && sfmData.getRigs().count(destinationView.getRigId()) > 0;

        if (sourceInRig && destinationRigExists)
        {
            // Attach the destination view to its rig frame pose
            const IndexT rigPoseId = sfmData::getRigPoseId(destinationView.getRigId(), destinationView.getFrameId());
            destinationView.setPoseId(rigPoseId);
            destinationView.setIndependantPose(false);

            // Copy the rig (frame) pose
            const auto [it, inserted] = transferredRigPoses.emplace(rigPoseId, sourceView.getPoseId());
            if (!inserted && it->second != sourceView.getPoseId())
            {
                ALICEVISION_LOG_WARNING("Rig pose " << rigPoseId << " receives poses from different reference frames ("
                                                    << it->second << " and " << sourceView.getPoseId() << "), keeping the last one. "
                                                    << "View: " << destinationView.getImage().getImagePath());
            }
            // Track the source of the pose currently assigned
            it->second = sourceView.getPoseId();
            sfmData.getPoses().assign(rigPoseId, sfmDataRef.getAbsolutePose(sourceView.getPoseId()));

            // Copy the sub-pose corresponding to this view only
            auto& subPoses = sfmData.getRigs().at(destinationView.getRigId()).getSubPoses();
            if (destinationView.getSubPoseId() >= subPoses.size())
            {
                subPoses.resize(destinationView.getSubPoseId() + 1);
            }
            subPoses[destinationView.getSubPoseId()] = sfmDataRef.getRigs().at(sourceView.getRigId()).getSubPose(sourceView.getSubPoseId());
            continue;
        }

        // Independent pose: the destination view gets its own pose, even if part of a rig
        if (destinationView.isPartOfRig() && !destinationView.isPoseIndependant())
        {
            destinationView.setPoseId(destinationView.getViewId());
            destinationView.setIndependantPose(true);
        }

        // getPose() composes rig pose and sub-pose if the source view is rig-dependent
        sfmData.setPose(destinationView, sfmDataRef.getPose(sourceView));
    }
}

void transferMatchingIntrinsics(sfmData::SfMData& sfmData,
                                const sfmData::SfMData& sfmDataRef,
                                const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Intrinsics");

    auto& intrinsics = sfmData.getIntrinsics();

    // (original destination intrinsic ID, reference intrinsic ID) -> destination intrinsic ID now holding the copy
    std::map<std::pair<IndexT, IndexT>, IndexT> copies;
    // Original destination intrinsics already overwritten by a copy
    std::set<IndexT> overwritten;

    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        if (!sfmDataRef.isPoseAndIntrinsicDefined(sourceViewId))
        {
            continue;
        }

        sfmData::View& destinationView = sfmData.getView(destinationViewId);
        const IndexT sourceIntrinsicId = sfmDataRef.getView(sourceViewId).getIntrinsicId();
        const IndexT destinationIntrinsicId = destinationView.getIntrinsicId();

        auto [it, isNewPair] = copies.emplace(std::make_pair(destinationIntrinsicId, sourceIntrinsicId), UndefinedIndexT);
        if (isNewPair)
        {
            // Overwrite the destination intrinsic the first time, otherwise create a new one
            if (intrinsics.count(destinationIntrinsicId) && !overwritten.count(destinationIntrinsicId))
            {
                it->second = destinationIntrinsicId;
                overwritten.insert(destinationIntrinsicId);
            }
            else
            {
                it->second = intrinsics.empty() ? 0 : intrinsics.rbegin()->first + 1;
            }

            // Clone instead of assign, as the camera model may differ
            intrinsics.insert_or_assign(it->second, std::shared_ptr<camera::IntrinsicBase>(sfmDataRef.getIntrinsics().at(sourceIntrinsicId)->clone()));
        }

        destinationView.setIntrinsicId(it->second);
    }
}

void transferMatchingLandmarks(sfmData::SfMData& sfmData,
                               const sfmData::SfMData& sfmDataRef,
                               const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Landmarks");

    const auto& refLandmarks = sfmDataRef.getLandmarks();
    if (refLandmarks.empty())
    {
        return;
    }

    // Map reference view IDs to destination view IDs.
    std::map<IndexT, IndexT> commonViewsMap;
    for (const auto& viewPair : commonViewIds)
    {
        commonViewsMap.emplace(viewPair.second, viewPair.first);
    }

    // Copy landmarks from the reference SfMData to the destination SfMData, 
    // only keeping observations that have matching views.

    sfmData::Landmarks newLandmarks;
    for (const auto& landIt : refLandmarks)
    {
        sfmData::Landmark newLandmark = landIt.second;
        newLandmark.getObservations().clear();

        for (const auto& obsIt : landIt.second.getObservations())
        {
            const auto matchingView = commonViewsMap.find(obsIt.first);
            if (matchingView != commonViewsMap.end())
            {
                newLandmark.getObservations().emplace(matchingView->second, obsIt.second);
            }
        }

        if (!newLandmark.getObservations().empty())
        {
            newLandmarks.emplace(landIt.first, newLandmark);
        }
    }

    sfmData.getLandmarks() = newLandmarks;
}

void transferMatchingSurveyPoints(sfmData::SfMData& sfmData,
                                  const sfmData::SfMData& sfmDataRef,
                                  const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Survey Points");

    const auto& refSurveyPoints = sfmDataRef.getSurveyPoints();

    // Survey points are stored per view: copy them from each reference view onto its matching destination view
    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        const auto it = refSurveyPoints.find(sourceViewId);
        if (it != refSurveyPoints.end())
        {
            sfmData.getSurveyPoints()[destinationViewId] = it->second;
        }
    }
}

}  // namespace sfm
}  // namespace aliceVision
