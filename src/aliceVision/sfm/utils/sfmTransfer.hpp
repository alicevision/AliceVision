// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <utility>
#include <vector>

namespace aliceVision {
namespace sfm {

/**
 * @brief Transfer the poses of the reference views onto their matching destination views.
 *        If both views are rig-dependent, the rig structure is kept: the rig (frame) pose and the
 *        sub-pose of the source view are copied into the destination rig.
 *        Otherwise, the absolute pose of the source view is copied as an independent pose.
 *
 *        Pairs whose reference view has no valid pose and intrinsic are skipped.
 *        The resection ID of the reference view is copied onto the destination view.
 *
 *        Rig case (reference view rig-dependent and destination rig existing):
 *        - the destination view is attached to the rig pose getRigPoseId(rigId, frameId), and made rig-dependent;
 *        - this rig pose is overwritten with the reference rig (frame) pose. If several reference frames
 *          target the same destination frame, a warning is emitted and the last one is kept;
 *        - only the sub-pose of this destination view is overwritten (sub-poses are resized if needed),
 *          the other sub-poses of the destination rig are kept.
 *
 *        Independent case: the reference absolute pose (rig pose composed with sub-pose if rig-dependent)
 *        is written. A rig-dependent destination view is detached from its rig: its pose ID becomes its
 *        view ID. Any existing pose with this ID is overwritten.
 *
 *        Destination poses no longer used are removed by the final cleanup.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingPoses(sfmData::SfMData& sfmData,
                           const sfmData::SfMData& sfmDataRef,
                           const std::vector<std::pair<IndexT, IndexT>>& commonViewIds);

/**
 * @brief Transfer the intrinsics of the reference views onto their matching destination views.
 *
 *        For each destination intrinsic, the first reference intrinsic that matches it is copied into it.
 *        If views of this destination intrinsic match another reference intrinsic, they are moved
 *        onto a new destination intrinsic holding a copy of it.
 *        Destination intrinsics are never merged, even if they match the same reference intrinsic.
 *
 *        Pairs whose reference view has no valid pose and intrinsic are skipped.
 *        As the first copy overwrites the destination intrinsic in place, unmatched destination views
 *        using this intrinsic also get the reference values.
 *        A destination view whose intrinsic ID does not exist gets a new intrinsic.
 *        New intrinsic IDs are allocated after the highest existing ID.
 *        Intrinsics are cloned, so the camera model of the destination may change.
 *
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingIntrinsics(sfmData::SfMData& sfmData,
                                const sfmData::SfMData& sfmDataRef,
                                const std::vector<std::pair<IndexT, IndexT>>& commonViewIds);

/**
 * @brief Replace the destination landmarks by the reference landmarks seen by matching views.
 *        If the reference SfMData has no landmarks, the destination landmarks are kept untouched.
 *        Otherwise, all destination landmarks are discarded. Each reference landmark is copied with its ID,
 *        its observations being re-indexed onto the destination view IDs. Observations in unmatched reference
 *        views are dropped, as well as landmarks without any remaining observation.
 *        The reference views are not required to be reconstructed.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingLandmarks(sfmData::SfMData& sfmData,
                               const sfmData::SfMData& sfmDataRef,
                               const std::vector<std::pair<IndexT, IndexT>>& commonViewIds);

/**
 * @brief Transfer the survey points of the reference views onto their matching destination views.
 *        Survey points are stored per view: for each matching pair where the reference view has survey points,
 *        they replace the survey points of the destination view. Other destination views keep their survey points.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingSurveyPoints(sfmData::SfMData& sfmData,
                                  const sfmData::SfMData& sfmDataRef,
                                  const std::vector<std::pair<IndexT, IndexT>>& commonViewIds);

}  // namespace sfm
}  // namespace aliceVision
