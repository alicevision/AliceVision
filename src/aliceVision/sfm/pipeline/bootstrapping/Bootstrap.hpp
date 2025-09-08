// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/tracksUtils.hpp>

#include <vector>

namespace aliceVision {
namespace sfm {

/**
 * @brief Create a minimal SfmData with poses and landmarks for two views
 * @param sfmData the input sfmData which contains camera information
 * @param referenceViewId the reference view id
 * @param otherViewId the other view id
 * @param otherTreference the relative pose
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views
 * @return true
*/
bool bootstrapBase(sfmData::SfMData & sfmData, 
                    const IndexT referenceViewId,
                    const IndexT otherViewId,
                    const geometry::Pose3 & otherTreference,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView);

/**
 * @brief Create a minimal SfmData with poses and landmarks for two views (given a mesh)
 * @param sfmData the input sfmData which contains camera information
 * @param landmarks the input Landmarks file which contains mesh geometry
 * @param referenceViewId the reference view id
 * @param nextViewId the reference view id
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views
 * @return true
*/
bool bootstrapMesh(sfmData::SfMData & sfmData, 
                    const sfmData::Landmarks & landmarks,
                    const IndexT referenceViewId,
                    const IndexT nextViewId,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView);


/**
 * @brief Create a minimal SfmData with poses and landmarks for two views
 * @param sfmData the input sfmData which contains camera information
 * @param referenceViewId the reference view id
 * @param otherViewId the other view id
 * @param otherTreference the relative pose
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views
 * @return true
*/
bool bootstrapDepth(sfmData::SfMData & sfmData, 
                    const IndexT referenceViewId,
                    const IndexT otherViewId,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView);
}
}