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
 * @brief estimate a median angle (parallax) between a reference view and another view
 * @param sfmData the input sfmData which contains camera information
 * @param referenceViewId the reference view id
 * @param otherViewId the other view id
 * @param otherTreference the relative pose
 * @param errorMax the maximal tolerated error
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views
 * @param resultAngle the output median angle
 * @param usedTracks the list of tracks which were successfully reconstructed
 * @return true
*/
bool estimatePairAngle(const sfmData::SfMData & sfmData, 
                       const IndexT referenceViewId,
                       const IndexT otherViewId,
                       const geometry::Pose3 & otherTreference,
                       const double & errorMax,
                       const track::TracksMap& tracksMap, 
                       const track::TracksPerView & tracksPerView, 
                       double & resultAngle, 
                       std::vector<size_t> & usedTracks);

}
}