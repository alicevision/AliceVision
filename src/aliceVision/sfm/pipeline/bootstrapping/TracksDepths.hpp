// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <aliceVision/track/Track.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Create a new sfmData cleared from any existing landmarks and poses
 * But with new landmarks created from tracks if depth exists
 * @param output the result sfmData
 * @param sfmData the input sfmData, used for views and intrinsics
 * @param tracksMap the input map of tracks
 * @param tracksPerView tracks grouped by views 
 * @param viewId the view of interest identifier
 * @return false if an error occured
*/
bool buildSfmDataFromDepthMap(sfmData::SfMData & output,
                            const sfmData::SfMData & sfmData, 
                            const track::TracksMap& tracksMap, 
                            const track::TracksPerView & tracksPerView, 
                            IndexT viewId);
}
}