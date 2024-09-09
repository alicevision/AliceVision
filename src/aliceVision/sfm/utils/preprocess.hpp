// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/Track.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Make sure the landmarks ids are consistent with the track ids
 * @param sfmData the sfmData to update
 * @param tracks the input information about tracks to compare with landmarks
*/
void remapLandmarkIdsToTrackIds(sfmData::SfMData& sfmData, const track::TracksMap & tracks);

}
}