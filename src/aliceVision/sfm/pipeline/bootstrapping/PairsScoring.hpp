// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/sfm/pipeline/relativePoses.hpp>

#include <vector>

namespace aliceVision {
namespace sfm {

/**
* @brief Get best pair with highest score
* @param sfmData the input sfmData which contains camera information
* @param pairs the input list of reconstructed pairs
* @param tracksMap the input map of tracks
* @param tracksPerView tracks grouped by views
* @param filterIn pair must contains one of the views inside this set (if non empty)
* @param filterout pair must NOT contains one of the views inside this set (if non empty)
* @param hardMinAngle minimal angle allowed (rejected)
* @param softMinAngle minimal angle allowed (tolerated, but heavily downgraded)
* @param maxAngle maximal angle allowed
* @return index in "pairs" of the best pair or UndefinedIndexT if no pair found
*/
IndexT findBestPair(const sfmData::SfMData & sfmData, 
                    const std::vector<sfm::ReconstructedPair> & pairs,
                    const track::TracksMap& tracksMap, 
                    const track::TracksPerView & tracksPerView, 
                    const std::set<IndexT> & filterIn,
                    const std::set<IndexT> & filterOut,
                    double hardMinAngle,
                    double softMinAngle,
                    double maxAngle);

/**
* @brief Get best pair from track Depths with highest score
* @param sfmData the input sfmData which contains camera information
* @param pairs the input list of reconstructed pairs
* @param tracksMap the input map of tracks
* @param tracksPerView tracks grouped by views
* @return The best pair (pair.reference is UndefinedIndexT if nothing found)
*/
sfm::ReconstructedPair findBestPairFromTrackDepths(const sfmData::SfMData & sfmData, 
                            const std::vector<sfm::ReconstructedPair> & pairs,
                            const track::TracksMap& tracksMap, 
                            const track::TracksPerView & tracksPerView);

}
}