// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Load match files.
 *
 * @param[out] out_pairwiseMatches
 * @param[in] sfmData
 * @param[in] folder
 * @param[in] matchesMode
 */
inline bool loadPairwiseMatches(
    matching::PairwiseMatches& out_pairwiseMatches,
    const SfMData& sfmData,
    const std::string& folder,
    const std::vector<feature::EImageDescriberType>& descTypes,
    const std::string& matchesMode,
    const int maxNbMatches = -1)
{
  ALICEVISION_LOG_DEBUG("- Loading matches...");
  if (!matching::Load(out_pairwiseMatches, sfmData.GetViewsKeys(), folder, descTypes, matchesMode, maxNbMatches))
  {
    ALICEVISION_LOG_WARNING("Unable to read the matches file(s) from: " << folder << " (mode: " << matchesMode << ")");
    return false;
  }
  return true;
}

} // namespace sfm
} // namespace aliceVision
