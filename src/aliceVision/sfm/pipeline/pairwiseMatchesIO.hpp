// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>

#include <vector>
#include <string>

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
    const int maxNbMatches = 0)
{
  std::vector<std::string> matchesFolders = sfmData.getMatchesFolders();
  matchesFolders.emplace_back(folder);

  ALICEVISION_LOG_DEBUG("Loading matches");
  if (!matching::Load(out_pairwiseMatches, sfmData.GetViewsKeys(), matchesFolders, descTypes, matchesMode, maxNbMatches))
  {
    std::stringstream ss("Unable to read the matches file(s) from:\n");
    for(const std::string& folder : matchesFolders)
      ss << "\t- " << folder << "\n";
    ss << "(mode: " << matchesMode << ")\n";
    ALICEVISION_LOG_WARNING(ss.str());
    return false;
  }
  return true;
}

} // namespace sfm
} // namespace aliceVision
