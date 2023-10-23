// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
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
 * @param[in] folders Path(s) to folder(s) in which computed matches are stored.
 * @param[in] descTypes ImageDescriberTypes used
 * @param[in] maxNbMatches Maximum number of matches per image pair (and per feature type), 0 = no limit
 * @param[in] useOnlyMatchesFromFolder If enabled, don't use sfmData matches folders
 */
inline bool loadPairwiseMatches(matching::PairwiseMatches& out_pairwiseMatches,
                                const sfmData::SfMData& sfmData,
                                const std::vector<std::string>& folders,
                                const std::vector<feature::EImageDescriberType>& descTypes,
                                const int maxNbMatches = 0,
                                const int minNbMatches = 0,
                                bool useOnlyMatchesFromFolder = false)
{
    std::vector<std::string> matchesFolders;

    ALICEVISION_LOG_DEBUG("List of provided match folders:");
    for (auto it = folders.begin(); it != folders.end(); ++it)
        ALICEVISION_LOG_DEBUG("\t - " << *it);

    if (!useOnlyMatchesFromFolder)
        matchesFolders = sfmData.getMatchesFolders();
    else
        ALICEVISION_LOG_DEBUG("Load only matches from given folder.");

    matchesFolders.insert(matchesFolders.end(), folders.begin(), folders.end());

    ALICEVISION_LOG_DEBUG("List of match folders to load:");
    for (auto it = matchesFolders.begin(); it != matchesFolders.end(); ++it)
        ALICEVISION_LOG_DEBUG("\t - " << *it);

    ALICEVISION_LOG_DEBUG("Loading matches");
    if (!matching::Load(out_pairwiseMatches, sfmData.getViewsKeys(), matchesFolders, descTypes, maxNbMatches, minNbMatches))
    {
        std::stringstream ss("Unable to read the matches file(s) from:\n");
        for (const std::string& folder : matchesFolders)
            ss << "\t- " << folder << "\n";
        ALICEVISION_LOG_WARNING(ss.str());
        return false;
    }
    return true;
}

}  // namespace sfm
}  // namespace aliceVision
