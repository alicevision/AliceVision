// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>

#include <string>

namespace aliceVision {
namespace matching {


/**
 * @brief Load a match file.
 *
 * @param[out] matches container for the output matches
 * @param[in] filepath the match file to load
 */
bool LoadMatchFile(PairwiseMatches& matches, const std::string& filepath);

/**
 * @brief Load the match file for each image.
 * @param[out] matches container for the output matches.
 * @param[in] viewsKeys the list of views whose match files need to be loaded.
 * @param[in] folder the folder where to look for all the files.
 * @param[in] extension the extension of the match file.
 * @return the number of match file actually loaded (if a file cannot be loaded it is discarded)
 */
std::size_t LoadMatchFilePerImage(PairwiseMatches& matches,
                                  const std::set<IndexT>& viewsKeys,
                                  const std::string& folder,
                                  const std::string& extension);

/**
 * @brief Load all the matches from the folder. Optionally filter the view, the type of descriptors
 * and the number of matches.
 *
 * @param[out] matches container for the output matches.
 * @param[in] viewsKeysFilter Restrict the matches to these views.
 * @param[in] folders The list of folder from where to lead the match files.
 * @param[in] descTypesFilter Restrict the matches to these types of descriptors.
 * @param[in] maxNbMatches keep at most \p maxNbMatches matches (0 takes all matches).
 * @param[in] minNbMatches discard the match files with less than \p minNbMatches (0 takes all files).
 * @return \p false if no file could be loaded.
 * @see filterMatchesByViews
 * @see filterTopMatches
 */
bool Load(PairwiseMatches& matches,
          const std::set<IndexT>& viewsKeysFilter,
          const std::vector<std::string>& folders,
          const std::vector<feature::EImageDescriberType>& descTypesFilter,
          int maxNbMatches = 0,
          int minNbMatches = 0);

/**
 * @brief Filter to keep only specific viewIds.
 * @param[in,out] matches the matches to filter.
 * @param[in] viewsKeys the list of views to keep.
 */
void filterMatchesByViews(PairwiseMatches& matches, const std::set<IndexT>& viewsKeys);

/**
 * @brief  Filter to keep only the \c limitNum first matches per descriptor type.
 * @param[in,out] allMatches he matches to filter.
 * @param[in] maxNum The maximum number of matches to retrieve.
 * @param[in] minNum The minimum number of matches that the file must contain (otherwise it is discarded).
 */
void filterTopMatches(PairwiseMatches& allMatches, int maxNum, int minNum);

/**
 * @brief Save match files.
 *
 * @param[in] matches: container for the output matches
 * @param[in] folder: folder containing the match files
 * @param[in] extension: txt or bin file format
 * @param[in] matchFilePerImage: do we store a global match file
 *            or one match file per image
 * @param[in] prefix: optional prefix for the output file(s)
 */
bool Save(const PairwiseMatches& matches,
          const std::string& folder,
          const std::string& extension,
          bool matchFilePerImage,
          const std::string& prefix = "");

}  // namespace matching
}  // namespace aliceVision
