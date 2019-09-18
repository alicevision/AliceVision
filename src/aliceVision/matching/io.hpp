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
 * @param[out] matches: container for the output matches
 * @param[in] folder: folder containing the match files
 */
bool LoadMatchFile(
  PairwiseMatches& matches,
  const std::string& folder,
  const std::string& filename);

/**
 * @brief Load the match file for each image.
 *
 * @param[out] matches: container for the output matches
 * @param[in] folder: folder containing the match files
 */
bool LoadMatchFilePerImage(
  PairwiseMatches& matches,
  const std::set<IndexT>& viewsKeys,
  const std::string& folder);

/**
 * @brief Load match files.
 *
 * @param[out] matches: container for the output matches
 * @param[in] sfm_data
 * @param[in] folder: folder containing the match files
 * @param[in] descTypes
 * @param[in] maxNbMatches: to load the N first matches for each desc. type. Load all the matches by default (: 0)
 */
bool Load(PairwiseMatches& matches,
  const std::set<IndexT>& viewsKeysFilter,
  const std::vector<std::string>& folders,
  const std::vector<feature::EImageDescriberType>& descTypesFilter,
  const int maxNbMatches = 0);

/**
 * @brief Filter to keep only specific viewIds.
 */
void filterMatchesByViews(
  PairwiseMatches& matches,
  const std::set<IndexT>& viewsKeys);

/**
 * @brief Filter to keep only the \c limitNum first matches per descriptor type.
 */
void filterTopMatches(
  PairwiseMatches& allMatches,
  const int limitNum);

/**
 * @brief Save match files.
 *
 * @param[in] matches: container for the output matches
 * @param[in] sfm_data
 * @param[in] folder: folder containing the match files
 * @param[in] extension: txt or bin file format
 * @param[in] matchFilePerImage: do we store a global match file
 *            or one match file per image
 * @param[in] prefix: optional prefix for the output file(s)
 */
bool Save(
  const PairwiseMatches& matches,
  const std::string& folder,
  const std::string& extension,
  bool matchFilePerImage,
  const std::string& prefix="");

}  // namespace matching
}  // namespace aliceVision
