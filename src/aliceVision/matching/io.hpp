// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 */
bool LoadMatchFile(
  PairwiseMatches & matches,
  const std::string & folder,
  const std::string & filename);

/**
 * @brief Load the match file for each image.
 *
 * @param[out] matches: container for the output matches
 * @param[in] folder: folder containing the match files
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 */
bool LoadMatchFilePerImage(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeys,
  const std::string & folder,
  const std::string & mode);

/**
 * @brief Load match files.
 *
 * @param[out] matches: container for the output matches
 * @param[in] sfm_data
 * @param[in] folder: folder containing the match files
 * @param[in] descTypes
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 */
bool Load(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeysFilter,
  const std::string & folder,
  const std::vector<feature::EImageDescriberType>& descTypesFilter,
  const std::string & mode);

/**
 * @brief Filter to keep only specific viewIds.
 */
void filterMatchesByViews(
  PairwiseMatches & matches,
  const std::set<IndexT> & viewsKeys);

/**
 * @brief Save match files.
 *
 * @param[in] matches: container for the output matches
 * @param[in] sfm_data
 * @param[in] folder: folder containing the match files
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 * @param[in] extension: txt or bin file format
 * @param[in] matchFilePerImage: do we store a global match file
 *            or one match file per image
 */
bool Save(
  const PairwiseMatches & matches,
  const std::string & folder,
  const std::string & mode,
  const std::string & extension,
  bool matchFilePerImage);

}  // namespace matching
}  // namespace aliceVision
