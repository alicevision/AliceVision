
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_IND_MATCH_UTILS_H
#define OPENMVG_MATCHING_IND_MATCH_UTILS_H

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/features/feature.hpp"

#include <string>

namespace openMVG {
namespace matching {

 /**
 * @brief For multiple descriptor techniques:
 * Create a vector which, for each descriptor, maps to the index
 * of the first descriptor associated with the same feature
 *
 * @param[in] feature: the list of features of the image
 * @param[out] indexVect: the mapping vector
 */
void createVectFeatures(const std::vector<features::PointFeature> &features, std::vector<IndexT>& indexVect);

/**
 * @brief For multiple descriptor techniques.
 * For each match of a vec_match file, remap the descriptor indexes
 * to the index of the first descriptor associated with the same feature.
 * (for track fusion purposes)
 *
 * @param[in] vec_match: the list of matches of the pair of images
 * @param[out] featuresI : the list of features of the image I
 * @param[out] featuresJ : the list of features of the image J
 */

void remapFeaturesToFirstIndex(std::vector<IndMatch> & vec_match,
        const std::vector<features::PointFeature> &featuresI,
        const std::vector<features::PointFeature> &featuresJ);

/**
 * @brief Load a match file.
 *
 * @param[out] matches: container for the output matches
 * @param[in] folder: folder containing the match files
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 */
bool LoadMatchFile(
  PairWiseMatches & matches,
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
  PairWiseMatches & matches,
  const std::set<IndexT> & viewsKeys,
  const std::string & folder,
  const std::string & mode);

/**
 * @brief Load match files.
 *
 * @param[out] matches: container for the output matches
 * @param[in] sfm_data
 * @param[in] folder: folder containing the match files
 * @param[in] mode: type of matching, it could be: "f", "e" or "putative".
 */
bool Load(
  PairWiseMatches & matches,
  const std::set<IndexT> & viewsKeys,
  const std::string & folder,
  const std::string & mode);

/**
 * @brief Filter to keep only specific viewIds.
 */
void filterMatches(
  PairWiseMatches & matches,
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
  const PairWiseMatches & matches,
  const std::string & folder,
  const std::string & mode,
  const std::string & extension,
  bool matchFilePerImage);

}  // namespace matching
}  // namespace openMVG

#endif // #define OPENMVG_MATCHING_IND_MATCH_UTILS_H
