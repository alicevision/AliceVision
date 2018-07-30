// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/Regions.hpp>
#include <aliceVision/feature/regionsFactory.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/matchingImageCollection/ImageCollectionMatcher_generic.hpp>

#include "aliceVision/sfmData/SfMData.hpp"
#include "aliceVision/sfmDataIO/sfmDataIO.hpp"

namespace aliceVision {
namespace feature {

/**
* @brief Compute the n best matches ('best' = mean of features' scale)
* @param[in] inputMatches Set of indices for (putative) matches.
* @param[in] regionsI Reference to the regions of the left image.
* @param[in] regionsJ Reference to the regions of the right image.
* @param[out] outputMatches Subset of inputMatches containing the best n matches, sorted.
*/
void sortMatches_byFeaturesScale(
	const aliceVision::matching::IndMatches& inputMatches,
	const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& regionsI,
	const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& regionsJ,
	aliceVision::matching::IndMatches& outputMatches);

/** 
 * @brief Sort matches according to their Lowe ratio (ascending order). 
 * @param[in,out] matches Set of indices for (putative) matches. 
 */ 
void sortMatches_byDistanceRatio(aliceVision::matching::IndMatches& matches);

/**
* @brief Compare method used in the match sorting.
* @param[in] firstElem The first element to be compared.
* @param[in] secondElem The second element to be compared.
* @return True if firstElem is less than secondElem.
*/
bool matchCompare(const std::pair<float, size_t>& firstElem, const std::pair<float, size_t>& secondElem);

/**
* @brief Extracts by copy the first (and best) uNumMatchesToKeep.
* @param[out] outputMatches Set of image pairs and their respective sets of matches thresholded to the first uNumMatchesToKeep.
* @param[in] uNumMatchesToKeep The N best matches to keep.
*/
void thresholdMatches(aliceVision::matching::IndMatches& outputMatches, const std::size_t uNumMatchesToKeep);

/**
 * @brief Perform the gris filtering on the matches
 * @param[in] lRegions The regions of the first picture
 * @param[in] rRegions The regions of the second picture
 * @param[in] indexImagePair The Pair of matched images
 * @param[in] sfm_data The sfm data file
 * @param[out] outMatches The remaining matches
 */
void matchesGridFiltering(const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& lRegions, 
        const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& rRegions, 
        const aliceVision::Pair& indexImagePair,
        const aliceVision::sfmData::SfMData sfm_data, 
        aliceVision::matching::IndMatches& outMatches);

}
}
