// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/matching/indMatch.hpp>
#include <aliceVision/features/regions.hpp>
#include <aliceVision/features/regions_factory.hpp>
#include <aliceVision/image/image.hpp>
#include <aliceVision/features/features.hpp>
#include <aliceVision/matching_image_collection/Matcher_Regions_AllInMemory.hpp>

#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/sfm/sfm_data_io.hpp"

namespace aliceVision {
namespace features {

/**
* @brief Compute the n best matches.
* @param[in] inputMatches Set of indices for (putative) matches.
* @param[in] regionsI Reference to the regions of the left image.
* @param[in] regionsJ Reference to the regions of the right image.
* @param[out] outputMatches Subset of inputMatches containing the best n matches, sorted.
*/
void sortMatches(
	const aliceVision::matching::IndMatches& inputMatches,
	const aliceVision::features::Feat_Regions<aliceVision::features::SIOPointFeature>& regionsI,
	const aliceVision::features::Feat_Regions<aliceVision::features::SIOPointFeature>& regionsJ,
	aliceVision::matching::IndMatches& outputMatches);


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
void matchesGridFiltering(const aliceVision::features::Feat_Regions<aliceVision::features::SIOPointFeature>& lRegions, 
        const aliceVision::features::Feat_Regions<aliceVision::features::SIOPointFeature>& rRegions, 
        const aliceVision::Pair& indexImagePair,
        const aliceVision::sfm::SfM_Data sfm_data, 
        aliceVision::matching::IndMatches& outMatches);

}
}
