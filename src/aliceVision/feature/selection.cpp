// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "selection.hpp"

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace feature {

const size_t gridSize = 3;
  
/**
* @brief Sort the matches.
* @param[in] inputMatches Set of indices for (putative) matches.
* @param[in] regionsI Reference to the regions of the left image.
* @param[in] regionsJ Reference to the regions of the right image.
* @param[out] outputMatches Subset of inputMatches containing the best n matches, sorted.
*/
void sortMatches_byFeaturesScale(
	const aliceVision::matching::IndMatches& inputMatches,
	const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& regionsI,
	const aliceVision::feature::FeatRegions<aliceVision::feature::SIOPointFeature>& regionsJ,
	aliceVision::matching::IndMatches& outputMatches)
{
	const std::vector<aliceVision::feature::SIOPointFeature>& vecFeatureI = regionsI.Features();
	const std::vector<aliceVision::feature::SIOPointFeature>& vecFeatureJ = regionsJ.Features();

	//outputMatches will contain the sorted matches if inputMatches.
	outputMatches.reserve(inputMatches.size());

	//This vector is just a temporary container to link the index of the matches in the original vector inputMatches.
	//It will be used to retrieve the correct matches after the sort.
	std::vector<std::pair<float, size_t>> vecFeatureScale;

	for (size_t i = 0; i < inputMatches.size(); i++)
  {
		float scale1 = vecFeatureI[inputMatches[i]._i].scale();
		float scale2 = vecFeatureJ[inputMatches[i]._j].scale();
		vecFeatureScale.emplace_back((scale1 + scale2) / 2.0, i);
	}

	std::sort(vecFeatureScale.begin(), vecFeatureScale.end(), matchCompare);

	//The sorted match vector is filled according to the result of the sorting above.
	for (size_t i = 0; i < vecFeatureScale.size(); i++)
  {
		outputMatches.push_back(inputMatches[vecFeatureScale[i].second]);
	}
}

void sortMatches_byDistanceRatio(aliceVision::matching::IndMatches& matches) 
{ 
  struct { 
    bool operator() (const matching::IndMatch & m1, const matching::IndMatch & m2) const  
    { 
      return m1._distanceRatio < m2._distanceRatio; 
    } 
  } increasingLoweRatio; 
 
  std::sort(matches.begin(), matches.end(), increasingLoweRatio); 
} 

/**
* @brief Compare method used in the match sorting.
* @param[in] firstElem The first element to be compared.
* @param[in] secondElem The second element to be compared.
* @return True if firstElem is less than secondElem.
*/
bool matchCompare(const std::pair<float, size_t>& firstElem, const std::pair<float, size_t>& secondElem)
{
	return firstElem.first > secondElem.first;
}

/**
* @brief Extracts by copy the first (and best) uNumMatchesToKeep.
* @param[out] outputMatches Set of image pairs and their respective sets of matches thresholded to the first uNumMatchesToKeep.
* @param[in] uNumMatchesToKeep The N best matches to keep.
*/
void thresholdMatches(aliceVision::matching::IndMatches& outputMatches, const std::size_t uNumMatchesToKeep)
{
	if (outputMatches.size() > uNumMatchesToKeep) {
		outputMatches.resize(uNumMatchesToKeep);
	}
}

/**
 * @brief Perform the grid filtering on the matches
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
        aliceVision::matching::IndMatches& outMatches)
{
  const std::size_t lWidth = sfm_data.getViews().at(indexImagePair.first)->getWidth();
  const std::size_t lHeight = sfm_data.getViews().at(indexImagePair.first)->getHeight();
  const std::size_t rWidth = sfm_data.getViews().at(indexImagePair.second)->getWidth();
  const std::size_t rHeight = sfm_data.getViews().at(indexImagePair.second)->getHeight();
  
  const size_t leftCellHeight = std::ceil(lHeight / (float)gridSize);
  const size_t leftCellWidth = std::ceil(lWidth / (float)gridSize);
  const size_t rightCellHeight = std::ceil(rHeight / (float)gridSize);
  const size_t rightCellWidth = std::ceil(rWidth / (float)gridSize);

  std::vector< aliceVision::matching::IndMatches > completeGrid(gridSize*gridSize*2);
  // Reserve all cells
  for(aliceVision::matching::IndMatches& cell: completeGrid)
  {
    cell.reserve(outMatches.size()/completeGrid.size());
  }
  // Split matches in grid cells
  for(const auto& match: outMatches)
  {
    const aliceVision::feature::SIOPointFeature& leftPoint = lRegions.Features()[match._i];
    const aliceVision::feature::SIOPointFeature& rightPoint = rRegions.Features()[match._j];
    
    const float leftGridIndex_f = std::floor(leftPoint.x() / (float)leftCellWidth) + std::floor(leftPoint.y() / (float)leftCellHeight) * gridSize;
    const float rightGridIndex_f = std::floor(rightPoint.x() / (float)rightCellWidth) + std::floor(rightPoint.y() / (float)rightCellHeight) * gridSize;
    // clamp the values if we have feature/marker centers outside the image size.
    const std::size_t leftGridIndex = clamp(leftGridIndex_f, 0.f, float(gridSize-1));
    const std::size_t rightGridIndex = clamp(rightGridIndex_f, 0.f, float(gridSize-1));

    aliceVision::matching::IndMatches& currentCaseL = completeGrid[leftGridIndex];
    aliceVision::matching::IndMatches& currentCaseR = completeGrid[rightGridIndex + gridSize*gridSize];
    
    if(currentCaseL.size() <= currentCaseR.size())
    {
      currentCaseL.push_back(match);
    }
    else
    {
      currentCaseR.push_back(match);
    }
  }
  
  // max Size of the cells:
  int maxSize = 0;
  for (const auto& cell: completeGrid)
  {
    if(cell.size() > maxSize)
    {
      maxSize = cell.size();
    }
  }
  
  aliceVision::matching::IndMatches finalMatches;
  finalMatches.reserve(outMatches.size());
  
  // Combine all cells into a global ordered vector
  for (int cmpt = 0; cmpt < maxSize; ++cmpt)
  {
    for(const auto& cell: completeGrid)
    {
      if(cmpt < cell.size())
      {
        finalMatches.push_back(cell[cmpt]);
      }
    }
  }
  
  outMatches.swap(finalMatches);
}

}
}
