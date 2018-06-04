// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GeometricFilterMatrix_HGrowing.hpp"
#include "geometricFilterUtils.hpp"

namespace aliceVision {
namespace matchingImageCollection {


bool aliceVision::matchingImageCollection::GeometricFilterMatrix_HGrowing::getMatches(
        const feature::EImageDescriberType &descType,
        const IndexT homographyId, matching::IndMatches &matches) const
{
  matches.clear();

  if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
    return false;
  if (homographyId > _HsAndMatchesPerDesc.at(descType).size() - 1)
    return false;

  matches = _HsAndMatchesPerDesc.at(descType).at(homographyId).second;

  if (matches.empty())
    return false;

  return true;
}

std::size_t GeometricFilterMatrix_HGrowing::getNbHomographies(const feature::EImageDescriberType &descType) const
{
  if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
    return 0;
  return _HsAndMatchesPerDesc.at(descType).size();
}


std::size_t GeometricFilterMatrix_HGrowing::getNbVerifiedMatches(const feature::EImageDescriberType &descType,
                                                                 const IndexT homographyId) const
{
  if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
    return 0;
  if (homographyId > _HsAndMatchesPerDesc.at(descType).size() - 1)
    return 0;

  return _HsAndMatchesPerDesc.at(descType).at(homographyId).second.size();
}

std::size_t GeometricFilterMatrix_HGrowing::getNbAllVerifiedMatches() const
{
  std::size_t counter = 0;
  for (const auto & HnMs : _HsAndMatchesPerDesc)
  {
    for (const std::pair<Mat3, matching::IndMatches> & HnM : HnMs.second)
    {
      counter += HnM.second.size();
    }
  }
  return counter;
}

bool GeometricFilterMatrix_HGrowing::growHomography(const std::vector<feature::SIOPointFeature> &featuresI,
                                                    const std::vector<feature::SIOPointFeature> &featuresJ,
                                                    const matching::IndMatches &matches, const IndexT &seedMatchId,
                                                    std::set<IndexT> &planarMatchesIndices, Mat3 &transformation) const
{
  assert(seedMatchId <= matches.size());

  planarMatchesIndices.clear();
  transformation = Mat3::Identity();

  const matching::IndMatch & seedMatch = matches.at(seedMatchId);
  const feature::SIOPointFeature & seedFeatureI = featuresI.at(seedMatch._i);
  const feature::SIOPointFeature & seedFeatureJ = featuresJ.at(seedMatch._j);

  double currTolerance;

  for (IndexT iRefineStep = 0; iRefineStep < _nbRefiningIterations; ++iRefineStep)
  {
    if (iRefineStep == 0)
    {
      computeSimilarity(seedFeatureI, seedFeatureJ, transformation);
      currTolerance = _similarityTolerance;
    }
    else if (iRefineStep <= 4)
    {
      estimateAffinity(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
      currTolerance = _affinityTolerance;
    }
    else
    {
      estimateHomography(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
      currTolerance = _homographyTolerance;
    }

    findTransformationInliers(featuresI, featuresJ, matches, transformation, currTolerance, planarMatchesIndices);

    if (planarMatchesIndices.size() < _minInliersToRefine)
      return false;

    // Note: the following statement is present in the MATLAB code but not implemented in YASM
//      if (planarMatchesIndices.size() >= _maxFractionPlanarMatches * matches.size())
//        break;
  }
  return !transformation.isIdentity();
}

}
}