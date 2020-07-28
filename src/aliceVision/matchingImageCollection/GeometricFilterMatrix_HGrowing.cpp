// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/matching/svgVisualization.hpp>
#include "GeometricFilterMatrix_HGrowing.hpp"

namespace aliceVision {
namespace matchingImageCollection {


bool GeometricFilterMatrix_HGrowing::getMatches(const feature::EImageDescriberType &descType,
                                                const IndexT homographyId, matching::IndMatches &matches) const
{
  matches.clear();

  if (_HsAndMatchesPerDesc.find(descType) == _HsAndMatchesPerDesc.end())
    return false;
  if (homographyId > _HsAndMatchesPerDesc.at(descType).size() - 1)
    return false;

  matches = _HsAndMatchesPerDesc.at(descType).at(homographyId).second;

  return !matches.empty();

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



bool growHomography(const std::vector<feature::PointFeature> &featuresI,
                    const std::vector<feature::PointFeature> &featuresJ,
                    const matching::IndMatches &matches,
                    const IndexT &seedMatchId,
                    std::set<IndexT> &planarMatchesIndices, Mat3 &transformation,
                    const GrowParameters& param)
{
  assert(seedMatchId <= matches.size());

  planarMatchesIndices.clear();
  transformation = Mat3::Identity();

  const matching::IndMatch & seedMatch = matches.at(seedMatchId);
  const feature::PointFeature & seedFeatureI = featuresI.at(seedMatch._i);
  const feature::PointFeature & seedFeatureJ = featuresJ.at(seedMatch._j);

  double currTolerance;

  for (IndexT iRefineStep = 0; iRefineStep < param._nbRefiningIterations; ++iRefineStep)
  {
    if (iRefineStep == 0)
    {
      computeSimilarity(seedFeatureI, seedFeatureJ, transformation);
      currTolerance = param._similarityTolerance;
    }
    else if (iRefineStep <= 4)
    {
      estimateAffinity(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
      currTolerance = param._affinityTolerance;
    }
    else
    {
      estimateHomography(featuresI, featuresJ, matches, transformation, planarMatchesIndices);
      currTolerance = param._homographyTolerance;
    }

    findTransformationInliers(featuresI, featuresJ, matches, transformation, currTolerance, planarMatchesIndices);

    if (planarMatchesIndices.size() < param._minInliersToRefine)
      return false;

    // Note: the following statement is present in the MATLAB code but not implemented in YASM
//      if (planarMatchesIndices.size() >= param._maxFractionPlanarMatches * matches.size())
//        break;
  }
  return !transformation.isIdentity();
}



void filterMatchesByHGrowing(const std::vector<feature::PointFeature>& siofeatures_I,
                             const std::vector<feature::PointFeature>& siofeatures_J,
                             const matching::IndMatches& putativeMatches,
                             std::vector<std::pair<Mat3, matching::IndMatches>>& homographiesAndMatches,
                             matching::IndMatches& outGeometricInliers,
                             const HGrowingFilteringParam& param)
{

  using namespace aliceVision::feature;
  using namespace aliceVision::matching;

  IndMatches remainingMatches = putativeMatches;
  GeometricFilterMatrix_HGrowing dummy;

  for(IndexT iH = 0; iH < param._maxNbHomographies; ++iH)
  {
    std::set<IndexT> usedMatchesId, bestMatchesId;
    Mat3 bestHomography;

    // -- Estimate H using homography-growing approach
    #pragma omp parallel for // (huge optimization but modify results a little)
    for(int iMatch = 0; iMatch < remainingMatches.size(); ++iMatch)
    {
      // Growing a homography from one match ([F.Srajer, 2016] algo. 1, p. 20)
      // each match is used once only per homography estimation (increases computation time) [1st improvement ([F.Srajer, 2016] p. 20) ]
      if (usedMatchesId.find(iMatch) != usedMatchesId.end())
        continue;
      std::set<IndexT> planarMatchesId; // be careful: it contains the id. in the 'remainingMatches' vector not 'putativeMatches' vector.
      Mat3 homography;

      if (!growHomography(siofeatures_I,
                          siofeatures_J,
                          remainingMatches,
                          iMatch,
                          planarMatchesId,
                          homography,
                          param._growParam))
      {
        continue;
      }

      #pragma omp critical
      usedMatchesId.insert(planarMatchesId.begin(), planarMatchesId.end());

      if (planarMatchesId.size() > bestMatchesId.size())
      {
        #pragma omp critical
        {
          // if(iH == 3) { std::cout << "best iMatch" << iMatch << std::endl; }
          bestMatchesId = planarMatchesId; // be careful: it contains the id. in the 'remainingMatches' vector not 'putativeMatches' vector.
          bestHomography = homography;
        }
      }
    } // 'iMatch'

    // -- Refine H using Ceres minimizer
    refineHomography(siofeatures_I, siofeatures_J, remainingMatches, bestHomography, bestMatchesId, param._growParam._homographyTolerance);

    // stop when the models get too small
    if (bestMatchesId.size() < param._minNbMatchesPerH)
      break;

    // Store validated results:
    {
      IndMatches matches;
      Mat3 H = bestHomography;
      for (IndexT id : bestMatchesId)
      {
        matches.push_back(remainingMatches.at(id));
      }
      homographiesAndMatches.emplace_back(H, matches);
    }

    // -- Update not used matches & Save geometrically verified matches
    for (IndexT id : bestMatchesId)
    {
      outGeometricInliers.push_back(remainingMatches.at(id));
    }

    // update remaining matches (/!\ Keep ordering)
    std::size_t cpt = 0;
    for (IndexT id : bestMatchesId)
    {
      remainingMatches.erase(remainingMatches.begin() + id - cpt);
      ++cpt;
    }

    // stop when the number of remaining matches is too small
    if (remainingMatches.size() < param._minNbMatchesPerH)
      break;

  } // 'iH'

}



void drawHomographyMatches(const sfmData::View &viewI,
                           const sfmData::View &viewJ,
                           const std::vector<feature::PointFeature> &siofeatures_I,
                           const std::vector<feature::PointFeature> &siofeatures_J,
                           const std::vector<std::pair<Mat3, matching::IndMatches>> &homographiesAndMatches,
                           const matching::IndMatches &putativeMatches,
                           const std::string &outFilename)
{

  const std::string& imagePathLeft = viewI.getImagePath();
  const std::string& imagePathRight = viewJ.getImagePath();
  const auto imageSizeLeft = std::make_pair(viewI.getWidth(), viewI.getHeight());
  const auto imageSizeRight = std::make_pair(viewJ.getWidth(), viewJ.getHeight());

  drawHomographyMatches(imagePathLeft,
                        imageSizeLeft,
                        siofeatures_I,
                        imagePathRight,
                        imageSizeRight,
                        siofeatures_J,
                        homographiesAndMatches,
                        putativeMatches,
                        outFilename);
}

}
}
