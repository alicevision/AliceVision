// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

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

bool GeometricFilterMatrix_HGrowing::growHomography(const std::vector<feature::SIOPointFeature> &featuresI,
                                                    const std::vector<feature::SIOPointFeature> &featuresJ,
                                                    const matching::IndMatches &matches,
                                                    const IndexT &seedMatchId,
                                                    std::set<IndexT> &planarMatchesIndices,
                                                    Mat3 &transformation) const
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



bool growHomography(const std::vector<feature::SIOPointFeature> &featuresI,
                    const std::vector<feature::SIOPointFeature> &featuresJ,
                    const matching::IndMatches &matches,
                    const IndexT &seedMatchId,
                    std::set<IndexT> &planarMatchesIndices, Mat3 &transformation,
                    const GrowParameters& param)
{
  assert(seedMatchId <= matches.size());

  planarMatchesIndices.clear();
  transformation = Mat3::Identity();

  const matching::IndMatch & seedMatch = matches.at(seedMatchId);
  const feature::SIOPointFeature & seedFeatureI = featuresI.at(seedMatch._i);
  const feature::SIOPointFeature & seedFeatureJ = featuresJ.at(seedMatch._j);

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
//      if (planarMatchesIndices.size() >= _maxFractionPlanarMatches * matches.size())
//        break;
  }
  return !transformation.isIdentity();
}



void filterMatchesByHGrowing(const std::vector<feature::SIOPointFeature>& siofeatures_I,
                             const std::vector<feature::SIOPointFeature>& siofeatures_J,
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
          if(iH == 3) std::cout << "best iMatch" << iMatch << std::endl;
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

bool refineHomography(const std::vector<feature::SIOPointFeature> &featuresI,
                      const std::vector<feature::SIOPointFeature> &featuresJ,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance)
{
  Mat2X pointsI;
  Mat2X pointsJ;
  feature::PointsToMat(featuresI, pointsI);
  feature::PointsToMat(featuresJ, pointsJ);
  return refineHomography(pointsI,
                          pointsJ,
                          remainingMatches,
                          homography,
                          bestMatchesId,
                          homographyTolerance);
}

bool refineHomography(const Mat2X& features_I,
                      const Mat2X& features_J,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance)
{
  ceres::Problem problem;
  // use a copy for the optimization to avoid changes in the input one
  Mat3 tempHomography = homography;

  for(IndexT matchId : bestMatchesId)
  {
    const matching::IndMatch& match = remainingMatches.at(matchId);

    const Vec2& x1 = features_I.col(match._i);
    const Vec2& x2 = features_J.col(match._j);

    RefineHRobustCostFunctor* costFun =
            new RefineHRobustCostFunctor(x1, x2, homographyTolerance);

    problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<
                    RefineHRobustCostFunctor,
                    1,
                    9>(costFun),
            nullptr,
            tempHomography.data());
  }

  ceres::Solver::Options solverOpt;
  solverOpt.max_num_iterations = 10;

  ceres::Solver::Summary summary;
  ceres::Solve(solverOpt,&problem,&summary);

  // if the optimization did not succeed return without changing
  if(!summary.IsSolutionUsable())
    return false;

  homography = tempHomography;

  // normalize the homography
  if(std::fabs(homography(2, 2)) > std::numeric_limits<double>::epsilon())
    homography /= homography(2,2);

  findTransformationInliers(features_I, features_J, remainingMatches, homography, homographyTolerance, bestMatchesId);

  return true;
}

}
}