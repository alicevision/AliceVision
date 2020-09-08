// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp>

#include <boost/progress.hpp>

#include <vector>
#include <map>

namespace aliceVision {
namespace matchingImageCollection {

using namespace aliceVision::matching;

/**
 * @brief Perform robust model estimation (with optional guided_matching)
 * or all the pairs and regions correspondences contained in the putativeMatches set.
 * Allow to keep only geometrically coherent matches.
 * It discards pairs that do not lead to a valid robust model estimation.
 * @param[out] geometricMatches
 * @param[in] sfmData
 * @param[in] regionsPerView
 * @param[in] functor
 * @param[in] putativeMatches
 * @param[in] guidedMatching
 * @param[in] distanceRatio
 * @param[in] randomNumberGenerator
 */
template<typename GeometryFunctor>
void robustModelEstimation(
  PairwiseMatches& out_geometricMatches,
  const sfmData::SfMData* sfmData,
  const feature::RegionsPerView& regionsPerView,
  const GeometryFunctor& functor,
  const PairwiseMatches& putativeMatches,
  std::mt19937 & randomNumberGenerator,
  const bool guidedMatching = false,
  const double distanceRatio = 0.6
  )
{
  out_geometricMatches.clear();

  boost::progress_display progressBar(putativeMatches.size(), std::cout, "Robust Model Estimation\n");
  
#pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < (int)putativeMatches.size(); ++i)
  {
    PairwiseMatches::const_iterator iter = putativeMatches.begin();
    std::advance(iter, i);

    const Pair currentPair = iter->first;
    const MatchesPerDescType& putativeMatchesPerType = iter->second;
    const Pair& imagePair = iter->first;

    // apply the geometric filter (robust model estimation)
    {
      MatchesPerDescType inliers;
      GeometryFunctor geometricFilter = functor; // use a copy since we are in a multi-thread context
      const EstimationStatus state = geometricFilter.geometricEstimation(sfmData, regionsPerView, imagePair, putativeMatchesPerType, randomNumberGenerator, inliers);
      if(state.hasStrongSupport)
      {
        if(guidedMatching)
        {
          MatchesPerDescType guidedGeometricInliers;
          geometricFilter.Geometry_guided_matching(sfmData, regionsPerView, imagePair, distanceRatio, guidedGeometricInliers);
          //ALICEVISION_LOG_DEBUG("#before/#after: " << putative_inliers.size() << "/" << guided_geometric_inliers.size());
          std::swap(inliers, guidedGeometricInliers);
        }

#pragma omp critical
        {
          out_geometricMatches.emplace(currentPair, std::move(inliers));
        }

      }
    }

#pragma omp critical
    {
      ++progressBar;
    }
  }
}

} // namespace matchingImageCollection
} // namespace aliceVision


