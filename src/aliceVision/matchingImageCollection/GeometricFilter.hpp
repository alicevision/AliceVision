// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include "aliceVision/feature/PointFeature.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <boost/progress.hpp>

#include <vector>
#include <map>

namespace aliceVision {
namespace matchingImageCollection {

using namespace aliceVision::matching;

/// Allow to keep only geometrically coherent matches
/// -> It discards pairs that do not lead to a valid robust model estimation
struct GeometricFilter
{
  GeometricFilter(
    const sfm::SfMData * sfm_data,
    const feature::RegionsPerView & regionsPerView
  ):_sfm_data(sfm_data), _regionsPerView(regionsPerView)
  {}

  /// Perform robust model estimation (with optional guided_matching) for all the pairs and regions correspondences contained in the putative_matches set.
  template<typename GeometryFunctor>
  void Robust_model_estimation(
    const GeometryFunctor & functor,
    const PairwiseMatches & putative_matches,
    const bool b_guided_matching = false,
    const double d_distance_ratio = 0.6
  );

  const PairwiseMatches & Get_geometric_matches() const {return _map_GeometricMatches;}

  // Data
  const sfm::SfMData * _sfm_data;
  const feature::RegionsPerView & _regionsPerView;
  PairwiseMatches _map_GeometricMatches;
};

template<typename GeometryFunctor>
void GeometricFilter::Robust_model_estimation(
  const GeometryFunctor & functor,
  const PairwiseMatches & putative_matches,
  const bool b_guided_matching,
  const double d_distance_ratio)
{
  boost::progress_display my_progress_bar( putative_matches.size() );
  
  #pragma omp parallel for schedule(dynamic)
  for (int i = 0; i < (int)putative_matches.size(); ++i)
  {
    PairwiseMatches::const_iterator iter = putative_matches.begin();
    advance(iter,i);

    Pair current_pair = iter->first;
    const MatchesPerDescType & putativeMatchesPerType = iter->second;
    const Pair& imagePair = iter->first;

    //-- Apply the geometric filter (robust model estimation)
    {
      MatchesPerDescType inliers;
      GeometryFunctor geometricFilter = functor; // use a copy since we are in a multi-thread context
      const EstimationStatus state = geometricFilter.geometricEstimation(_sfm_data, _regionsPerView, imagePair, putativeMatchesPerType, inliers);
      if (state.hasStrongSupport)
      {
        if (b_guided_matching)
        {
          MatchesPerDescType guided_geometric_inliers;
          geometricFilter.Geometry_guided_matching(_sfm_data, _regionsPerView, imagePair, d_distance_ratio, guided_geometric_inliers);
          //ALICEVISION_LOG_DEBUG("#before/#after: " << putative_inliers.size() << "/" << guided_geometric_inliers.size());
          std::swap(inliers, guided_geometric_inliers);
        }
        #pragma omp critical
        {
          _map_GeometricMatches.insert(std::make_pair(current_pair, std::move(inliers)));
        }
      }
    }
    #pragma omp critical
    {
      ++my_progress_bar;
    }
  }
}

} // namespace aliceVision
} // namespace matchingImageCollection


