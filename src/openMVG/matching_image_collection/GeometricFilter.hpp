
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/features/feature.hpp"
#include "openMVG/features/RegionsPerView.hpp"
#include "openMVG/matching/indMatch.hpp"

#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"

#include <vector>
#include <map>

namespace openMVG {
namespace matching_image_collection {

using namespace openMVG::matching;

/// Allow to keep only geometrically coherent matches
/// -> It discards pairs that do not lead to a valid robust model estimation
struct ImageCollectionGeometricFilter
{
  ImageCollectionGeometricFilter(
    const sfm::SfM_Data * sfm_data,
    const features::RegionsPerView & regionsPerView
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
  const sfm::SfM_Data * _sfm_data;
  const features::RegionsPerView & _regionsPerView;
  PairwiseMatches _map_GeometricMatches;
};

template<typename GeometryFunctor>
void ImageCollectionGeometricFilter::Robust_model_estimation(
  const GeometryFunctor & functor,
  const PairwiseMatches & putative_matches,
  const bool b_guided_matching,
  const double d_distance_ratio)
{
  C_Progress_display my_progress_bar( putative_matches.size() );

#ifdef OPENMVG_USE_OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
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
      if (geometricFilter.Robust_estimation(_sfm_data, _regionsPerView, imagePair, putativeMatchesPerType, inliers))
      {
        if (b_guided_matching)
        {
          MatchesPerDescType guided_geometric_inliers;
          geometricFilter.Geometry_guided_matching(_sfm_data, _regionsPerView, imagePair, d_distance_ratio, guided_geometric_inliers);
          //OPENMVG_LOG_DEBUG("#before/#after: " << putative_inliers.size() << "/" << guided_geometric_inliers.size());
          std::swap(inliers, guided_geometric_inliers);
        }

#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
        {
          _map_GeometricMatches.insert(std::make_pair(current_pair, std::move(inliers)));
        }
      }
    }
#ifdef OPENMVG_USE_OPENMP
#pragma omp critical
#endif
    {
      ++my_progress_bar;
    }
  }
}

} // namespace openMVG
} // namespace matching_image_collection


