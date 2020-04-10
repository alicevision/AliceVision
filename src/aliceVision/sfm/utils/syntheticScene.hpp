// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfm {

/**
 * @brief Create features from a known SfMData (synthetic scene).
 * @param[out] out_featuresPerView
 * @param[in] sfmData synthetic SfM dataset
 * @param[in] descType
 * @param[in] noise
 */
template <typename NoiseGenerator>
void generateSyntheticFeatures(feature::FeaturesPerView& out_featuresPerView,
                               feature::EImageDescriberType descType,
                               const sfmData::SfMData& sfmData,
                               NoiseGenerator& noise)
{
  assert(descType != feature::EImageDescriberType::UNINITIALIZED);
  std::default_random_engine generator;

  // precompute output feature vectors size and resize
  {
    std::map<IndexT, std::size_t> nbFeatPerView;
    for(const auto& it: sfmData.getViews())
    {
      nbFeatPerView[it.first] = 0;
    }
    for(const auto& it: sfmData.getLandmarks())
    {
      const sfmData::Landmark& landmark = it.second;

      for(const auto& obsIt: landmark.observations)
      {
        const IndexT viewId = obsIt.first;
        const sfmData::Observation& obs = obsIt.second;
        nbFeatPerView[viewId] = std::max(nbFeatPerView[viewId], std::size_t(obs.id_feat+1));
      }
    }
    for(auto& it: nbFeatPerView)
    {
      // create Point Features vectors at the right size
      feature::PointFeatures pointFeatures(it.second);
      out_featuresPerView.addFeatures(it.first, descType, pointFeatures);
    }
  }
  // Use arbitrary values for feature scale and orientation
  const float scale = 0.0f;
  const float orientation = 0.0f;

  // Fill with the observation values
  for(const auto& it: sfmData.getLandmarks())
  {
    const sfmData::Landmark& landmark = it.second;

    for(const auto& obsIt: landmark.observations)
    {
      const IndexT viewId = obsIt.first;
      const sfmData::Observation& obs = obsIt.second;

      out_featuresPerView.getFeaturesPerDesc(viewId)[descType][obs.id_feat] = feature::PointFeature(obs.x(0) + noise(generator), obs.x(1) + noise(generator), scale, orientation);
    }
  }
}

/**
 * @brief Generate features matches between views from a known SfMData (synthetic scene).
 * @param[out] out_pairwiseMatches The output pairwiseMatches
 * @param[in] sfmData The synthetic SfM dataset
 * @param[in] descType The desciptor type
 */
void generateSyntheticMatches(matching::PairwiseMatches& out_pairwiseMatches, const sfmData::SfMData& sfmData, feature::EImageDescriberType descType);

// Translate a synthetic scene into a valid SfMData scene
// As only one intrinsic is defined we used shared intrinsic
sfmData::SfMData getInputScene(const NViewDataSet& d, const NViewDatasetConfigurator& config, camera::EINTRINSIC eintrinsic);

// Translate a synthetic scene into a valid SfMData scene
// As only one intrinsic is defined we used shared intrinsic
sfmData::SfMData getInputRigScene(const NViewDataSet& d, const NViewDatasetConfigurator& config, camera::EINTRINSIC eintrinsic);

} // namespace sfm
} // namespace aliceVision
