
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include <openMVG/matching/indMatch.hpp>

#include <openMVG/features/FeaturesPerView.hpp>


namespace openMVG {
namespace matching_image_collection {

/**
* @brief Fill matrices with un-distorted feature positions ("image perfect" features)
*
* @param[in] putativeMatches Selected corresponding features id (match)
* @param[in] cam_I Inth Camera interface
* @param[in] feature_I Inth view features
* @param[in] cam_J Jnth Camera interface
* @param[in] feature_J Jnth view features
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT >
void fillMatricesWithUndistortFeaturesMatches(
  const matching::IndMatches & putativeMatches,
  const cameras::IntrinsicBase * cam_I,
  const features::PointFeatures & feature_I,
  const cameras::IntrinsicBase * cam_J,
  const features::PointFeatures & feature_J,
  MatT & x_I, MatT & x_J)
{
  typedef typename MatT::Scalar Scalar; // Output matrix type

  const bool I_hasValidIntrinsics = cam_I && cam_I->isValid();
  const bool J_hasValidIntrinsics = cam_J && cam_J->isValid();

  if (I_hasValidIntrinsics)
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const features::PointFeature & pt_I = feature_I[putativeMatches[i]._i];
      x_I.col(i) = cam_I->get_ud_pixel(pt_I.coords().cast<double>());
    }
  }
  else
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const features::PointFeature & pt_I = feature_I[putativeMatches[i]._i];
      x_I.col(i) = pt_I.coords().cast<double>();
    }
  }
  if (J_hasValidIntrinsics)
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const features::PointFeature & pt_J = feature_J[putativeMatches[i]._j];
      x_J.col(i) = cam_J->get_ud_pixel(pt_J.coords().cast<double>());
    }
  }
  else
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const features::PointFeature & pt_J = feature_J[putativeMatches[i]._j];
      x_J.col(i) = pt_J.coords().cast<double>();
    }
  }
}

/**
* @brief Get un-distorted feature positions for the pair pairIndex from the RegionsPerView interface
* @param[in] pairIndex Pair from which you need to extract the corresponding points
* @param[in] putativeMatches Matches of the 'pairIndex' pair
* @param[in] sfm_data SfM_Data scene container
* @param[in] regionsPerView Interface that provides the features positions
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT >
void MatchesPairToMat(
  const Pair pairIndex,
  const matching::MatchesPerDescType & putativeMatchesPerType,
  const sfm::SfM_Data * sfmData,
  const features::RegionsPerView& regionsPerView,
  const std::vector<features::EImageDescriberType>& descTypes,
  MatT & x_I, MatT & x_J)
{
  const sfm::View * view_I = sfmData->views.at(pairIndex.first).get();
  const sfm::View * view_J = sfmData->views.at(pairIndex.second).get();

  // Retrieve corresponding pair camera intrinsic if any
  const cameras::IntrinsicBase * cam_I =
    sfmData->GetIntrinsics().count(view_I->id_intrinsic) ?
      sfmData->GetIntrinsics().at(view_I->id_intrinsic).get() : nullptr;
  const cameras::IntrinsicBase * cam_J =
    sfmData->GetIntrinsics().count(view_J->id_intrinsic) ?
      sfmData->GetIntrinsics().at(view_J->id_intrinsic).get() : nullptr;

  // Create the output matrices with all matched features for images I and J
  const size_t n = putativeMatchesPerType.getNbAllMatches();
  x_I.resize(2, n);
  x_J.resize(2, n);

  size_t startM = 0;
  for(size_t d = 0; d < descTypes.size(); ++d)
  {
    const features::EImageDescriberType& descType = descTypes[d];

    if(!putativeMatchesPerType.count(descType))
      continue; // we may have 0 feature for some descriptor types
    const matching::IndMatches& putativeMatches = putativeMatchesPerType.at(descType);

    const features::PointFeatures feature_I = regionsPerView.getRegions(pairIndex.first, descType).GetRegionsPositions();
    const features::PointFeatures feature_J = regionsPerView.getRegions(pairIndex.second, descType).GetRegionsPositions();

    // fill subpart of the matrices with undistorted features

    auto subpart_I = x_I.block(0, startM, 2, putativeMatches.size());
    auto subpart_J = x_J.block(0, startM, 2, putativeMatches.size());

    fillMatricesWithUndistortFeaturesMatches(
      putativeMatches,
      cam_I, feature_I,
      cam_J, feature_J,
      subpart_I,
      subpart_J);

    startM += putativeMatches.size();
  }
}

/**
* @brief Get un-distorted feature positions for the pair pairIndex from the Features_Provider interface
* @param[in] pairIndex Pair from which you need to extract the corresponding points
* @param[in] putativeMatches Matches of the 'pairIndex' pair
* @param[in] sfm_data SfM_Data scene container
* @param[in] featuresProvider Interface that provides the features positions
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT >
void MatchesPairToMat(
  const Pair pairIndex,
  const matching::MatchesPerDescType & putativeMatchesPerType,
  const sfm::SfM_Data * sfmData,
  const features::FeaturesPerView& featuresPerView,
  const std::vector<features::EImageDescriberType>& descTypes,
  MatT & x_I, MatT & x_J)
{
  const sfm::View * view_I = sfmData->views.at(pairIndex.first).get();
  const sfm::View * view_J = sfmData->views.at(pairIndex.second).get();

  // Retrieve corresponding pair camera intrinsic if any
  const cameras::IntrinsicBase * cam_I =
    sfmData->GetIntrinsics().count(view_I->id_intrinsic) ?
      sfmData->GetIntrinsics().at(view_I->id_intrinsic).get() : nullptr;
  const cameras::IntrinsicBase * cam_J =
    sfmData->GetIntrinsics().count(view_J->id_intrinsic) ?
      sfmData->GetIntrinsics().at(view_J->id_intrinsic).get() : nullptr;

  // Create the output matrices with all matched features for images I and J

  const size_t n = putativeMatchesPerType.getNbAllMatches();
  x_I.resize(2, n);
  x_J.resize(2, n);

  size_t y = 0;
  for(size_t d = 0; d < descTypes.size(); ++d)
  {
    const features::EImageDescriberType& descType = descTypes[d];

    if(!putativeMatchesPerType.count(descType))
      continue; // we may have 0 feature for some descriptor types
    const matching::IndMatches& putativeMatches = putativeMatchesPerType.at(descType);


    const features::PointFeatures feature_I = featuresPerView.getFeatures(pairIndex.first, descType);
    const features::PointFeatures feature_J = featuresPerView.getFeatures(pairIndex.second, descType);

    // fill subpart of the matrices with undistorted features
    fillMatricesWithUndistortFeaturesMatches(
      putativeMatches,
      cam_I, feature_I,
      cam_J, feature_J,
      x_I.block(y, 0, putativeMatches.size(), 2), x_J.block(y, 0, putativeMatches.size(), 2));

    y += putativeMatches.size();
  }
}

/**
 * @brief copyInlierMatches
 * @param[in] inliers
 * @param[in] putativeMatchesPerType
 * @param[in] descTypes
 * @param[out] out_geometricInliersPerType
 */
void copyInlierMatches(
  const std::vector<size_t>& inliers,
  const matching::MatchesPerDescType & putativeMatchesPerType,
  const std::vector<features::EImageDescriberType> descTypes,
  matching::MatchesPerDescType & out_geometricInliersPerType)
{
  std::vector<size_t> orderedInliers = inliers;
  std::sort(orderedInliers.begin(), orderedInliers.end());

  size_t currentDescType = 0;
  size_t currentDescTypeStartIndex = 0;
  size_t currentDescTypeMaxLength = putativeMatchesPerType.getNbMatches(descTypes[currentDescType]);
  for(const size_t globalInlierIndex : orderedInliers)
  {
    while(globalInlierIndex >= currentDescTypeMaxLength)
    {
      ++currentDescType;
      currentDescTypeStartIndex = currentDescTypeMaxLength;
      currentDescTypeMaxLength += putativeMatchesPerType.getNbMatches(descTypes[currentDescType]);
    }
    const size_t localIndex = globalInlierIndex - currentDescTypeStartIndex;
    const auto descType = descTypes[currentDescType];
    out_geometricInliersPerType[descType].push_back( putativeMatchesPerType.at(descType)[localIndex] );
  }
}

} // namespace openMVG
} //namespace matching_image_collection
