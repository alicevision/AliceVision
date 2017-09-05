// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <openMVG/matching/indMatch.hpp>
#include <openMVG/features/FeaturesPerView.hpp>

namespace openMVG {
namespace matching_image_collection {

// TODO: remove PointFeature to avoid this hack
inline Vec2 getFeaturePosition(const std::unique_ptr<features::Regions>& regions, std::size_t i)
{
  return regions->GetRegionPosition(i);
}

inline Vec2 getFeaturePosition(const features::PointFeatures& features, std::size_t i)
{
  return features[i].coords().cast<double>();
}

/**
* @brief Fill matrices with un-distorted feature positions
*
* @param[in] putativeMatches Selected corresponding features id (match)
* @param[in] cam_I Inth Camera interface
* @param[in] feature_I Inth view features
* @param[in] cam_J Jnth Camera interface
* @param[in] feature_J Jnth view features
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT, class FeatOrRegions>
void fillMatricesWithUndistortFeaturesMatches(
  const matching::IndMatches & putativeMatches,
  const cameras::IntrinsicBase * cam_I,
  const FeatOrRegions & feature_I,
  const cameras::IntrinsicBase * cam_J,
  const FeatOrRegions & feature_J,
  MatT & x_I, MatT & x_J)
{
  typedef typename MatT::Scalar Scalar; // Output matrix type

  const bool I_hasValidIntrinsics = cam_I && cam_I->isValid();
  const bool J_hasValidIntrinsics = cam_J && cam_J->isValid();

  if (I_hasValidIntrinsics)
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const Vec2 pt_I = getFeaturePosition(feature_I, putativeMatches[i]._i);
      x_I.col(i) = cam_I->get_ud_pixel(pt_I);
    }
  }
  else
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const Vec2 pt_I = getFeaturePosition(feature_I, putativeMatches[i]._i);
      x_I.col(i) = pt_I;
    }
  }
  if (J_hasValidIntrinsics)
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const Vec2 pt_J = getFeaturePosition(feature_J, putativeMatches[i]._j);
      x_J.col(i) = cam_J->get_ud_pixel(pt_J);
    }
  }
  else
  {
    for (size_t i=0; i < putativeMatches.size(); ++i)
    {
      const Vec2 pt_J = getFeaturePosition(feature_J, putativeMatches[i]._j);
      x_J.col(i) = pt_J;
    }
  }
}

/**
* @brief Get un-distorted feature positions for the pair pairIndex from the Features_Provider interface
* @param[in] putativeMatchesPerType Matches of the 'pairIndex' pair
* @param[in] cam_I
* @param[in] cam_J
* @param[in] features_I
* @param[in] features_J
* @param[in] descTypes
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT, class MapFeatOrRegionPerDesc>
void MatchesPairToMat(
  const matching::MatchesPerDescType & putativeMatchesPerType,
  const cameras::IntrinsicBase * cam_I,
  const cameras::IntrinsicBase * cam_J,
  const MapFeatOrRegionPerDesc& features_I,
  const MapFeatOrRegionPerDesc& features_J,
  const std::vector<features::EImageDescriberType>& descTypes,
  MatT & x_I, MatT & x_J)
{
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

    const auto& feature_I = features_I.at(descType);
    const auto& feature_J = features_J.at(descType);

    // auto b_I = x_I.block(y, 0, putativeMatches.size(), 2);
    // auto b_J = x_J.block(y, 0, putativeMatches.size(), 2);
    auto subpart_I = x_I.block(0, y, 2, putativeMatches.size());
    auto subpart_J = x_J.block(0, y, 2, putativeMatches.size());

    // fill subpart of the matrices with undistorted features
    fillMatricesWithUndistortFeaturesMatches(
      putativeMatches,
      cam_I, feature_I,
      cam_J, feature_J,
      subpart_I,
      subpart_J);

    y += putativeMatches.size();
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
  const cameras::IntrinsicBase * cam_I = sfmData->GetIntrinsicPtr(view_I->getIntrinsicId());
  const cameras::IntrinsicBase * cam_J = sfmData->GetIntrinsicPtr(view_J->getIntrinsicId());

  MatchesPairToMat(
      putativeMatchesPerType,
      cam_I,
      cam_J,
      regionsPerView.getRegionsPerDesc(pairIndex.first),
      regionsPerView.getRegionsPerDesc(pairIndex.second),
      descTypes,
      x_I, x_J);
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
