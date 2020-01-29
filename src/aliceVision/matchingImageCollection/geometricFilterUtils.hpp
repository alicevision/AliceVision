// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>

namespace aliceVision {
namespace matchingImageCollection {

// TODO: remove PointFeature to avoid this hack
inline Vec2 getFeaturePosition(const std::unique_ptr<feature::Regions>& regions, std::size_t i)
{
  return regions->GetRegionPosition(i);
}

inline Vec2 getFeaturePosition(const feature::PointFeatures& features, std::size_t i)
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
void fillMatricesWithUndistortFeaturesMatches(const matching::IndMatches &putativeMatches,
                                              const camera::IntrinsicBase *cam_I,
                                              const FeatOrRegions &feature_I,
                                              const camera::IntrinsicBase *cam_J,
                                              const FeatOrRegions &feature_J,
                                              MatT & x_I, MatT & x_J)
{
  typedef typename MatT::Scalar Scalar; // Output matrix type

  const bool I_hasValidIntrinsics = cam_I && cam_I->isValid() && cam_I->hasDistortion();
  const bool J_hasValidIntrinsics = cam_J && cam_J->isValid() && cam_J->hasDistortion();

  for (size_t i = 0; i < putativeMatches.size(); ++i)
  {
    const Vec2 pt_I = getFeaturePosition(feature_I, putativeMatches[i]._i);
    if (I_hasValidIntrinsics)
      x_I.col(i) = cam_I->get_ud_pixel(pt_I);
    else
      x_I.col(i) = pt_I;
  }

  for (size_t i = 0; i < putativeMatches.size(); ++i)
  {
    const Vec2 pt_J = getFeaturePosition(feature_J, putativeMatches[i]._j);
    if (J_hasValidIntrinsics)
      x_J.col(i) = cam_J->get_ud_pixel(pt_J);
    else
      x_J.col(i) = pt_J;
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
void fillMatricesWithUndistortFeaturesMatches(const matching::MatchesPerDescType &putativeMatchesPerType,
                      const camera::IntrinsicBase *cam_I,
                      const camera::IntrinsicBase *cam_J,
                      const MapFeatOrRegionPerDesc &features_I,
                      const MapFeatOrRegionPerDesc &features_J,
                      const std::vector<feature::EImageDescriberType> &descTypes,
                      MatT &x_I, MatT &x_J)
{
  // Create the output matrices with all matched features for images I and J
  const size_t n = putativeMatchesPerType.getNbAllMatches();
  x_I.resize(2, n);
  x_J.resize(2, n);

  size_t y = 0;
  for (const auto& descType : descTypes)
  {
    if(!putativeMatchesPerType.count(descType))
    {
      // we may have 0 feature for some descriptor types
      continue;
    }

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
 * @param[in] sfm_data SfMData scene container
 * @param[in] regionsPerView Interface that provides the features positions
 * @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
 * @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
 */
template<typename MatT >
void fillMatricesWithUndistortFeaturesMatches(const Pair &pairIndex,
                      const matching::MatchesPerDescType &putativeMatchesPerType,
                      const sfmData::SfMData *sfmData,
                      const feature::RegionsPerView &regionsPerView,
                      const std::vector<feature::EImageDescriberType> &descTypes,
                      MatT &x_I, MatT &x_J)
{
  const sfmData::View * view_I = sfmData->views.at(pairIndex.first).get();
  const sfmData::View * view_J = sfmData->views.at(pairIndex.second).get();

  // Retrieve corresponding pair camera intrinsic if any
  const camera::IntrinsicBase * cam_I = sfmData->getIntrinsicPtr(view_I->getIntrinsicId());
  const camera::IntrinsicBase * cam_J = sfmData->getIntrinsicPtr(view_J->getIntrinsicId());

  fillMatricesWithUndistortFeaturesMatches(
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
void copyInlierMatches(const std::vector<size_t> &inliers,
                       const matching::MatchesPerDescType &putativeMatchesPerType,
                       const std::vector<feature::EImageDescriberType> &descTypes,
                       matching::MatchesPerDescType &out_geometricInliersPerType);

/**
 * @brief Compute the transformation that standardize the input points so that
 * they are z-scores (i.e. zero mean and unit standard deviation).
 * @param[in] points2d The 2D inputs points.
 * @param[out] t the transformation that standardize the points.
 */
void centerMatrix(const Eigen::Matrix2Xf & points2d, Mat3 & t);

/**
 * @brief Compute the standardizing tranformation for the input features.
 * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/relative_pose.h#L1075
 * @param[in] featuresI The input matching features from the first image.
 * @param[in] featuresJ The input matching features from the secong image.
 * @param[in] matches Indicate which feature is concerned about the returned matrices.
 * @param[out] cI The standardizing matrix to apply to (the subpart of) \c featuresI
 * @param[out] cJ The standardizing matrix to apply to (the subpart of) \c featuresJ
 * @param[in] usefulMatchesId To consider a subpart of \c matches only.
 */
void centeringMatrices(const std::vector<feature::PointFeature> & featuresI,
                       const std::vector<feature::PointFeature> & featuresJ,
                       const matching::IndMatches & matches,
                       Mat3 & cI,
                       Mat3 & cJ,
                       const std::set<IndexT> & usefulMatchesId = std::set<IndexT>());
/**
 * @brief Compute the similarity transformation between 2 features (using their scale & orientation).
 * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/relative_pose.cpp#L1649
 * @param[in] feat1 The first feature with known scale & orientation.
 * @param[in] feat2 The second feature with known scale & orientation.
 * @param[out] S The similarity transformation between f1 et f2 so that f2 = S * f1.
 */
void computeSimilarity(const feature::PointFeature & feat1,
                       const feature::PointFeature & feat2,
                       Mat3 & S);

/**
 * @brief Estimate (using SVD) the Affinity transformation from a set of matches.
 * Based on: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L1669
 * @param[in] featuresI
 * @param[in] featuresJ
 * @param[in] matches The matches to consider for the estimation.
 * @param[out] affineTransformation The estimated Affine transformation.
 * @param[in] usefulMatchesId To consider a subpart of \c matches only.
 */
void estimateAffinity(const std::vector<feature::PointFeature> & featuresI,
                      const std::vector<feature::PointFeature> & featuresJ,
                      const matching::IndMatches & matches,
                      Mat3 & affineTransformation,
                      const std::set<IndexT> & usefulMatchesId = std::set<IndexT>());

/**
 * @brief estimateHomography Estimate (using SVD) the Homography transformation from a set of matches.
 * Based on: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L1694
 * @param[in] featuresI
 * @param[in] featuresJ
 * @param[in] matches The matches to consider for the estimation.
 * @param[out] H The estimated Homography transformation.
 * @param[in] usefulMatchesId To consider a subpart of \c matches only.
 */
void estimateHomography(const std::vector<feature::PointFeature> & featuresI,
                        const std::vector<feature::PointFeature> & featuresJ,
                        const matching::IndMatches & matches,
                        Mat3 &H,
                        const std::set<IndexT> & usefulMatchesId = std::set<IndexT>());

/**
 * @brief Return the id. of the matches with a reprojection error < to the desirered \c tolerance.
 * @param[in] featuresI
 * @param[in] featuresJ
 * @param[in] matches The matches to test.
 * @param[in] transformation The 3x3 transformation matrix.
 * @param[in] tolerance The tolerated pixel error.
 * @param[in] inliersId The index in the \c matches vector.
 */
void findTransformationInliers(const std::vector<feature::PointFeature> & featuresI,
                               const std::vector<feature::PointFeature> & featuresJ,
                               const matching::IndMatches & matches,
                               const Mat3 & transformation,
                               double tolerance,
                               std::set<IndexT> & inliersId);
/**
 * @brief Return the id. of the matches with a reprojection error < to the desirered \c tolerance.
 * @param[in] featuresI
 * @param[in] featuresJ
 * @param[in] matches The matches to test.
 * @param[in] transformation The 3x3 transformation matrix.
 * @param[in] tolerance The tolerated pixel error.
 * @param[in] inliersId The index in the \c matches vector.
 */
void findTransformationInliers(const Mat2X& featuresI,
                               const Mat2X& featuresJ,
                               const matching::IndMatches &matches,
                               const Mat3 &transformation,
                               double tolerance,
                               std::set<IndexT> &inliersId);


bool refineHomography(const std::vector<feature::PointFeature> &featuresI,
                      const std::vector<feature::PointFeature> &featuresJ,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance);

bool refineHomography(const Mat2X& features_I,
                      const Mat2X& features_J,
                      const matching::IndMatches& remainingMatches,
                      Mat3& homography,
                      std::set<IndexT>& bestMatchesId,
                      double homographyTolerance);

} // namespace aliceVision
} // namespace matchingImageCollection
