// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/feature/FeaturesPerView.hpp>

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
void fillMatricesWithUndistortFeaturesMatches(
  const matching::IndMatches & putativeMatches,
  const camera::IntrinsicBase * cam_I,
  const FeatOrRegions & feature_I,
  const camera::IntrinsicBase * cam_J,
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
  const camera::IntrinsicBase * cam_I,
  const camera::IntrinsicBase * cam_J,
  const MapFeatOrRegionPerDesc& features_I,
  const MapFeatOrRegionPerDesc& features_J,
  const std::vector<feature::EImageDescriberType>& descTypes,
  MatT & x_I, MatT & x_J)
{
  // Create the output matrices with all matched features for images I and J
  const size_t n = putativeMatchesPerType.getNbAllMatches();
  x_I.resize(2, n);
  x_J.resize(2, n);

  size_t y = 0;
  for(size_t d = 0; d < descTypes.size(); ++d)
  {
    const feature::EImageDescriberType& descType = descTypes[d];

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
* @param[in] sfm_data SfMData scene container
* @param[in] regionsPerView Interface that provides the features positions
* @param[out] x_I Pixel perfect features from the Inth image putativeMatches matches
* @param[out] x_J Pixel perfect features from the Jnth image putativeMatches matches
*/
template<typename MatT >
void MatchesPairToMat(
  const Pair& pairIndex,
  const matching::MatchesPerDescType & putativeMatchesPerType,
  const sfm::SfMData * sfmData,
  const feature::RegionsPerView& regionsPerView,
  const std::vector<feature::EImageDescriberType>& descTypes,
  MatT & x_I, MatT & x_J)
{
  const sfm::View * view_I = sfmData->views.at(pairIndex.first).get();
  const sfm::View * view_J = sfmData->views.at(pairIndex.second).get();

  // Retrieve corresponding pair camera intrinsic if any
  const camera::IntrinsicBase * cam_I = sfmData->GetIntrinsicPtr(view_I->getIntrinsicId());
  const camera::IntrinsicBase * cam_J = sfmData->GetIntrinsicPtr(view_J->getIntrinsicId());

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
inline void copyInlierMatches(
  const std::vector<size_t>& inliers,
  const matching::MatchesPerDescType& putativeMatchesPerType,
  const std::vector<feature::EImageDescriberType>& descTypes,
  matching::MatchesPerDescType& out_geometricInliersPerType)
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

/**
 * @brief centerMatrix
 * @param[in] points2d
 * @param[out] c
 */
inline void centerMatrix(const Eigen::Matrix2Xf & points2d, Mat3 & c)
{
  c = Mat3::Identity();
  
  const Vec2f mean = points2d.rowwise().mean();
  const std::size_t nbPoints = points2d.cols();

  Vec2f stdDev = ((points2d.colwise() - mean).cwiseAbs2().rowwise().sum()/(nbPoints - 1)).cwiseSqrt();
  
  if(stdDev(0) < 0.1)
    stdDev(0) = 0.1;
  if(stdDev(1) < 0.1)
    stdDev(1) = 0.1;
  
  c << 1./stdDev(0), 0.,            -mean(0)/stdDev(0),
      0.,            1./stdDev(1),  -mean(1)/stdDev(1),
      0.,            0.,            1.;
}

/**
   * @brief Compute the matrices to get a centered and normalized the features.
   * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/relative_pose.h#L1075
   * @param[in] featuresI
   * @param[in] featuresJ
   * @param[in] matches Indicate which feature is concerned about the returned matrices.
   * @param[out] cI The matrix to apply to (the subpart of) \c featuresI
   * @param[out] cJ The matrix to apply to (the subpart of) \c featuresJ
   * @param[in] usefulMatchesId To consider a subpart of \c matches only.
   */
inline void centeringMatrices(const std::vector<feature::SIOPointFeature> & featuresI,
                       const std::vector<feature::SIOPointFeature> & featuresJ,
                       const matching::IndMatches & matches,
                       Mat3 & cI,
                       Mat3 & cJ,
                       const std::set<IndexT> & usefulMatchesId = std::set<IndexT>())
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault
  
  std::set<IndexT> matchesId = usefulMatchesId; // duplicate
  std::size_t nbMatches = usefulMatchesId.size();
  
  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }
  
  int iMatch = 0;
  Matf ptsI(2, nbMatches);
  Matf ptsJ(2, nbMatches);
  
  for (IndexT matchId : matchesId)
  {
    ptsI.col(iMatch) = featuresI.at(matches.at(matchId)._i).coords();
    ptsJ.col(iMatch) = featuresJ.at(matches.at(matchId)._j).coords();
    ++iMatch;
  }
  
  centerMatrix(ptsI, cI);
  centerMatrix(ptsJ, cJ);
}

/**
   * @brief Compute the similarity transformation between 2 features (using their scale & orientation).
   * Based on: https://github.com/fsrajer/yasfm/blob/3a09bc0ee69b7021910d646386cd92deab504a2c/YASFM/relative_pose.cpp#L1649
   * @param[in] feat1 The first feature with known scale & orientation.
   * @param[in] feat2 The second feature with known scale & orientation.
   * @param[out] S The similarity transformation between f1 et f2.
   */
inline void computeSimilarity(const feature::SIOPointFeature & feat1,
                       const feature::SIOPointFeature & feat2,
                       Mat3 & S)
{
  S = Mat3::Identity(); 
  
  const Vec2f & coord1 = feat1.coords();
  const double scale1 = feat1.scale();
  const double orientation1 = feat1.orientation();
  
  const Vec2f & coord2 = feat2.coords();
  const double scale2 =  feat2.scale();
  const double orientation2 = feat2.orientation();              
  
  const double c1 = cos(orientation1);
  const double s1 = sin(orientation1);
  const double c2 = cos(orientation2);
  const double s2 = sin(orientation2);
  
  Mat3 A1;
  A1 << scale1 * c1, scale1 * (-s1), coord1(0),
        scale1 * s1, scale1 * c1, coord1(1),
        0, 0, 1;
  Mat3 A2;
  A2 << scale2 * c2, scale2 * (-s2), coord2(0),
        scale2 * s2, scale2 * c2, coord2(1),
        0, 0, 1;
  
  S = A2 * A1.inverse();                               
}

/**
   * @brief Estimate (using SVD) the Affinity transformation from a set of matches.
   * Based on: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L1669
   * @param[in] featuresI 
   * @param[in] featuresJ
   * @param[in] matches The matches to consider for the estimation.
   * @param[out] affineTransformation The estimated Affine transformation.
   * @param[in] usefulMatchesId To consider a subpart of \c matches only. 
   */
inline void estimateAffinity(const std::vector<feature::SIOPointFeature> & featuresI,
                      const std::vector<feature::SIOPointFeature> & featuresJ,
                      const matching::IndMatches & matches,
                      Mat3 & affineTransformation,
                      const std::set<IndexT> & usefulMatchesId = std::set<IndexT>())
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault
  
  affineTransformation = Mat3::Identity();
  
  std::set<IndexT> matchesId = usefulMatchesId; // duplicate
  
  std::size_t nbMatches = usefulMatchesId.size();
  
  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }
  
  Mat M(Mat::Zero(2*nbMatches,6));
  Vec b(2*nbMatches);
  int iMatch = 0;
  for (IndexT matchId : matchesId)
  {
    const feature::SIOPointFeature & featI = featuresI.at(matches.at(matchId)._i);
    const feature::SIOPointFeature & featJ = featuresJ.at(matches.at(matchId)._j);
    const Vec2 featICoords (featI.x(), featI.y());
    
    M.block(iMatch,0,1,3) = featICoords.homogeneous().transpose();
    M.block(iMatch+nbMatches,3,1,3) = featICoords.homogeneous().transpose();
    b(iMatch) = featJ.x();
    b(iMatch+nbMatches) = featJ.y();
    
    ++iMatch;
  }
  
  const Vec a = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
  affineTransformation.row(0) = a.topRows(3).transpose();
  affineTransformation.row(1) = a.bottomRows(3).transpose();
  affineTransformation(2,0) = 0.;
  affineTransformation(2,1) = 0.;
  affineTransformation(2,2) = 1.;
}

/**
   * @brief estimateHomography Estimate (using SVD) the Homography transformation from a set of matches.
   * Based on: https://github.com/fsrajer/yasfm/blob/master/YASFM/relative_pose.cpp#L1694
   * @param[in] featuresI
   * @param[in] featuresJ
   * @param[in] matches The matches to consider for the estimation.
   * @param[out] H The estimated Homography transformation.
   * @param[in] usefulMatchesId To consider a subpart of \c matches only. 
   */
inline void estimateHomography(const std::vector<feature::SIOPointFeature> & featuresI,
                        const std::vector<feature::SIOPointFeature> & featuresJ,
                        const matching::IndMatches & matches,
                        Mat3 &H,
                        const std::set<IndexT> & usefulMatchesId = std::set<IndexT>())
{
  assert(!featuresI.empty());
  assert(!featuresJ.empty());
  assert(!matches.empty());
  assert(*std::max_element(usefulMatchesId.begin(), usefulMatchesId.end()) <= matches.size()); // prevent segfault
  
  H = Mat3::Identity();
  
  std::size_t nbMatches = usefulMatchesId.size();
  
  std::set<IndexT> matchesId = usefulMatchesId; // duplicate
  
  if (usefulMatchesId.empty())
  {
    nbMatches = matches.size();
    // set every match as useful for estimation
    for (IndexT i = 0; i < nbMatches; ++i)
      matchesId.insert(i);
  }
  
  Mat3 CI, CJ;
  centeringMatrices(featuresI, featuresJ, matches, CI, CJ, matchesId);
  
  Mat A(Mat::Zero(2*nbMatches,9));
  
  IndexT iMatch = 0;
  for(IndexT matchId : matchesId)
  {
    const feature::SIOPointFeature & featI = featuresI.at(matches.at(matchId)._i);
    const feature::SIOPointFeature & featJ = featuresJ.at(matches.at(matchId)._j);
    Vec2 fI(featI.x(), featI.y()); 
    Vec2 fJ(featJ.x(), featJ.y());
    Vec3 ptI = CI * fI.homogeneous();
    Vec3 ptJ = CJ * fJ.homogeneous();
    
    A.block(iMatch,0,1,3) = ptI.transpose();
    A.block(iMatch,6,1,3) = -ptJ(0) * ptI.transpose();
    A.block(iMatch+nbMatches,3,1,3) = ptI.transpose();
    A.block(iMatch+nbMatches,6,1,3) = -ptJ(1) * ptI.transpose();
    ++iMatch;
  }
  
  Eigen::JacobiSVD<Mat> svd(A, Eigen::ComputeThinU | Eigen::ComputeFullV);
  Vec h = svd.matrixV().rightCols(1);
  Mat3 H0;
  H0.row(0) = h.topRows(3).transpose();
  H0.row(1) = h.middleRows(3,3).transpose();
  H0.row(2) = h.bottomRows(3).transpose();
  
  H = CJ.inverse() * H0 * CI;
  if(std::fabs(H(2, 2)) > std::numeric_limits<double>::epsilon())
    H /= H(2,2);
}

/**
   * @brief Return the id. of the matches with a reprojection error < to the desirered \c tolerance.
   * @param[in] featuresI
   * @param[in] featuresJ
   * @param[in] matches The matches to test.
   * @param[in] transformation The 3x3 transformation matrix.
   * @param[in] tolerance The tolerated pixel error.
   * @param[in] inliersId The index in the \c matches vector.
   */
inline void findTransformationInliers(const std::vector<feature::SIOPointFeature> & featuresI, 
                               const std::vector<feature::SIOPointFeature> & featuresJ, 
                               const matching::IndMatches & matches,
                               const Mat3 & transformation,
                               const double tolerance,
                               std::set<IndexT> & inliersId)
{
  inliersId.clear();
  const double squaredTolerance = Square(tolerance);

#pragma omp parallel for 
  for (int iMatch = 0; iMatch < matches.size(); ++iMatch)
  {
    const feature::SIOPointFeature & featI = featuresI.at(matches.at(iMatch)._i);
    const feature::SIOPointFeature & featJ = featuresJ.at(matches.at(iMatch)._j);
    
    const Vec2 ptI(featI.x(), featI.y());
    const Vec2 ptJ(featJ.x(), featJ.y());
    
    const Vec3 ptIp_hom = transformation * ptI.homogeneous();
    
    const double dist = (ptJ - ptIp_hom.hnormalized()).squaredNorm();
    
    if (dist < squaredTolerance)
    {
#pragma omp critical
      inliersId.insert(iMatch);
    }
  }
}

} // namespace aliceVision
} // namespace matchingImageCollection
