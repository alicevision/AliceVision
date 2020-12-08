// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/IndMatchDecorator.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/matching/guidedMatching.hpp>
#include <aliceVision/multiview/relativePose/Homography4PSolver.hpp>
#include <aliceVision/multiview/relativePose/HomographyError.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief A contrario homography matrix estimation template functor used for filter pair of putative correspondences
 */
struct GeometricFilterMatrix_H_AC : public GeometricFilterMatrix
{
  GeometricFilterMatrix_H_AC(double dPrecision = std::numeric_limits<double>::infinity(),
                             std::size_t iteration = 1024)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_H(Mat3::Identity())
  {}

  /**
   * @brief Given two sets of image points, it estimates the homography matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<typename Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(const sfmData::SfMData* sfmData,
                                       const Regions_or_Features_ProviderT& regionsPerView,
                                       const Pair& pairIndex,
                                       const matching::MatchesPerDescType& putativeMatchesPerType,
                                       std::mt19937 &randomNumberGenerator,
                                       matching::MatchesPerDescType& out_geometricInliersPerType)
  {
    out_geometricInliersPerType.clear();

    // get back corresponding view index
    const IndexT I = pairIndex.first;
    const IndexT J = pairIndex.second;

    const sfmData::View& viewI = sfmData->getView(I);
    const sfmData::View& viewJ = sfmData->getView(J);

    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);

    if(descTypes.empty())
      return EstimationStatus(false, false);

    // retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    fillMatricesWithUndistortFeaturesMatches(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

    // define the AContrario adapted Homography matrix solver
    using KernelT = multiview::RelativePoseKernel<
                    multiview::relativePose::Homography4PSolver,
                    multiview::relativePose::HomographyAsymmetricError,
                    multiview::UnnormalizerI,
                    robustEstimation::Mat3Model>;

    const KernelT kernel(xI, viewI.getWidth(), viewI.getHeight(),
                         xJ, viewJ.getWidth(), viewJ.getHeight(), false); // configure as point to point error model.

    // robustly estimate the Homography matrix with A Contrario ransac
    const double upperBoundPrecision = Square(m_dPrecision);

    std::vector<std::size_t> inliers;
    robustEstimation::Mat3Model model;
    const std::pair<double,double> ACRansacOut = robustEstimation::ACRANSAC(kernel, randomNumberGenerator, inliers, m_stIteration, &model, upperBoundPrecision);
    m_H = model.getMatrix();

    if (inliers.empty())
      return EstimationStatus(false, false);

    m_dPrecision_robust = ACRansacOut.first;

    // fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(inliers,
                      putativeMatchesPerType,
                      descTypes,
                      out_geometricInliersPerType);

    // check if resection has strong support
    const bool hasStrongSupport = matching::hasStrongSupport(out_geometricInliersPerType, kernel.getMinimumNbRequiredSamples());

    return EstimationStatus(true, hasStrongSupport);
  }

  /**
   * @brief Export point feature based vector to a matrix [(x,y)'T, (x,y)'T].
   *        Use the camera intrinsics in order to get undistorted pixel coordinates
   */
  template<typename MatT>
  static void fillMatricesWithUndistortFeatures(const camera::IntrinsicBase* cam,
                                                const feature::PointFeatures& vec_feats,
                                                MatT& m)
  {
    using Scalar = typename MatT::Scalar; // output matrix type

    const bool hasValidIntrinsics = cam && cam->isValid();
    std::size_t i = 0;

    if(hasValidIntrinsics)
    {
      for(feature::PointFeatures::const_iterator iter = vec_feats.begin(); iter != vec_feats.end(); ++iter, ++i)
          m.col(i) = cam->get_ud_pixel(Vec2(iter->x(), iter->y()));
    }
    else
    {
      for(feature::PointFeatures::const_iterator iter = vec_feats.begin(); iter != vec_feats.end(); ++iter, ++i)
          m.col(i) = iter->coords().cast<Scalar>();
    }
  }

  template<typename MatT>
  static void createMatricesWithUndistortFeatures(const camera::IntrinsicBase* cam, const feature::MapRegionsPerDesc& regionsPerDesc, MatT& m)
  {
    std::size_t nbRegions = 0;
    for(const auto& regions: regionsPerDesc)
      nbRegions += regions.second->RegionCount();

    m.resize(2, nbRegions);

    std::size_t y = 0;
    for(const auto& regions: regionsPerDesc)
    {
      fillMatricesWithUndistortFeatures(cam, regions.second, m.block(0, y, 2, regions.second->RegionCount()));
      y += regions.second->RegionCount();
    }
  }

  template<typename MatT>
  static void createMatricesWithUndistortFeatures(const camera::IntrinsicBase* cam,
                                                  const feature::PointFeatures& vec_feats,
                                                  MatT& m)
  {
    std::size_t nbRegions = vec_feats.size();
    m.resize(2, nbRegions);
    fillMatricesWithUndistortFeatures(cam, vec_feats, m);
  }

  /**
   * @brief Geometry_guided_matching
   * @param sfm_data
   * @param regionsPerView
   * @param pairIndex
   * @param dDistanceRatio
   * @param matches
   * @return
   */
  bool Geometry_guided_matching(const sfmData::SfMData* sfmData,
                                const feature::RegionsPerView& regionsPerView,
                                const Pair imageIdsPair,
                                const double dDistanceRatio,
                                matching::MatchesPerDescType& matches) override
  {
    if(m_dPrecision_robust != std::numeric_limits<double>::infinity())
    {
      const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(imageIdsPair);

      if(descTypes.empty())
        return false;

      // get back corresponding view index
      const IndexT I = imageIdsPair.first;
      const IndexT J = imageIdsPair.second;

      const sfmData::View& viewI = sfmData->getView(I);
      const sfmData::View& viewJ = sfmData->getView(J);

      // retrieve corresponding pair camera intrinsic if any
      const camera::IntrinsicBase* camI = sfmData->getIntrinsics().count(viewI.getIntrinsicId()) ?
                                          sfmData->getIntrinsics().at(viewI.getIntrinsicId()).get() : nullptr;
      const camera::IntrinsicBase* camJ = sfmData->getIntrinsics().count(viewJ.getIntrinsicId()) ?
                                          sfmData->getIntrinsics().at(viewJ.getIntrinsicId()).get() : nullptr;

      robustEstimation::Mat3Model model(m_H);

      if(dDistanceRatio < 0)
      {
        for(const feature::EImageDescriberType descType : descTypes)
        {
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          matching::IndMatches localMatches;

          const feature::Regions& regionsI = regionsPerView.getRegions(I, descType);
          const feature::Regions& regionsJ = regionsPerView.getRegions(J, descType);
          const feature::PointFeatures pointsFeaturesI = regionsI.GetRegionsPositions();
          const feature::PointFeatures pointsFeaturesJ = regionsJ.GetRegionsPositions();

          // filtering based only on region positions
          Mat xI, xJ;
          createMatricesWithUndistortFeatures(camI, pointsFeaturesI, xI);
          createMatricesWithUndistortFeatures(camJ, pointsFeaturesJ, xJ);

          matching::guidedMatching<robustEstimation::Mat3Model, multiview::relativePose::HomographyAsymmetricError>(model, xI, xJ, Square(m_dPrecision_robust), localMatches);

          // remove matches that have the same (X,Y) coordinates
          matching::IndMatchDecorator<float> matchDeduplicator(localMatches, pointsFeaturesI, pointsFeaturesJ);
          matchDeduplicator.getDeduplicated(localMatches);
          matches[descType] = localMatches;
        }
      }
      else
      {
        // filtering based on region positions and regions descriptors
        matching::guidedMatching<robustEstimation::Mat3Model, multiview::relativePose::HomographyAsymmetricError>(model,
                                                                 camI, regionsPerView.getAllRegions(I),
                                                                 camJ, regionsPerView.getAllRegions(J),
                                                                 Square(m_dPrecision_robust), Square(dDistanceRatio),
                                                                 matches);
      }
    }

    return matches.getNbAllMatches() != 0;
  }

  // stored data
  Mat3 m_H;
};

} // namespace matchingImageCollection
} // namespace aliceVision


