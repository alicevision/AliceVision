// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp>
#include <aliceVision/matchingImageCollection/geometricFilterUtils.hpp>
#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>
#include <aliceVision/robustEstimation/guidedMatching.hpp>
#include <aliceVision/robustEstimation/supportEstimation.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/multiview/relativePose/Fundamental7PSolver.hpp>
#include <aliceVision/multiview/relativePose/Fundamental8PSolver.hpp>
#include <aliceVision/multiview/relativePose/Fundamental10PSolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/feature/RegionsPerView.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace matchingImageCollection {

/**
 * @brief A contrario fundamental matrix estimation template functor used for filter pair of putative correspondences
 */
struct GeometricFilterMatrix_F_AC : public GeometricFilterMatrix
{
  GeometricFilterMatrix_F_AC(double dPrecision = std::numeric_limits<double>::infinity(),
                             std::size_t iteration = 1024,
                             robustEstimation::ERobustEstimator estimator = robustEstimation::ERobustEstimator::ACRANSAC,
                             bool estimateDistortion = false)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_F(Mat3::Identity())
    , m_estimator(estimator)
    , m_estimateDistortion(estimateDistortion)
  {}

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<class Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(const sfmData::SfMData* sfmData,
                                       const Regions_or_Features_ProviderT& regionsPerView,
                                       const Pair& pairIndex,
                                       const matching::MatchesPerDescType& putativeMatchesPerType,
                                       matching::MatchesPerDescType& out_geometricInliersPerType)
  {
    out_geometricInliersPerType.clear();

    // get back corresponding view index
    const IndexT I = pairIndex.first;
    const IndexT J = pairIndex.second;

    const sfmData::View& viewI = sfmData->getView(I);
    const sfmData::View& viewJ = sfmData->getView(J);

    const camera::IntrinsicBase* camI = sfmData->getIntrinsicPtr(viewI.getIntrinsicId());
    const camera::IntrinsicBase* camJ = sfmData->getIntrinsicPtr(viewJ.getIntrinsicId());

    const std::pair<std::size_t, std::size_t> imageSizeI(viewI.getWidth(), viewI.getHeight());
    const std::pair<std::size_t, std::size_t> imageSizeJ(viewJ.getWidth(), viewJ.getHeight());

    return geometricEstimation(regionsPerView.getDataPerDesc(pairIndex.first),
                               regionsPerView.getDataPerDesc(pairIndex.second),
                               camI, camJ,
                               imageSizeI, imageSizeJ,
                               putativeMatchesPerType,
                               out_geometricInliersPerType);
  }

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<class MapFeatOrRegionsPerDesc>
  EstimationStatus geometricEstimation(const MapFeatOrRegionsPerDesc& regionI,
                                       const MapFeatOrRegionsPerDesc& regionJ,
                                       const camera::IntrinsicBase* camI,
                                       const camera::IntrinsicBase* camJ,
                                       const std::pair<std::size_t, std::size_t>& imageSizeI, // size of the first image
                                       const std::pair<std::size_t, std::size_t>& imageSizeJ, // size of the second image
                                       const matching::MatchesPerDescType& putativeMatchesPerType,
                                       matching::MatchesPerDescType& out_geometricInliersPerType)
  {
    out_geometricInliersPerType.clear();

    const std::vector<feature::EImageDescriberType> descTypes = getCommonDescTypes(regionI, regionJ);

    if(descTypes.empty())
      return EstimationStatus(false, false);

    // retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    fillMatricesWithUndistortFeaturesMatches(putativeMatchesPerType, camI, camJ,
                                             regionI, regionJ,
                                             descTypes, xI, xJ);

    std::vector<std::size_t> inliers;
    std::pair<bool, std::size_t> estimationPair;

    switch(m_estimator)
    {
      case robustEstimation::ERobustEstimator::ACRANSAC:
      {
        if(m_estimateDistortion)
        {
          estimationPair = geometricEstimation_Mat_ACRANSAC<multiview::relativePose::Fundamental10PSolver, multiview::relativePose::Fundamental10PModel>(xI, xJ, imageSizeI, imageSizeJ, inliers);
        }
        else
          estimationPair = geometricEstimation_Mat_ACRANSAC<multiview::relativePose::Fundamental7PSolver, robustEstimation::Mat3Model>(xI, xJ, imageSizeI, imageSizeJ, inliers);
      }
      break;
      case robustEstimation::ERobustEstimator::LORANSAC:
      {
        if(m_estimateDistortion)
          throw std::invalid_argument("[GeometricFilterMatrix_F_AC::geometricEstimation] Using fundamental matrix and f10 solver with LO_RANSAC is not yet implemented");

        estimationPair = geometricEstimation_Mat_LORANSAC<multiview::relativePose::Fundamental7PSolver, multiview::relativePose::Fundamental8PSolver>(xI, xJ, imageSizeI, imageSizeJ, inliers);
      }
      break;

      default:
        throw std::runtime_error("[GeometricFilterMatrix_F_AC::geometricEstimation] only ACRansac and LORansac are supported!");
    }

    if(!estimationPair.first) // estimation is not valid
    {
      assert(inliers.empty());
      return EstimationStatus(false, false);
    }

    // fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(inliers, putativeMatchesPerType, descTypes, out_geometricInliersPerType);

    // if matches has strong support
    const bool hasStrongSupport = robustEstimation::hasStrongSupport(out_geometricInliersPerType, estimationPair.second);

    return EstimationStatus(true, hasStrongSupport);
  }

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   *        For ACRANSAC estimator
   * @param[in] xI The first set of points
   * @param[in] xJ The second set of points
   * @param[in] imageSizeI The size of the first image (used for normalizing the points)
   * @param[in] imageSizeJ The size of the second image
   * @param[out] geometric_inliers A vector containing the indices of the inliers
   * @return true if geometric_inliers is not empty
   */
  template<class SolverT_, class ModelT_>
  std::pair<bool, std::size_t> geometricEstimation_Mat_ACRANSAC(const Mat& xI, // points of the first image
                                                                const Mat& xJ, // points of the second image
                                                                const std::pair<std::size_t, std::size_t>& imageSizeI, // size of the first image
                                                                const std::pair<std::size_t, std::size_t>& imageSizeJ, // size of the first image
                                                                std::vector<std::size_t>& out_inliers)
  {
    out_inliers.clear();

    // define the AContrario adapted Fundamental matrix solver
    typedef multiview::RelativePoseKernel<
      SolverT_,
      multiview::relativePose::FundamentalEpipolarDistanceError,
      //multiview::relativePose::FundamentalSymmetricEpipolarDistanceError,
      multiview::UnnormalizerT,
      ModelT_>
      KernelT;

    const KernelT kernel(xI, imageSizeI.first, imageSizeI.second,
                         xJ, imageSizeJ.first, imageSizeJ.second, true);

    // robustly estimate the Fundamental matrix with A Contrario ransac
    const double upperBoundPrecision = Square(m_dPrecision);

    ModelT_ model;
    const std::pair<double,double> ACRansacOut = robustEstimation::ACRANSAC(kernel, out_inliers, m_stIteration, &model, upperBoundPrecision);
    m_F = model.getMatrix();

    if(out_inliers.empty())
      return std::make_pair(false, kernel.getMinimumNbRequiredSamples());

    m_dPrecision_robust = ACRansacOut.first;

    return std::make_pair(true, kernel.getMinimumNbRequiredSamples());
  }

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   *        For LORANSAC estimator
   * @param[in] xI The first set of points
   * @param[in] xJ The second set of points
   * @param[in] imageSizeI The size of the first image (used for normalizing the points)
   * @param[in] imageSizeJ The size of the second image
   * @param[out] geometric_inliers A vector containing the indices of the inliers
   * @return true if geometric_inliers is not empty
   */
  template<class SolverT_, class SolverLsT_>
  std::pair<bool, std::size_t> geometricEstimation_Mat_LORANSAC(const Mat& xI, // points of the first image
                                                                const Mat& xJ, // points of the second image
                                                                const std::pair<std::size_t, std::size_t>& imageSizeI, // size of the first image
                                                                const std::pair<std::size_t, std::size_t>& imageSizeJ, // size of the first image
                                                                std::vector<std::size_t>& out_inliers)
  {
    out_inliers.clear();

    // just a safeguard
    if(m_dPrecision == std::numeric_limits<double>::infinity())
      throw std::invalid_argument("[GeometricFilterMatrix_F_AC_AC::geometricEstimation] the threshold of the LORANSAC is set to infinity!");

    typedef multiview::RelativePoseKernel<
            SolverT_,
            multiview::relativePose::FundamentalSymmetricEpipolarDistanceError,
            multiview::UnnormalizerT,
            robustEstimation::Mat3Model,
            SolverLsT_>
            KernelT;

    const KernelT kernel(xI, imageSizeI.first, imageSizeI.second,
                         xJ, imageSizeJ.first, imageSizeJ.second, true);

    //@fixme scorer should be using the pixel error, not the squared version, refactoring needed
    const double normalizedThreshold = Square(m_dPrecision * kernel.normalizer2()(0, 0));
    robustEstimation::ScoreEvaluator<KernelT> scorer(normalizedThreshold);

    robustEstimation::Mat3Model model = robustEstimation::LO_RANSAC(kernel, scorer, &out_inliers);
    m_F = model.getMatrix();

    if(out_inliers.empty())
      return std::make_pair(false, kernel.getMinimumNbRequiredSamples());

    m_dPrecision_robust = m_dPrecision;

    return std::make_pair(true, kernel.getMinimumNbRequiredSamples());
  }

  /**
   * @brief Geometry_guided_matching
   * @param sfmData
   * @param regionsPerView
   * @param imagesPair
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
    if (m_dPrecision_robust != std::numeric_limits<double>::infinity())
    {
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

      robustEstimation::Mat3Model model(m_F);

      // check the features correspondences that agree in the geometric and photometric domain
      robustEstimation::guidedMatching<robustEstimation::Mat3Model, multiview::relativePose::FundamentalEpipolarDistanceError>(
        model,
        camI,                            // camera::IntrinsicBase
        regionsPerView.getAllRegions(I), // feature::Regions
        camJ,                            // camera::IntrinsicBase
        regionsPerView.getAllRegions(J), // feature::Regions
        Square(m_dPrecision_robust), Square(dDistanceRatio),
        matches);
    }
    return matches.getNbAllMatches() != 0;
  }
  

  // Stored data

  Mat3 m_F;
  robustEstimation::ERobustEstimator m_estimator;
  bool m_estimateDistortion;
};

} // namespace matchingImageCollection
} // namespace aliceVision
