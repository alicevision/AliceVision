// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"
#include "aliceVision/matchingImageCollection/geometricFilterUtils.hpp"
#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/multiview/fundamentalKernelSolver.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/robustEstimation/estimators.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"
#include "aliceVision/robustEstimation/LORansac.hpp"
#include "aliceVision/robustEstimation/LORansacKernelAdaptor.hpp"
#include "aliceVision/robustEstimation/ScoreEvaluator.hpp"
#include "aliceVision/robustEstimation/guidedMatching.hpp"
#include "aliceVision/sfmData/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"

namespace aliceVision {
namespace matchingImageCollection {

//-- A contrario fundamental matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilterMatrix_F_AC: public GeometricFilterMatrix
{
  GeometricFilterMatrix_F_AC(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024,
    robustEstimation::ERobustEstimator estimator = robustEstimation::ERobustEstimator::ACRANSAC)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_F(Mat3::Identity())
    , m_estimator(estimator)
  {}

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<class Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(
    const sfmData::SfMData * sfmData,
    const Regions_or_Features_ProviderT & regionsPerView,
    const Pair& pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace aliceVision;
    using namespace aliceVision::robustEstimation;
    out_geometricInliersPerType.clear();

    // Get back corresponding view index
    const IndexT iIndex = pairIndex.first;
    const IndexT jIndex = pairIndex.second;

    const sfmData::View * view_I = sfmData->views.at(iIndex).get();
    const sfmData::View * view_J = sfmData->views.at(jIndex).get();

    const camera::IntrinsicBase * cam_I = sfmData->getIntrinsicPtr(view_I->getIntrinsicId());
    const camera::IntrinsicBase * cam_J = sfmData->getIntrinsicPtr(view_J->getIntrinsicId());

    const std::pair<size_t,size_t> imageSizeI(sfmData->getViews().at(iIndex)->getWidth(), sfmData->getViews().at(iIndex)->getHeight());
    const std::pair<size_t,size_t> imageSizeJ(sfmData->getViews().at(jIndex)->getWidth(), sfmData->getViews().at(jIndex)->getHeight());

    return geometricEstimation(
        regionsPerView.getDataPerDesc(pairIndex.first), regionsPerView.getDataPerDesc(pairIndex.second),
        cam_I, cam_J,
        imageSizeI, imageSizeJ,
        putativeMatchesPerType,
        out_geometricInliersPerType);
  }

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<class MapFeatOrRegionsPerDesc>
  EstimationStatus geometricEstimation(
      const MapFeatOrRegionsPerDesc& region_I,
      const MapFeatOrRegionsPerDesc& region_J,
      const camera::IntrinsicBase * cam_I,
      const camera::IntrinsicBase * cam_J,
      const std::pair<size_t,size_t> & imageSizeI,     // size of the first image
      const std::pair<size_t,size_t> & imageSizeJ,     // size of the first image
      const matching::MatchesPerDescType & putativeMatchesPerType,
      matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace aliceVision;
    using namespace aliceVision::robustEstimation;
    out_geometricInliersPerType.clear();

    const std::vector<feature::EImageDescriberType> descTypes = getCommonDescTypes(region_I, region_J);

    if(descTypes.empty())
      return EstimationStatus(false, false);

    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    fillMatricesWithUndistortFeaturesMatches(putativeMatchesPerType, cam_I, cam_J,
                     region_I, region_J,
                     descTypes, xI, xJ);
    std::vector<size_t> inliers;

    std::pair<bool, std::size_t> estimationPair = geometricEstimation_Mat(
        xI, xJ,
        imageSizeI,
        imageSizeJ,
        inliers);

    if (!estimationPair.first) // estimation is not valid
    {
      assert(inliers.empty());
      return EstimationStatus(false, false);
    }

    // Fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(
          inliers,
          putativeMatchesPerType,
          descTypes,
          out_geometricInliersPerType);

    // If matches has strong support
    const bool hasStrongSupport = robustEstimation::hasStrongSupport(out_geometricInliersPerType, estimationPair.second);

    return EstimationStatus(true, hasStrongSupport);
  }

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   * 
   * @param[in] xI The first set of points
   * @param[in] xJ The second set of points
   * @param[in] imageSizeI The size of the first image (used for normalizing the points)
   * @param[in] imageSizeJ The size of the second image
   * @param[out] geometric_inliers A vector containing the indices of the inliers
   * @return true if geometric_inliers is not empty
   */
  std::pair<bool, std::size_t> geometricEstimation_Mat(
    const Mat& xI,       // points of the first image
    const Mat& xJ,       // points of the second image
    const std::pair<size_t,size_t> & imageSizeI,     // size of the first image  
    const std::pair<size_t,size_t> & imageSizeJ,     // size of the first image
    std::vector<size_t> & out_inliers)
  {
    using namespace aliceVision;
    using namespace aliceVision::robustEstimation;
    out_inliers.clear();

    switch(m_estimator)
    {
      case ERobustEstimator::ACRANSAC:
      {
        // Define the AContrario adapted Fundamental matrix solver
        typedef ACKernelAdaptor<
          aliceVision::fundamental::kernel::SevenPointSolver,
          aliceVision::fundamental::kernel::SimpleError,
          //aliceVision::fundamental::kernel::SymmetricEpipolarDistanceError,
          UnnormalizerT,
          Mat3>
          KernelType;

        const KernelType kernel(
          xI, imageSizeI.first, imageSizeI.second,
          xJ, imageSizeJ.first, imageSizeJ.second, true);

        // Robustly estimate the Fundamental matrix with A Contrario ransac
        const double upper_bound_precision = Square(m_dPrecision);
        const std::pair<double,double> ACRansacOut =
          ACRANSAC(kernel, out_inliers, m_stIteration, &m_F, upper_bound_precision);

        if(out_inliers.empty())
          return std::make_pair(false, KernelType::MINIMUM_SAMPLES);

        m_dPrecision_robust = ACRansacOut.first;

        return std::make_pair(true, KernelType::MINIMUM_SAMPLES);
      }
      case ERobustEstimator::LORANSAC:
      {
        // just a safeguard
        if(m_dPrecision == std::numeric_limits<double>::infinity())
        {
          throw std::invalid_argument("[GeometricFilterMatrix_F_AC_AC::geometricEstimation] the threshold of the LORANSAC is set to infinity!");
        }

        typedef KernelAdaptorLoRansac<
                aliceVision::fundamental::kernel::SevenPointSolver,
                aliceVision::fundamental::kernel::SymmetricEpipolarDistanceError,
                UnnormalizerT,
                Mat3,
                aliceVision::fundamental::kernel::EightPointSolver>
                KernelType;

        const KernelType kernel(xI, imageSizeI.first, imageSizeI.second,
                                xJ, imageSizeJ.first, imageSizeJ.second, true);

        //@fixme scorer should be using the pixel error, not the squared version, refactoring needed
        const double normalizedThreshold = Square(m_dPrecision * kernel.normalizer2()(0, 0));
        ScoreEvaluator<KernelType> scorer(normalizedThreshold);

        m_F = LO_RANSAC(kernel, scorer, &out_inliers);

        if(out_inliers.empty())
          return std::make_pair(false, KernelType::MINIMUM_SAMPLES);

        m_dPrecision_robust = m_dPrecision;

        return std::make_pair(true, KernelType::MINIMUM_SAMPLES);
      }
    default:
      throw std::runtime_error("[GeometricFilterMatrix_F_AC_AC::geometricEstimation] only ACRansac and LORansac are supported!");
    }
    return std::make_pair(false, 0);;
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
  bool Geometry_guided_matching(
    const sfmData::SfMData * sfmData,
    const feature::RegionsPerView& regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches) override
  {
    if (m_dPrecision_robust != std::numeric_limits<double>::infinity())
    {
      // Get back corresponding view index
      const IndexT viewId_I = imageIdsPair.first;
      const IndexT viewId_J = imageIdsPair.second;

      const sfmData::View * view_I = sfmData->views.at(viewId_I).get();
      const sfmData::View * view_J = sfmData->views.at(viewId_J).get();

      // Retrieve corresponding pair camera intrinsic if any
      const camera::IntrinsicBase * cam_I =
        sfmData->getIntrinsics().count(view_I->getIntrinsicId()) ?
          sfmData->getIntrinsics().at(view_I->getIntrinsicId()).get() : nullptr;
      const camera::IntrinsicBase * cam_J =
        sfmData->getIntrinsics().count(view_J->getIntrinsicId()) ?
          sfmData->getIntrinsics().at(view_J->getIntrinsicId()).get() : nullptr;

      // Check the features correspondences that agree in the geometric and photometric domain
      robustEstimation::GuidedMatching<Mat3,
                                     fundamental::kernel::EpipolarDistanceError>(
        m_F,
        cam_I, // camera::IntrinsicBase
        regionsPerView.getAllRegions(viewId_I), // feature::Regions
        cam_J, // camera::IntrinsicBase
        regionsPerView.getAllRegions(viewId_J), // feature::Regions
        Square(m_dPrecision_robust), Square(dDistanceRatio),
        matches);
    }
    return matches.getNbAllMatches() != 0;
  }
  
  //
  //-- Stored data
  Mat3 m_F;
  robustEstimation::ERobustEstimator m_estimator;
};

} // namespace matchingImageCollection
} // namespace aliceVision
