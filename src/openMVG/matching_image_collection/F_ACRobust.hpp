// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/matching_image_collection/GeometricFilterMatrix.hpp"
#include "openMVG/matching_image_collection/Geometric_Filter_utils.hpp"
#include "openMVG/matching/indMatch.hpp"
#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/robust_estimation/robust_estimators.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/robust_estimation/robust_estimator_LORansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_LORansacKernelAdaptor.hpp"
#include "openMVG/robust_estimation/score_evaluator.hpp"
#include "openMVG/robust_estimation/guided_matching.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/features/RegionsPerView.hpp"

namespace openMVG {
namespace matching_image_collection {

//-- A contrario fundamental matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilter_FMatrix: public GeometricFilterMatrix
{
  GeometricFilter_FMatrix(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024,
    robust::EROBUST_ESTIMATOR estimator = robust::ROBUST_ESTIMATOR_ACRANSAC)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_F(Mat3::Identity())
    , m_estimator(estimator)
  {}

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<class Regions_or_Features_ProviderT>
  EstimationState geometricEstimation(
    const sfm::SfM_Data * sfmData,
    const Regions_or_Features_ProviderT & regionsPerView,
    const Pair pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace openMVG;
    using namespace openMVG::robust;
    out_geometricInliersPerType.clear();

    // Get back corresponding view index
    const IndexT iIndex = pairIndex.first;
    const IndexT jIndex = pairIndex.second;

    const sfm::View * view_I = sfmData->views.at(iIndex).get();
    const sfm::View * view_J = sfmData->views.at(jIndex).get();

    const cameras::IntrinsicBase * cam_I = sfmData->GetIntrinsicPtr(view_I->id_intrinsic);
    const cameras::IntrinsicBase * cam_J = sfmData->GetIntrinsicPtr(view_J->id_intrinsic);

    const std::pair<size_t,size_t> imageSizeI(sfmData->GetViews().at(iIndex)->ui_width, sfmData->GetViews().at(iIndex)->ui_height);
    const std::pair<size_t,size_t> imageSizeJ(sfmData->GetViews().at(jIndex)->ui_width, sfmData->GetViews().at(jIndex)->ui_height);

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
  EstimationState geometricEstimation(
      const MapFeatOrRegionsPerDesc& region_I,
      const MapFeatOrRegionsPerDesc& region_J,
      const cameras::IntrinsicBase * cam_I,
      const cameras::IntrinsicBase * cam_J,
      const std::pair<size_t,size_t> & imageSizeI,     // size of the first image
      const std::pair<size_t,size_t> & imageSizeJ,     // size of the first image
      const matching::MatchesPerDescType & putativeMatchesPerType,
      matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace openMVG;
    using namespace openMVG::robust;
    out_geometricInliersPerType.clear();

    const std::vector<features::EImageDescriberType> descTypes = getCommonDescTypes(region_I, region_J);

    if(descTypes.empty())
      return EstimationState(false, false);

    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    MatchesPairToMat(putativeMatchesPerType, cam_I, cam_J,
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
      return EstimationState(false, false);
    }

    // Fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(
          inliers,
          putativeMatchesPerType,
          descTypes,
          out_geometricInliersPerType);

    // If matches has strong support
    const bool hasStrongSupport = robust::hasStrongSupport(out_geometricInliersPerType, estimationPair.second);

    return EstimationState(true, hasStrongSupport);
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
    using namespace openMVG;
    using namespace openMVG::robust;
    out_inliers.clear();

    switch(m_estimator)
    {
      case ROBUST_ESTIMATOR_ACRANSAC:
      {
        // Define the AContrario adapted Fundamental matrix solver
        typedef ACKernelAdaptor<
          openMVG::fundamental::kernel::SevenPointSolver,
          openMVG::fundamental::kernel::SimpleError,
          //openMVG::fundamental::kernel::SymmetricEpipolarDistanceError,
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
      case ROBUST_ESTIMATOR_LORANSAC:
      {
        // just a safeguard
        if(m_dPrecision == std::numeric_limits<double>::infinity())
        {
          throw std::invalid_argument("[GeometricFilter_FMatrix_AC::geometricEstimation] the threshold of the LORANSAC is set to infinity!");
        }

        typedef KernelAdaptorLoRansac<
                openMVG::fundamental::kernel::SevenPointSolver,
                openMVG::fundamental::kernel::SymmetricEpipolarDistanceError,
                UnnormalizerT,
                Mat3,
                openMVG::fundamental::kernel::EightPointSolver>
                KernelType;

        const KernelType kernel(xI, imageSizeI.first, imageSizeI.second,
                                xJ, imageSizeJ.first, imageSizeJ.second, true);

        //@fixme scorer should be using the pixel error, not the squared version, refactoring needed
        const double normalizedThreshold = Square(m_dPrecision * kernel.normalizer2()(0, 0));
        ScorerEvaluator<KernelType> scorer(normalizedThreshold);

        m_F = LO_RANSAC(kernel, scorer, &out_inliers);

        if(out_inliers.empty())
          return std::make_pair(false, KernelType::MINIMUM_SAMPLES);

        m_dPrecision_robust = m_dPrecision;

        return std::make_pair(true, KernelType::MINIMUM_SAMPLES);
      }
    default:
      throw std::runtime_error("[GeometricFilter_FMatrix_AC::geometricEstimation] only ACRansac and LORansac are supported!");
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
    const sfm::SfM_Data * sfmData,
    const features::RegionsPerView& regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches) override
  {
    if (m_dPrecision_robust != std::numeric_limits<double>::infinity())
    {
      // Get back corresponding view index
      const IndexT viewId_I = imageIdsPair.first;
      const IndexT viewId_J = imageIdsPair.second;

      const sfm::View * view_I = sfmData->views.at(viewId_I).get();
      const sfm::View * view_J = sfmData->views.at(viewId_J).get();

      // Retrieve corresponding pair camera intrinsic if any
      const cameras::IntrinsicBase * cam_I =
        sfmData->GetIntrinsics().count(view_I->id_intrinsic) ?
          sfmData->GetIntrinsics().at(view_I->id_intrinsic).get() : nullptr;
      const cameras::IntrinsicBase * cam_J =
        sfmData->GetIntrinsics().count(view_J->id_intrinsic) ?
          sfmData->GetIntrinsics().at(view_J->id_intrinsic).get() : nullptr;

      // Check the features correspondences that agree in the geometric and photometric domain
      geometry_aware::GuidedMatching<Mat3,
                                     fundamental::kernel::EpipolarDistanceError>(
        m_F,
        cam_I, // cameras::IntrinsicBase
        regionsPerView.getAllRegions(viewId_I), // features::Regions
        cam_J, // cameras::IntrinsicBase
        regionsPerView.getAllRegions(viewId_J), // features::Regions
        Square(m_dPrecision_robust), Square(dDistanceRatio),
        matches);
    }
    return matches.getNbAllMatches() != 0;
  }
  
  //
  //-- Stored data
  Mat3 m_F;
  robust::EROBUST_ESTIMATOR m_estimator;
};

} // namespace matching_image_collection
} // namespace openMVG
