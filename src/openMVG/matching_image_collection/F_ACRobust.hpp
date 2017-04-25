// Copyright (c) 2012, 2013, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/multiview/solver_fundamental_kernel.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/robust_estimation/robust_estimators.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/robust_estimation/robust_estimator_LORansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_LORansacKernelAdaptor.hpp"
#include "openMVG/robust_estimation/score_evaluator.hpp"
#include "openMVG/robust_estimation/guided_matching.hpp"

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/features/RegionsPerView.hpp"
#include "openMVG/matching_image_collection/Geometric_Filter_utils.hpp"


namespace openMVG {
namespace matching_image_collection {

//-- A contrario fundamental matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilter_FMatrix_AC
{
  GeometricFilter_FMatrix_AC(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : m_dPrecision(dPrecision), m_stIteration(iteration), m_F(Mat3::Identity()),
      m_dPrecision_robust(std::numeric_limits<double>::infinity()){};

  /**
   * @brief Given two sets of image points, it estimates the fundamental matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<typename Regions_or_Features_ProviderT>
  bool Robust_estimation(
    const sfm::SfM_Data * sfmData,
    const Regions_or_Features_ProviderT & regionsPerView,
    const Pair pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & geometricInliersPerType)
  {
    using namespace openMVG;
    using namespace openMVG::robust;
    geometricInliersPerType.clear();

    // Get back corresponding view index
    const IndexT iIndex = pairIndex.first;
    const IndexT jIndex = pairIndex.second;

    const std::vector<features::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return false;

    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    MatchesPairToMat(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

    std::vector<size_t> inliers;
    bool valid = Robust_estimation(
        xI, xJ,
        std::make_pair(sfmData->GetViews().at(iIndex)->ui_width, sfmData->GetViews().at(iIndex)->ui_height),
        std::make_pair(sfmData->GetViews().at(jIndex)->ui_width, sfmData->GetViews().at(jIndex)->ui_height),
        inliers);

    if (!valid)
      return false;

    std::sort(inliers.begin(), inliers.end());
    // Fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(
          inliers,
          putativeMatchesPerType,
          descTypes,
          geometricInliersPerType);

    return true;
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
   * @return true if the estimated fundamental matrix is supported by enough points,
   * namely if there are at least KernelType::MINIMUM_SAMPLES *2.5 points supporting
   * the estimated fundamental matrix 
   */
  bool Robust_estimation(
    const Mat& xI,       // points of the first image
    const Mat& xJ,       // points of the second image
    const std::pair<size_t,size_t> & imageSizeI,     // size of the first image  
    const std::pair<size_t,size_t> & imageSizeJ,     // size of the first image
    std::vector<size_t> &vec_inliers,
    robust::EROBUST_ESTIMATOR estimator = robust::ROBUST_ESTIMATOR_ACRANSAC)
  {
    using namespace openMVG;
    using namespace openMVG::robust;
    vec_inliers.clear();

    switch(estimator)
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
          ACRANSAC(kernel, vec_inliers, m_stIteration, &m_F, upper_bound_precision);

        bool valid = ( (vec_inliers.size() > KernelType::MINIMUM_SAMPLES * OPENMVG_MINIMUM_SAMPLES_COEF) );

        // if the estimation has enough support set its precision
        if(valid) m_dPrecision_robust = ACRansacOut.first;

        return valid;
      }
      case ROBUST_ESTIMATOR_LORANSAC:
      {
        // just a safeguard
        if(m_dPrecision == std::numeric_limits<double>::infinity())
        {
          throw std::invalid_argument("[GeometricFilter_FMatrix_AC::Robust_estimation] the threshold of the LORANSAC is set to infinity!");
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

        m_F = LO_RANSAC(kernel, scorer, &vec_inliers);

        bool valid = ( (vec_inliers.size() > KernelType::MINIMUM_SAMPLES * OPENMVG_MINIMUM_SAMPLES_COEF) );

        // for LORansac the robust precision is the same as the threshold
        if(valid) m_dPrecision_robust = m_dPrecision;

        return valid;
      }
    default:
      throw std::runtime_error("[GeometricFilter_FMatrix_AC::Robust_estimation] only ACRansac and LORansac are supported!");
    }
    return false;
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
    matching::MatchesPerDescType & matches)
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
  
  double m_dPrecision;  //upper_bound precision used for robust estimation
  size_t m_stIteration; //maximal number of iteration for robust estimation
  //
  //-- Stored data
  Mat3 m_F;
  double m_dPrecision_robust;
};

} // namespace openMVG
} //namespace matching_image_collection
