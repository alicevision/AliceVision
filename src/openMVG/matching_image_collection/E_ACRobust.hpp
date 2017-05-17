
// Copyright (c) 2012, 2013, 2014, 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#pragma once

#include "openMVG/types.hpp"
#include "openMVG/multiview/solver_essential_kernel.hpp"
#include "openMVG/multiview/essential.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansac.hpp"
#include "openMVG/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "openMVG/robust_estimation/guided_matching.hpp"
#include <limits>

#include "openMVG/matching/indMatch.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/features/RegionsPerView.hpp"
#include "openMVG/matching_image_collection/Geometric_Filter_utils.hpp"

namespace openMVG {
namespace matching_image_collection {

//-- A contrario essential matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilter_EMatrix_AC
{
  GeometricFilter_EMatrix_AC(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : m_dPrecision(dPrecision), m_stIteration(iteration), m_E(Mat3::Identity()),
      m_dPrecision_robust(std::numeric_limits<double>::infinity())
  {}

  /**
   * @brief Given two sets of image points, it estimates the essential matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<typename Regions_or_Features_ProviderT>
  bool geometricEstimation(
    const sfm::SfM_Data * sfmData,
    const Regions_or_Features_ProviderT& regionsPerView,
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

    // Reject pair with missing Intrinsic information
    const sfm::View * view_I = sfmData->views.at(iIndex).get();
    const sfm::View * view_J = sfmData->views.at(jIndex).get();

    // Check that valid cameras can be retrieved for the pair of views
    const cameras::IntrinsicBase * cam_I =
      sfmData->GetIntrinsics().count(view_I->id_intrinsic) ?
        sfmData->GetIntrinsics().at(view_I->id_intrinsic).get() : nullptr;
    const cameras::IntrinsicBase * cam_J =
      sfmData->GetIntrinsics().count(view_J->id_intrinsic) ?
        sfmData->GetIntrinsics().at(view_J->id_intrinsic).get() : nullptr;

    if (!cam_I || !cam_J)
      return false;
    if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
      return false;

    // Get corresponding point regions arrays
    Mat xI,xJ;
    MatchesPairToMat(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

    // Define the AContrario adapted Essential matrix solver
    typedef ACKernelAdaptorEssential<
        openMVG::essential::kernel::FivePointKernel,
        openMVG::fundamental::kernel::EpipolarDistanceError,
        UnnormalizerT,
        Mat3>
        KernelType;

    const cameras::Pinhole_Intrinsic * ptrPinhole_I = (const cameras::Pinhole_Intrinsic*)(cam_I);
    const cameras::Pinhole_Intrinsic * ptrPinhole_J = (const cameras::Pinhole_Intrinsic*)(cam_J);

    KernelType kernel(
      xI, sfmData->GetViews().at(iIndex)->ui_width, sfmData->GetViews().at(iIndex)->ui_height,
      xJ, sfmData->GetViews().at(jIndex)->ui_width, sfmData->GetViews().at(jIndex)->ui_height,
      ptrPinhole_I->K(), ptrPinhole_J->K());

    // Robustly estimate the Essential matrix with A Contrario ransac
    const double upper_bound_precision = Square(m_dPrecision);

    std::vector<size_t> inliers;
    const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, inliers, m_stIteration, &m_E, upper_bound_precision);

    if (inliers.size() <= KernelType::MINIMUM_SAMPLES * OPENMVG_MINIMUM_SAMPLES_COEF)
    {
      inliers.clear();
      return false;
    }

    m_dPrecision_robust = ACRansacOut.first;

    // Fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(
          inliers,
          putativeMatchesPerType,
          descTypes,
          geometricInliersPerType);

    return true;
  }

  /**
   * @brief Geometry_guided_matching
   * @param sfmData
   * @param regionsPerView
   * @param imageIdsPair
   * @param dDistanceRatio
   * @param matches
   * @return
   */
  bool Geometry_guided_matching
  (
    const sfm::SfM_Data * sfmData,
    const features::RegionsPerView& regionsPerView,
    const Pair imageIdsPair,
    const double dDistanceRatio,
    matching::MatchesPerDescType & matches
  )
  {
    if (m_dPrecision_robust != std::numeric_limits<double>::infinity())
    {
      // Get back corresponding view index
      const IndexT viewId_I = imageIdsPair.first;
      const IndexT viewId_J = imageIdsPair.second;

      const sfm::View * view_I = sfmData->views.at(viewId_I).get();
      const sfm::View * view_J = sfmData->views.at(viewId_J).get();

      // Check that valid cameras can be retrieved for the pair of views
      const cameras::IntrinsicBase * cam_I =
        sfmData->GetIntrinsics().count(view_I->id_intrinsic) ?
          sfmData->GetIntrinsics().at(view_I->id_intrinsic).get() : nullptr;
      const cameras::IntrinsicBase * cam_J =
        sfmData->GetIntrinsics().count(view_J->id_intrinsic) ?
          sfmData->GetIntrinsics().at(view_J->id_intrinsic).get() : nullptr;

      if (!cam_I || !cam_J)
        return false;

      if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
        return false;

      const cameras::Pinhole_Intrinsic * ptrPinhole_I = (const cameras::Pinhole_Intrinsic*)(cam_I);
      const cameras::Pinhole_Intrinsic * ptrPinhole_J = (const cameras::Pinhole_Intrinsic*)(cam_J);

      Mat3 F;
      FundamentalFromEssential(m_E, ptrPinhole_I->K(), ptrPinhole_J->K(), &F);

      geometry_aware::GuidedMatching<Mat3,
            openMVG::fundamental::kernel::EpipolarDistanceError>(
            //openMVG::fundamental::kernel::SymmetricEpipolarDistanceError>(
        F,
        cam_I, regionsPerView.getAllRegions(viewId_I),
        cam_J, regionsPerView.getAllRegions(viewId_J),
        Square(m_dPrecision_robust), Square(dDistanceRatio),
        matches);
    }
    return matches.getNbAllMatches() != 0;
  }

  double m_dPrecision;  //upper_bound precision used for robust estimation
  size_t m_stIteration; //maximal number of iteration for robust estimation
  //
  //-- Stored data
  Mat3 m_E;
  double m_dPrecision_robust;
};

} // namespace openMVG
} //namespace matching_image_collection

