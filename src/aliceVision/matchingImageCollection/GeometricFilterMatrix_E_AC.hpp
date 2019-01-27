// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/types.hpp"
#include "aliceVision/multiview/essentialKernelSolver.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"
#include "aliceVision/robustEstimation/guidedMatching.hpp"
#include <limits>

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/sfmData/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"

namespace aliceVision {
namespace matchingImageCollection {

//-- A contrario essential matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilterMatrix_E_AC : public GeometricFilterMatrix
{
  GeometricFilterMatrix_E_AC(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_E(Mat3::Identity())
  {}

  /**
   * @brief Given two sets of image points, it estimates the essential matrix
   * relating them using a robust method (like A Contrario Ransac).
   */
  template<typename Regions_or_Features_ProviderT>
  EstimationStatus geometricEstimation(
    const sfmData::SfMData * sfmData,
    const Regions_or_Features_ProviderT& regionsPerView,
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

    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return EstimationStatus(false, false);

    // Reject pair with missing Intrinsic information
    const sfmData::View * view_I = sfmData->views.at(iIndex).get();
    const sfmData::View * view_J = sfmData->views.at(jIndex).get();

    // Check that valid cameras can be retrieved for the pair of views
    const camera::IntrinsicBase * cam_I = sfmData->getIntrinsicPtr(view_I->getIntrinsicId());
    const camera::IntrinsicBase * cam_J = sfmData->getIntrinsicPtr(view_J->getIntrinsicId());

    if (!cam_I || !cam_J)
      return EstimationStatus(false, false);
    if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
      return EstimationStatus(false, false);

    // Get corresponding point regions arrays
    Mat xI,xJ;
    fillMatricesWithUndistortFeaturesMatches(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

    // Define the AContrario adapted Essential matrix solver
    typedef ACKernelAdaptorEssential<
        aliceVision::essential::kernel::FivePointKernel,
        aliceVision::fundamental::kernel::EpipolarDistanceError,
        UnnormalizerT,
        Mat3>
        KernelType;

    const camera::Pinhole * ptrPinhole_I = (const camera::Pinhole*)(cam_I);
    const camera::Pinhole * ptrPinhole_J = (const camera::Pinhole*)(cam_J);

    KernelType kernel(
      xI, sfmData->getViews().at(iIndex)->getWidth(), sfmData->getViews().at(iIndex)->getHeight(),
      xJ, sfmData->getViews().at(jIndex)->getWidth(), sfmData->getViews().at(jIndex)->getHeight(),
      ptrPinhole_I->K(), ptrPinhole_J->K());

    // Robustly estimate the Essential matrix with A Contrario ransac
    const double upper_bound_precision = Square(m_dPrecision);

    std::vector<size_t> inliers;
    const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, inliers, m_stIteration, &m_E, upper_bound_precision);

    if (inliers.empty())
      return EstimationStatus(false, false);

    m_dPrecision_robust = ACRansacOut.first;

    // Fill geometricInliersPerType with inliers from putativeMatchesPerType
    copyInlierMatches(
          inliers,
          putativeMatchesPerType,
          descTypes,
          out_geometricInliersPerType);

    // Check if resection has strong support
    const bool hasStrongSupport = robustEstimation::hasStrongSupport(out_geometricInliersPerType, KernelType::MINIMUM_SAMPLES);

    return EstimationStatus(true, hasStrongSupport);
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

      // Check that valid cameras can be retrieved for the pair of views
      const camera::IntrinsicBase * cam_I =
        sfmData->getIntrinsics().count(view_I->getIntrinsicId()) ?
          sfmData->getIntrinsics().at(view_I->getIntrinsicId()).get() : nullptr;
      const camera::IntrinsicBase * cam_J =
        sfmData->getIntrinsics().count(view_J->getIntrinsicId()) ?
          sfmData->getIntrinsics().at(view_J->getIntrinsicId()).get() : nullptr;

      if (!cam_I || !cam_J)
        return false;

      if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
        return false;

      const camera::Pinhole * ptrPinhole_I = (const camera::Pinhole*)(cam_I);
      const camera::Pinhole * ptrPinhole_J = (const camera::Pinhole*)(cam_J);

      Mat3 F;
      FundamentalFromEssential(m_E, ptrPinhole_I->K(), ptrPinhole_J->K(), &F);

      robustEstimation::GuidedMatching<Mat3,
            aliceVision::fundamental::kernel::EpipolarDistanceError>(
            //aliceVision::fundamental::kernel::SymmetricEpipolarDistanceError>(
        F,
        cam_I, regionsPerView.getAllRegions(viewId_I),
        cam_J, regionsPerView.getAllRegions(viewId_J),
        Square(m_dPrecision_robust), Square(dDistanceRatio),
        matches);
    }
    return matches.getNbAllMatches() != 0;
  }

  //
  //-- Stored data
  Mat3 m_E;
};

} // namespace matchingImageCollection
} // namespace aliceVision


