// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/types.hpp"
#include "aliceVision/multiview/solver_essential_kernel.hpp"
#include "aliceVision/multiview/essential.hpp"
#include "aliceVision/robust_estimation/robust_estimator_ACRansac.hpp"
#include "aliceVision/robust_estimation/robust_estimator_ACRansacKernelAdaptator.hpp"
#include "aliceVision/robust_estimation/guided_matching.hpp"
#include <limits>

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/sfm/sfm_data.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matching_image_collection/GeometricFilterMatrix.hpp"

namespace aliceVision {
namespace matching_image_collection {

//-- A contrario essential matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilter_EMatrix_AC : public GeometricFilterMatrix
{
  GeometricFilter_EMatrix_AC(
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
    const sfm::SfM_Data * sfmData,
    const Regions_or_Features_ProviderT& regionsPerView,
    const Pair pairIndex,
    const matching::MatchesPerDescType & putativeMatchesPerType,
    matching::MatchesPerDescType & out_geometricInliersPerType)
  {
    using namespace aliceVision;
    using namespace aliceVision::robust;
    out_geometricInliersPerType.clear();

    // Get back corresponding view index
    const IndexT iIndex = pairIndex.first;
    const IndexT jIndex = pairIndex.second;

    const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(pairIndex);
    if(descTypes.empty())
      return EstimationStatus(false, false);

    // Reject pair with missing Intrinsic information
    const sfm::View * view_I = sfmData->views.at(iIndex).get();
    const sfm::View * view_J = sfmData->views.at(jIndex).get();

    // Check that valid cameras can be retrieved for the pair of views
    const camera::IntrinsicBase * cam_I = sfmData->GetIntrinsicPtr(view_I->getIntrinsicId());
    const camera::IntrinsicBase * cam_J = sfmData->GetIntrinsicPtr(view_J->getIntrinsicId());

    if (!cam_I || !cam_J)
      return EstimationStatus(false, false);
    if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
      return EstimationStatus(false, false);

    // Get corresponding point regions arrays
    Mat xI,xJ;
    MatchesPairToMat(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

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
      xI, sfmData->GetViews().at(iIndex)->getWidth(), sfmData->GetViews().at(iIndex)->getHeight(),
      xJ, sfmData->GetViews().at(jIndex)->getWidth(), sfmData->GetViews().at(jIndex)->getHeight(),
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
    const bool hasStrongSupport = robust::hasStrongSupport(out_geometricInliersPerType, KernelType::MINIMUM_SAMPLES);

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
    const sfm::SfM_Data * sfmData,
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

      const sfm::View * view_I = sfmData->views.at(viewId_I).get();
      const sfm::View * view_J = sfmData->views.at(viewId_J).get();

      // Check that valid cameras can be retrieved for the pair of views
      const camera::IntrinsicBase * cam_I =
        sfmData->GetIntrinsics().count(view_I->getIntrinsicId()) ?
          sfmData->GetIntrinsics().at(view_I->getIntrinsicId()).get() : nullptr;
      const camera::IntrinsicBase * cam_J =
        sfmData->GetIntrinsics().count(view_J->getIntrinsicId()) ?
          sfmData->GetIntrinsics().at(view_J->getIntrinsicId()).get() : nullptr;

      if (!cam_I || !cam_J)
        return false;

      if ( !isPinhole(cam_I->getType()) || !isPinhole(cam_J->getType()))
        return false;

      const camera::Pinhole * ptrPinhole_I = (const camera::Pinhole*)(cam_I);
      const camera::Pinhole * ptrPinhole_J = (const camera::Pinhole*)(cam_J);

      Mat3 F;
      FundamentalFromEssential(m_E, ptrPinhole_I->K(), ptrPinhole_J->K(), &F);

      geometry_aware::GuidedMatching<Mat3,
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

} // namespace matching_image_collection
} // namespace aliceVision


