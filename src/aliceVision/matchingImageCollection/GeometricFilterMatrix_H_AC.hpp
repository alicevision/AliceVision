// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/multiview/homographyKernelSolver.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"
#include "aliceVision/robustEstimation/guidedMatching.hpp"

#include "aliceVision/matching/IndMatch.hpp"
#include "aliceVision/matching/IndMatchDecorator.hpp"
#include "aliceVision/sfmData/SfMData.hpp"
#include "aliceVision/feature/RegionsPerView.hpp"
#include "aliceVision/matchingImageCollection/GeometricFilterMatrix.hpp"

namespace aliceVision {
namespace matchingImageCollection {

//-- A contrario homography matrix estimation template functor used for filter pair of putative correspondences
struct GeometricFilterMatrix_H_AC : public GeometricFilterMatrix
{
  GeometricFilterMatrix_H_AC(
    double dPrecision = std::numeric_limits<double>::infinity(),
    size_t iteration = 1024)
    : GeometricFilterMatrix(dPrecision, std::numeric_limits<double>::infinity(), iteration)
    , m_H(Mat3::Identity())
  {}

  /**
   * @brief Given two sets of image points, it estimates the homography matrix
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

    // Retrieve all 2D features as undistorted positions into flat arrays
    Mat xI, xJ;
    fillMatricesWithUndistortFeaturesMatches(pairIndex, putativeMatchesPerType, sfmData, regionsPerView, descTypes, xI, xJ);

    // Define the AContrario adapted Homography matrix solver
    typedef ACKernelAdaptor<
        aliceVision::homography::kernel::FourPointSolver,
        aliceVision::homography::kernel::AsymmetricError,
        UnnormalizerI,
        Mat3>
        KernelType;

    KernelType kernel(
      xI, sfmData->getViews().at(iIndex)->getWidth(), sfmData->getViews().at(iIndex)->getHeight(),
      xJ, sfmData->getViews().at(jIndex)->getWidth(), sfmData->getViews().at(jIndex)->getHeight(),
      false); // configure as point to point error model.

    // Robustly estimate the Homography matrix with A Contrario ransac
    const double upper_bound_precision = Square(m_dPrecision);

    std::vector<size_t> inliers;
    const std::pair<double,double> ACRansacOut = ACRANSAC(kernel, inliers, m_stIteration, &m_H, upper_bound_precision);

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
   * @brief Export point feature based vector to a matrix [(x,y)'T, (x,y)'T].
   * Use the camera intrinsics in order to get undistorted pixel coordinates
   */
  template<typename MatT >
  static void fillMatricesWithUndistortFeatures(
    const camera::IntrinsicBase * cam,
    const feature::PointFeatures & vec_feats,
    MatT & m)
  {
    using Scalar = typename MatT::Scalar; // Output matrix type

    const bool hasValidIntrinsics = cam && cam->isValid();
    size_t i = 0;

    if (hasValidIntrinsics)
    {
      for( feature::PointFeatures::const_iterator iter = vec_feats.begin();
        iter != vec_feats.end(); ++iter, ++i)
      {
          m.col(i) = cam->get_ud_pixel(Vec2(iter->x(), iter->y()));
      }
    }
    else
    {
      for( feature::PointFeatures::const_iterator iter = vec_feats.begin();
        iter != vec_feats.end(); ++iter, ++i)
      {
          m.col(i) = iter->coords().cast<Scalar>();
        }
    }
  }

  template<typename MatT >
  static void createMatricesWithUndistortFeatures(
    const camera::IntrinsicBase * cam,
    const feature::MapRegionsPerDesc & regionsPerDesc,
    MatT & m)
  {
    size_t nbRegions = 0;
    for(const auto &regions: regionsPerDesc)
    {
      nbRegions += regions.second->RegionCount();
    }
    m.resize(2, nbRegions);

    size_t y = 0;
    for(const auto &regions: regionsPerDesc)
    {
      fillMatricesWithUndistortFeatures(
            cam,
            regions.second,
            m.block(0, y, 2, regions.second->RegionCount()));
      y += regions.second->RegionCount();
    }
  }

  template<typename MatT >
  static void createMatricesWithUndistortFeatures(
    const camera::IntrinsicBase * cam,
    const feature::PointFeatures & vec_feats,
    MatT & m)
  {
    size_t nbRegions = vec_feats.size();
    m.resize(2, nbRegions);

    fillMatricesWithUndistortFeatures(
          cam,
          vec_feats,
          m);
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
      const std::vector<feature::EImageDescriberType> descTypes = regionsPerView.getCommonDescTypes(imageIdsPair);
      if(descTypes.empty())
        return false;

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

      if (dDistanceRatio < 0)
      {
        for(const feature::EImageDescriberType descType: descTypes)
        {
          assert(descType != feature::EImageDescriberType::UNINITIALIZED);
          matching::IndMatches localMatches;

          const feature::Regions& regions_I = regionsPerView.getRegions(viewId_I, descType);
          const feature::Regions& regions_J = regionsPerView.getRegions(viewId_J, descType);
          const feature::PointFeatures pointsFeaturesI = regions_I.GetRegionsPositions();
          const feature::PointFeatures pointsFeaturesJ = regions_J.GetRegionsPositions();

          // Filtering based only on region positions
          Mat xI, xJ;
          createMatricesWithUndistortFeatures(cam_I, pointsFeaturesI, xI);
          createMatricesWithUndistortFeatures(cam_J, pointsFeaturesJ, xJ);

          robustEstimation::GuidedMatching
            <Mat3, aliceVision::homography::kernel::AsymmetricError>(
            m_H, xI, xJ, Square(m_dPrecision_robust), localMatches);

          // Remove matches that have the same (X,Y) coordinates
          matching::IndMatchDecorator<float> matchDeduplicator(localMatches, pointsFeaturesI, pointsFeaturesJ);
          matchDeduplicator.getDeduplicated(localMatches);
          matches[descType] = localMatches;
        }
      }
      else
      {
        // Filtering based on region positions and regions descriptors
        robustEstimation::GuidedMatching
          <Mat3, aliceVision::homography::kernel::AsymmetricError>(
          m_H,
          cam_I, regionsPerView.getAllRegions(viewId_I),
          cam_J, regionsPerView.getAllRegions(viewId_J),
          Square(m_dPrecision_robust), Square(dDistanceRatio),
          matches);
      }
    }
    return matches.getNbAllMatches() != 0;
  }

  //
  //-- Stored data
  Mat3 m_H;
};

} // namespace matchingImageCollection
} // namespace aliceVision


