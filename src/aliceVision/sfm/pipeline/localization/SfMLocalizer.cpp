// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SfMLocalizer.hpp"
#include <aliceVision/config.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/BundleAdjustmentSymbolicCeres.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>
#include <aliceVision/matching/supportEstimation.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>
#include <aliceVision/multiview/resection/ResectionKernel.hpp>
#include <aliceVision/multiview/resection/Resection6PSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/multiview/ResectionKernel.hpp>

namespace aliceVision {
namespace sfm {

bool SfMLocalizer::Localize(const Pair& imageSize,
                            const camera::IntrinsicBase* optionalIntrinsics,
                            std::mt19937 &randomNumberGenerator, 
                            ImageLocalizerMatchData& resectionData,
                            geometry::Pose3& pose,
                            robustEstimation::ERobustEstimator estimator)
{
  // compute the camera pose (resectioning)

  Mat34 P;
  resectionData.vec_inliers.clear();

  // setup the admissible upper bound residual error
  const double precision =
    resectionData.error_max == std::numeric_limits<double>::infinity() ?
    std::numeric_limits<double>::infinity() :
    Square(resectionData.error_max);

  std::size_t minimumSamples = 0;
  const camera::Pinhole* pinholeCam = dynamic_cast<const camera::Pinhole*>(optionalIntrinsics);

  if(pinholeCam == nullptr || !pinholeCam->isValid())
  {
    // classic resection (try to compute the entire P matrix)
    using SolverT = multiview::resection::Resection6PSolver;
    using KernelT = multiview::ResectionKernel<SolverT, multiview::resection::ProjectionDistanceSquaredError, multiview::UnnormalizerResection, robustEstimation::Mat34Model>;

    const KernelT kernel(resectionData.pt2D, resectionData.pt3D, imageSize.first, imageSize.second);

    minimumSamples = kernel.getMinimumNbRequiredSamples();

    // robust estimation of the Projection matrix and its precision
    robustEstimation::Mat34Model model;
    const std::pair<double,double> ACRansacOut = robustEstimation::ACRANSAC(kernel, randomNumberGenerator, resectionData.vec_inliers, resectionData.max_iteration, &model, precision);
    P = model.getMatrix();
    // update the upper bound precision of the model found by AC-RANSAC
    resectionData.error_max = ACRansacOut.first;
  }
  else
  {
    // undistort the points if the camera has a distortion model
    Mat pt2Dundistorted;
    const bool hasDistortion = pinholeCam->hasDistortion();
    if(hasDistortion)
    {
      const std::size_t numPts = resectionData.pt2D.cols();
      pt2Dundistorted = Mat2X(2, numPts);
      for(std::size_t iPoint = 0; iPoint < numPts; ++iPoint)
      {
        pt2Dundistorted.col(iPoint) = pinholeCam->get_ud_pixel(resectionData.pt2D.col(iPoint));
      }
    }

    switch(estimator)
    {
      case robustEstimation::ERobustEstimator::ACRANSAC:
      {
        // since K calibration matrix is known, compute only [R|t]
        using SolverT = multiview::resection::P3PSolver;
        using KernelT = multiview::ResectionKernel_K<SolverT, multiview::resection::ProjectionDistanceSquaredError, multiview::UnnormalizerResection, robustEstimation::Mat34Model>;

        // otherwise we just pass the input points
        const KernelT kernel = KernelT(hasDistortion ? pt2Dundistorted : resectionData.pt2D, resectionData.pt3D, pinholeCam->K());

        minimumSamples = kernel.getMinimumNbRequiredSamples();

        // robust estimation of the Projection matrix and its precision
        robustEstimation::Mat34Model model;
        const std::pair<double, double> ACRansacOut = robustEstimation::ACRANSAC(kernel, randomNumberGenerator, resectionData.vec_inliers, resectionData.max_iteration, &model, precision);

        P = model.getMatrix();

        // update the upper bound precision of the model found by AC-RANSAC
        resectionData.error_max = ACRansacOut.first;
        break;
      }

      case robustEstimation::ERobustEstimator::LORANSAC:
      {

        // just a safeguard
        if(resectionData.error_max == std::numeric_limits<double>::infinity())
        {
          // switch to a default value
          resectionData.error_max = 4.0;
          ALICEVISION_LOG_DEBUG("LORansac: error was set to infinity, a default value of " 
                  << resectionData.error_max << " is going to be used");
        }

        // use the P3P solver for generating the model
        using SolverT = multiview::resection::P3PSolver;
        using SolverLsT = multiview::resection::Resection6PSolver;

        // use the six point algorithm as Least square solution to refine the model
        using KernelT = multiview::ResectionKernel_K<SolverT, multiview::resection::ProjectionDistanceSquaredError, multiview::UnnormalizerResection, robustEstimation::Mat34Model, SolverLsT>;

        // otherwise we just pass the input points
        const KernelT kernel = KernelT(hasDistortion ? pt2Dundistorted : resectionData.pt2D, resectionData.pt3D, pinholeCam->K());

        minimumSamples = kernel.getMinimumNbRequiredSamples();

        // this is just stupid and ugly, the threshold should be always give as pixel
        // value, the scorer should be not aware of the fact that we treat squared errors
        // and normalization inside the kernel
        // @todo refactor, maybe move scorer directly inside the kernel
        const double threshold = resectionData.error_max * resectionData.error_max * (kernel.normalizer2()(0, 0) * kernel.normalizer2()(0, 0));
        robustEstimation::ScoreEvaluator<KernelT> scorer(threshold);

        const robustEstimation::Mat34Model model = robustEstimation::LO_RANSAC(kernel, scorer, randomNumberGenerator, &resectionData.vec_inliers);
        P = model.getMatrix();

        break;
      }

      default:
        throw std::runtime_error("[SfMLocalizer::localize] Only ACRansac and LORansac are supported!");
    }
  }

  const bool resection = matching::hasStrongSupport(resectionData.vec_inliers, resectionData.vec_descType, minimumSamples);

  if(!resection)
  {
    ALICEVISION_LOG_DEBUG("Resection status is false:\n"
                          "\t- resection_data.vec_inliers.size() = " << resectionData.vec_inliers.size() << "\n"
                          "\t- minimumSamples = " << minimumSamples);
  }

  if(resection)
  {
    resectionData.projection_matrix = P;
    Mat3 K, R;
    Vec3 t;
    KRt_from_P(P, &K, &R, &t);
    pose = geometry::Pose3(R, -R.transpose() * t);
  }

  ALICEVISION_LOG_INFO("Robust Resection information:\n"
    "\t- resection status: " << resection << "\n"
    "\t- threshold (error max): " << resectionData.error_max << "\n"
    "\t- # points used for resection: " << resectionData.pt2D.cols() << "\n"
    "\t- # points validated by robust resection: " << resectionData.vec_inliers.size());

  return resection;
}

bool SfMLocalizer::RefinePose(camera::IntrinsicBase* intrinsics,
                              geometry::Pose3& pose,
                              const ImageLocalizerMatchData& matchingData,
                              bool refinePose,
                              bool refineIntrinsic)
{
  // Setup a tiny SfM scene with the corresponding 2D-3D data
  sfmData::SfMData tinyScene;

  // view
  std::shared_ptr<sfmData::View> view = std::make_shared<sfmData::View>("", 0, 0, 0);
  tinyScene.views.insert(std::make_pair(0, view));

  // pose
  tinyScene.setPose(*view, sfmData::CameraPose(pose));

  // intrinsic (the shared_ptr does not take the ownership, will not release the input pointer)
  std::shared_ptr<camera::IntrinsicBase> localIntrinsics(intrinsics->clone());
  tinyScene.intrinsics[0] = localIntrinsics;

  const double unknownScale = 0.0;
  // structure data (2D-3D correspondences)
  for(std::size_t i = 0; i < matchingData.vec_inliers.size(); ++i)
  {
    const std::size_t idx = matchingData.vec_inliers[i];
    sfmData::Landmark landmark;
    landmark.X = matchingData.pt3D.col(idx);
    landmark.observations[0] = sfmData::Observation(matchingData.pt2D.col(idx), UndefinedIndexT, unknownScale); // TODO-SCALE
    tinyScene.structure[i] = std::move(landmark);
  }

  BundleAdjustmentCeres BA;
  BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_NONE;

  if(refinePose)
    refineOptions |= BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION;
  if(refineIntrinsic)
    refineOptions |= BundleAdjustment::REFINE_INTRINSICS_ALL;

  const bool success = BA.adjust(tinyScene, refineOptions);

  if(!success)
    return false;

  pose = tinyScene.getPose(*view).getTransform();

  if(refineIntrinsic)
    intrinsics->assign(*localIntrinsics);

  return true;
}

} // namespace sfm
} // namespace aliceVision
