// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/pipelines/localization/SfM_Localizer.hpp"
#include "aliceVision/sfm/sfm_data_BA_ceres.hpp"

#include "aliceVision/multiview/solver_resection_kernel.hpp"
#include "aliceVision/multiview/solver_resection_p3p.hpp"
#include "aliceVision/robustEstimation/ACRansac.hpp"
#include "aliceVision/robustEstimation/ACRansacKernelAdaptator.hpp"
#include <aliceVision/robustEstimation/LORansac.hpp>
#include <aliceVision/robustEstimation/LORansacKernelAdaptor.hpp>
#include <aliceVision/robustEstimation/ScoreEvaluator.hpp>

#include <aliceVision/config.hpp>
namespace aliceVision {
namespace sfm {

struct ResectionSquaredResidualError 
{
  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  // Return the squared error
  static double Error(const Mat34 & P, const Vec2 & pt2D, const Vec3 & pt3D)
  {
    const Vec2 x = Project(P, pt3D);
    return (x - pt2D).squaredNorm();
  }
};

bool SfM_Localizer::Localize
(
  const Pair & image_size,
  const camera::IntrinsicBase * optional_intrinsics,
  Image_Localizer_Match_Data & resection_data,
  geometry::Pose3 & pose,
  robustEstimation::EROBUST_ESTIMATOR estimator
)
{
  // --
  // Compute the camera pose (resectioning)
  // --
  Mat34 P;
  resection_data.vec_inliers.clear();

  // Setup the admissible upper bound residual error
  const double dPrecision =
    resection_data.error_max == std::numeric_limits<double>::infinity() ?
    std::numeric_limits<double>::infinity() :
    Square(resection_data.error_max);

  size_t MINIMUM_SAMPLES = 0;
  const camera::Pinhole * pinhole_cam = dynamic_cast<const camera::Pinhole *>(optional_intrinsics);
  if (pinhole_cam == nullptr || !pinhole_cam->isValid())
  {
    //--
    // Classic resection (try to compute the entire P matrix)
    typedef aliceVision::resection::kernel::SixPointResectionSolver SolverType;
    MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;

    typedef aliceVision::robustEstimation::ACKernelAdaptorResection<
      SolverType, ResectionSquaredResidualError, aliceVision::robustEstimation::UnnormalizerResection, Mat34>
      KernelType;

    KernelType kernel(resection_data.pt2D, image_size.first, image_size.second,
      resection_data.pt3D);
    // Robust estimation of the Projection matrix and its precision
    const std::pair<double,double> ACRansacOut =
      aliceVision::robustEstimation::ACRANSAC(kernel, resection_data.vec_inliers, resection_data.max_iteration, &P, dPrecision, true);
    // Update the upper bound precision of the model found by AC-RANSAC
    resection_data.error_max = ACRansacOut.first;
  }
  else
  {
    // undistort the points if the camera has a distortion model
    Mat pt2Dundistorted;
    const bool has_disto = pinhole_cam->have_disto();
    if(has_disto)
    {
      const std::size_t numPts = resection_data.pt2D.cols();
      pt2Dundistorted = Mat2X(2, numPts);
      for(std::size_t iPoint = 0; iPoint < numPts; ++iPoint)
      {
        pt2Dundistorted.col(iPoint) = pinhole_cam->get_ud_pixel(resection_data.pt2D.col(iPoint));
      }
    }

    switch(estimator)
    {
      case robustEstimation::ROBUST_ESTIMATOR_ACRANSAC:
      {
        //--
        // Since K calibration matrix is known, compute only [R|t]
        typedef aliceVision::euclidean_resection::P3PSolver SolverType;
        MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;

        typedef aliceVision::robustEstimation::ACKernelAdaptorResection_K<
                SolverType, ResectionSquaredResidualError,
                aliceVision::robustEstimation::UnnormalizerResection, Mat34> KernelType;

        // otherwise we just pass the input points
        KernelType kernel = KernelType(has_disto ? pt2Dundistorted : resection_data.pt2D, resection_data.pt3D, pinhole_cam->K());

        // Robust estimation of the Projection matrix and its precision
        const std::pair<double, double> ACRansacOut =
                aliceVision::robustEstimation::ACRANSAC(kernel, resection_data.vec_inliers, resection_data.max_iteration, &P, dPrecision, true);
        // Update the upper bound precision of the model found by AC-RANSAC
        resection_data.error_max = ACRansacOut.first;
        break;
      }

      case robustEstimation::ROBUST_ESTIMATOR_LORANSAC:
      {

        // just a safeguard
        if(resection_data.error_max == std::numeric_limits<double>::infinity())
        {
          // switch to a default value
          resection_data.error_max = 4.0;
          ALICEVISION_LOG_DEBUG("LORansac: error was set to infinity, a default value of " 
                  << resection_data.error_max << " is going to be used");
        }

        // use the P3P solver for generating the model
        typedef aliceVision::euclidean_resection::P3PSolver SolverType;
        MINIMUM_SAMPLES = SolverType::MINIMUM_SAMPLES;
        // use the six point algorithm as Least square solution to refine the model
        typedef aliceVision::resection::kernel::SixPointResectionSolver SolverLSType;

        typedef aliceVision::robustEstimation::KernelAdaptorResectionLORansac_K<SolverType,
                ResectionSquaredResidualError,
                aliceVision::robustEstimation::UnnormalizerResection,
                SolverLSType,
                Mat34> KernelType;

        // otherwise we just pass the input points
        KernelType kernel = KernelType(has_disto ? pt2Dundistorted : resection_data.pt2D, resection_data.pt3D, pinhole_cam->K());

        // this is just stupid and ugly, the threshold should be always give as pixel
        // value, the scorer should be not aware of the fact that we treat squared errors
        // and normalization inside the kernel
        // @todo refactor, maybe move scorer directly inside the kernel
        const double threshold = resection_data.error_max * resection_data.error_max * (kernel.normalizer2()(0, 0) * kernel.normalizer2()(0, 0));
        robustEstimation::ScoreEvaluator<KernelType> scorer(threshold);
        P = robustEstimation::LO_RANSAC(kernel, scorer, &resection_data.vec_inliers);
        break;
      }

      default:
        throw std::runtime_error("[SfM_Localizer::Localize] Only ACRansac and LORansac are supported!");
    }
  }

  const bool bResection = robustEstimation::hasStrongSupport(resection_data.vec_inliers, resection_data.vec_descType, MINIMUM_SAMPLES);

  if (!bResection)
  {
    ALICEVISION_LOG_DEBUG("bResection is false");
    ALICEVISION_LOG_DEBUG(" because resection_data.vec_inliers.size() = " << resection_data.vec_inliers.size());
    ALICEVISION_LOG_DEBUG(" and MINIMUM_SAMPLES = " << MINIMUM_SAMPLES);
  }

  if (bResection)
  {
    resection_data.projection_matrix = P;
    Mat3 K, R;
    Vec3 t;
    KRt_From_P(P, &K, &R, &t);
    pose = geometry::Pose3(R, -R.transpose() * t);
  }
  ALICEVISION_LOG_DEBUG(
    "-------------------------------\n"
    "-- Robust Resection\n"
    "-- Resection status: " << bResection << "\n"
    "-- #Points used for Resection: " << resection_data.pt2D.cols() << "\n"
    "-- #Points validated by robust Resection: " << resection_data.vec_inliers.size() << "\n"
    "-- Threshold: " << resection_data.error_max << "\n"
    "-------------------------------");
  return bResection;
}

bool SfM_Localizer::RefinePose
(
  camera::IntrinsicBase * intrinsics,
  geometry::Pose3 & pose,
  const Image_Localizer_Match_Data & matching_data,
  bool b_refine_pose,
  bool b_refine_intrinsic
)
{
  // Setup a tiny SfM scene with the corresponding 2D-3D data
  SfM_Data sfm_data;

  // view
  std::shared_ptr<View> view = std::make_shared<View>("", 0, 0, 0);
  sfm_data.views.insert( std::make_pair(0, view));

  // pose
  sfm_data.setPose(*view, pose);

  // intrinsic (the shared_ptr does not take the ownership, will not release the input pointer)
  std::shared_ptr<camera::IntrinsicBase> localIntrinsics(intrinsics->clone());
  sfm_data.intrinsics[0] = localIntrinsics;

  // structure data (2D-3D correspondences)
  for (size_t i = 0; i < matching_data.vec_inliers.size(); ++i)
  {
    const size_t idx = matching_data.vec_inliers[i];
    Landmark landmark;
    landmark.X = matching_data.pt3D.col(idx);
    landmark.observations[0] = Observation(matching_data.pt2D.col(idx), UndefinedIndexT);
    sfm_data.structure[i] = std::move(landmark);
  }

  Bundle_Adjustment_Ceres bundle_adjustment_obj;
  BA_Refine refineOptions = BA_REFINE_NONE;
  if(b_refine_pose)
    refineOptions |= BA_REFINE_ROTATION | BA_REFINE_TRANSLATION;
  if(b_refine_intrinsic)
    refineOptions |= BA_REFINE_INTRINSICS_ALL;

  const bool b_BA_Status = bundle_adjustment_obj.Adjust(sfm_data, refineOptions);
  if (!b_BA_Status)
    return false;

  pose = sfm_data.getPose(*view);
  if(b_refine_intrinsic)
    intrinsics->assign(*localIntrinsics);
  return true;
}

} // namespace sfm
} // namespace aliceVision
