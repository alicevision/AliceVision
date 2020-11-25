// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "RelativePoseInfo.hpp"
#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/matching/supportEstimation.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/relativePose/Essential5PSolver.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>

namespace aliceVision {
namespace sfm {

bool estimate_Rt_fromE(const Mat3 & K1, const Mat3 & K2,
  const Mat & x1, const Mat & x2,
  const Mat3 & E, const std::vector<size_t> & vec_inliers,
  Mat3 * R, Vec3 * t)
{
  // Accumulator to find the best solution
  std::vector<size_t> f(4, 0);

  std::vector<Mat3> Rs;  // Rotation matrix.
  std::vector<Vec3> ts;  // Translation matrix.

  // Recover best rotation and translation from E.
  motionFromEssential(E, &Rs, &ts);

  //-> Test the 4 solutions will all the points
  assert(Rs.size() == 4);
  assert(ts.size() == 4);

  Mat34 P1, P2;
  Mat3 R1 = Mat3::Identity();
  Vec3 t1 = Vec3::Zero();
  P_from_KRt(K1, R1, t1, &P1);

  for (unsigned int i = 0; i < 4; ++i)
  {
    const Mat3 &R2 = Rs[i];
    const Vec3 &t2 = ts[i];
    P_from_KRt(K2, R2, t2, &P2);
    Vec3 X;

    for (size_t k = 0; k < vec_inliers.size(); ++k)
    {
      const Vec2
        & x1_ = x1.col(vec_inliers[k]),
        & x2_ = x2.col(vec_inliers[k]);
      multiview::TriangulateDLT(P1, x1_, P2, x2_, &X);
      // Test if point is front to the two cameras.
      if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0)
      {
        ++f[i];
      }
    }
  }
  // Check the solution:
  const std::vector<size_t>::iterator iter = std::max_element(f.begin(), f.end());
  if (*iter == 0)
  {
    // There is no right solution with points in front of the cameras
    return false;
  }
  const size_t index = std::distance(f.begin(), iter);
  (*R) = Rs[index];
  (*t) = ts[index];

  return true;
}

bool robustRelativePose(const Mat3& K1, const Mat3& K2,
                        const Mat& x1, const Mat& x2,
                        std::mt19937 &randomNumberGenerator,
                        RelativePoseInfo & relativePose_info,
                        const std::pair<size_t, size_t> & size_ima1,
                        const std::pair<size_t, size_t> & size_ima2,
                        const size_t max_iteration_count)
{
  // use the 5 point solver to estimate E
  using SolverT = multiview::relativePose::Essential5PSolver;

  // define the kernel
  using KernelT = multiview::RelativePoseKernel_K<SolverT, multiview::relativePose::FundamentalEpipolarDistanceError, robustEstimation::Mat3Model>;

  KernelT kernel(x1, size_ima1.first, size_ima1.second,
                 x2, size_ima2.first, size_ima2.second, K1, K2);


  robustEstimation::Mat3Model model;

  // robustly estimation of the Essential matrix and its precision
  const std::pair<double,double> acRansacOut = robustEstimation::ACRANSAC(kernel, randomNumberGenerator,
                                                                          relativePose_info.vec_inliers,
                                                                          max_iteration_count,
                                                                          &model,
                                                                          relativePose_info.initial_residual_tolerance);
  relativePose_info.essential_matrix = model.getMatrix();
  relativePose_info.found_residual_precision = acRansacOut.first;

  if(relativePose_info.vec_inliers.size() < kernel.getMinimumNbRequiredSamples() * ALICEVISION_MINIMUM_SAMPLES_COEF)
  {
    ALICEVISION_LOG_INFO("robustRelativePose: no sufficient coverage (the model does not support enough samples): " << relativePose_info.vec_inliers.size());
    return false; // no sufficient coverage (the model does not support enough samples)
  }

  // estimation of the relative poses
  Mat3 R;
  Vec3 t;
  if (!estimate_Rt_fromE(
    K1, K2, x1, x2,
    relativePose_info.essential_matrix, relativePose_info.vec_inliers, &R, &t))
  {
    ALICEVISION_LOG_INFO("robustRelativePose: cannot find a valid [R|t] couple that makes the inliers in front of the camera.");
    return false; // cannot find a valid [R|t] couple that makes the inliers in front of the camera.
  }

  // Store [R|C] for the second camera, since the first camera is [Id|0]
  relativePose_info.relativePose = geometry::Pose3(R, -R.transpose() * t);

  return true;
}

} // namespace sfm
} // namespace aliceVision

