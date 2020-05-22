// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/robustEstimation/ISolver.hpp>
#include <aliceVision/multiview/resection/ISolverErrorResection.hpp>

namespace aliceVision {
namespace multiview {
namespace resection {

/**
 * @brief The structure P4PfModel contain one output model
 */
struct P4PfModel
{
  P4PfModel(Mat R, Vec3 t, double f)
    : _R(R)
    , _t(t)
    , _f(f)
  {}

  Mat34 getP() const
  {
    Mat34 P;
    Mat K = Mat(3, 3);

    K << _f, 0, 0,
         0, _f, 0,
         0,  0, 1;

    P.block(0, 0, 3, 3) = K * _R;
    P.block(0, 3, 3, 1) = K * _t;

    return P;
  }

  /// rotation matrix
  Mat _R;
  /// translation vector
  Vec3 _t;
  /// focal length
  double _f;
};

struct P4PfError : public ISolverErrorResection<P4PfModel>
{
  /**
   * @brief Compute the residual of the projection distance(p2d, project(P,p3d))
   * @param[in] model solution
   * @param[in] p2d feature vector
   * @param[in] p3d corresponding 3D world point
   */
  inline double error(const P4PfModel& model, const Vec2& p2d, const Vec3& p3d) const override
  {
    return (p2d - project(model.getP(), p3d)).norm();
  }
};

/**
 * @brief  Compute the absolute pose and focal length of a camera using three 3D-to-2D correspondences
 * @author Martin Bujnak, adapted to aliceVision by Michal Polic
 * @ref [1] A general solution to the p4p
 *          Bujnak, M., Kukelova, Z., and Pajdla T.
 *          CVPR 2008
 */
class P4PfSolver : public robustEstimation::ISolver<P4PfModel>
{
public:

  /**
   * @brief Return the minimum number of required samples
   * @return minimum number of required samples
   */
  inline std::size_t getMinimumNbRequiredSamples() const override
  {
    return 4;
  }

  /**
   * @brief Return the maximum number of models
   * @return maximum number of models
   */
  inline std::size_t getMaximumNbModels() const override
  {
    return 10;
  }

  /**
   * @brief Solve the problem of camera pose.
   * @param[in] x2d featureVectors 2 x 4 matrix with feature vectors with subtracted principal point (each column is a vector)
   * @param[in] x3d worldPoints 3 x 4 matrix with corresponding 3D world points (each column is a point)
   * @param[out] models M x n vector that will contain the each solution in structure P4PfModel (rotation matrix P4PfModel._R,
   * translation vector P4PfModel._t, focal length P4PfModel._f). Following equation holds for each solution:
   * lambda*pt2D = diag([P4PfModel._f P4PfModel._f 1])*[P4PfModel._R P4PfModel._t] * pt3D
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<P4PfModel>& models)  const override;

  /**
   * @brief Solve the problem.
   *
   * @param[in]  x2d 2d points in the first image. One per column.
   * @param[in]  x3d Corresponding 3d points in the second image. One per column.
   * @param[out] models A vector into which the computed models are stored.
   * @param[in]  weights.
   */
  void solve(const Mat& x2d, const Mat& x3d, std::vector<P4PfModel>& models, const std::vector<double>& weights) const override
  {
     throw std::logic_error("P4PfSolver does not support problem solving with weights.");
  }
};

} // namespace resection
} // namespace multiview
} // namespace aliceVision
