// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2011 Laurent Kneip, ETH Zurich.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

#include <iostream>

namespace aliceVision {
namespace resection {

typedef Eigen::Matrix<double, 5, 1> Vec5;

void solveQuartic(const Vec5 & factors, Vec4 & realRoots);

/**
 * @brief Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *
 * Reference: [1] A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *              Kneip, L.; Scaramuzza, D. ; Siegwart, R.
 *              CVPR 2011
 *
 * @param featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 * @param worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 * @param solutions: 3x16 matrix that will contain the solutions
 *                   form: [ C1,R1, C2,R2 ... ]
 *                   the obtained orientation matrices are defined as transforming points from the cam to the world frame
 * @return bool: true if correct execution
 *               false if world points aligned
 *
 * Author: Laurent Kneip, adapted to the project by Pierre Moulon
 */

bool compute_P3P_Poses(const Mat3 & featureVectors, const Mat3 & worldPoints, Mat & solutions);

struct P3PSolver
{

  enum
  {
    MINIMUM_SAMPLES = 3
  };

  enum
  {
    MAX_MODELS = 4
  };
  // Solve the problem of camera pose.
  static void Solve(const Mat &pt2D, const Mat &pt3D, std::vector<Mat34> *models);

  // Compute the residual of the projection distance(pt2D, Project(P,pt3D))
  static double Error(const Mat34 & P, const Vec2 & pt2D, const Vec3 & pt3D);
};

class P3P_ResectionKernel_K
{
public:
  typedef Mat34 Model;

  enum
  {
    MINIMUM_SAMPLES = 3
  };

  P3P_ResectionKernel_K(const Mat2X &x_camera, const Mat3X &X, const Mat3 &K = Mat3::Identity());

  void Fit(const std::vector<size_t> &samples, std::vector<Model> *models) const;

  double Error(size_t sample, const Model &model) const;

  size_t NumSamples() const;

private:
  Mat2X x_image_; // camera coordinates
  Mat3X x_camera_; // camera coordinates (normalized)
  Mat3X X_; // 3D points
  Mat3 K_;
};

} // namespace resection
} // namespace aliceVision
