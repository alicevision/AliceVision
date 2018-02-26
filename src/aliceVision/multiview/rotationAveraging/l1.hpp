// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/multiview/rotationAveraging/common.hpp>

//------------------
//-- Bibliography --
//------------------
//- [1] "Efficient and Robust Large-Scale Rotation Averaging"
//- Authors: Avishek Chatterjee and Venu Madhav Govindu
//- Date: December 2013.
//- Conference: ICCV.

namespace aliceVision   {
namespace rotationAveraging  {
namespace l1  {

// Defines
typedef double REAL;
typedef std::vector<aliceVision::Mat3> Matrix3x3Arr;

/**
 * @brief Compute an initial estimation of global rotation (chain rotations along a MST).
 *
 * @param[in] RelRs Relative weighted rotation matrices
 * @param[out] Rs output global rotation matrices
 * @param[in] nMainViewID Id of the image considered as Identity (unit rotation)
 */
void InitRotationsMST
(
  const RelativeRotations& RelRs,
  Matrix3x3Arr& Rs,
  const size_t nMainViewID
);

/**
 * @brief Compute an initial estimation of global rotation and refines them under the L1 norm, [1].
 *
 * @param[in] RelRs Relative weighted rotation matrices
 * @param[out] Rs output global rotation matrices
 * @param[in] nMainViewID Id of the image considered as Identity (unit rotation)
 * @param[in] threshold (optionnal) threshold
 * @param[out] vec_inliers rotation labelled as inliers or outliers
 */
bool GlobalRotationsRobust(
  const RelativeRotations& RelRs,
  Matrix3x3Arr& Rs,
  const size_t nMainViewID,
  float threshold = 0.f,
  std::vector<bool> * vec_inliers = nullptr);

/**
 * @brief Implementation of Iteratively Reweighted Least Squares (IRLS) [1].
 *
 * @param[in] RelRs Relative weighted rotation matrices
 * @param[out] Rs output global rotation matrices
 * @param[in] nMainViewID Id of the image considered as Identity (unit rotation)
 * @param[in] sigma factor
 */
bool RefineRotationsAvgL1IRLS(
  const RelativeRotations& RelRs,
  Matrix3x3Arr& Rs,
  const size_t nMainViewID,
  REAL sigma=aliceVision::degreeToRadian(5.0));

/**
 * @brief Sort relative rotation as inlier, outlier rotations.
 *
 * @param[in] RelRs Relative weighted rotation matrices
 * @param[out] Rs output global rotation matrices
 * @param[in] threshold used to label rotations as inlier, or outlier (if 0, threshold is computed with the X84 law)
 * @param[in] vec_inliers inlier, outlier labels
 */
unsigned int FilterRelativeRotations(
  const RelativeRotations& RelRs,
  const Matrix3x3Arr& Rs,
  float threshold = 0.f,
  std::vector<bool> * vec_inliers = nullptr);


// Minimization Stuff

// L1RA [1] for dense A matrix
bool RobustRegressionL1PD(
  const Eigen::Matrix<REAL, Eigen::Dynamic, Eigen::Dynamic>& A,
  const Eigen::Matrix<REAL, Eigen::Dynamic, 1>& b,
  Eigen::Matrix<REAL, Eigen::Dynamic, 1>& x,
  REAL pdtol=1e-3, unsigned pdmaxiter=50);

// L1RA [1] for sparse A matrix
bool RobustRegressionL1PD(
  const Eigen::SparseMatrix<REAL, Eigen::ColMajor>& A,
  const Eigen::Matrix<REAL, Eigen::Dynamic, 1>& b,
  Eigen::Matrix<REAL, Eigen::Dynamic, 1>& x,
  REAL pdtol=1e-3, unsigned pdmaxiter=50);

/// IRLS [1] for dense A matrix
bool IterativelyReweightedLeastSquares(
  const Eigen::Matrix<REAL, Eigen::Dynamic, Eigen::Dynamic>& A,
  const Eigen::Matrix<REAL, Eigen::Dynamic, 1>& b,
  Eigen::Matrix<REAL, Eigen::Dynamic, 1>& x,
  REAL sigma, REAL eps=1e-5);

/// IRLS [1] for sparse A matrix
bool IterativelyReweightedLeastSquares(
  const Eigen::SparseMatrix<REAL, Eigen::ColMajor>& A,
  const Eigen::Matrix<REAL, Eigen::Dynamic, 1>& b,
  Eigen::Matrix<REAL, Eigen::Dynamic, 1>& x,
  REAL sigma, REAL eps=1e-5);

} // namespace l1
} // namespace rotationAveraging
} // namespace aliceVision
