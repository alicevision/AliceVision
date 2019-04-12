// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace multiview {

/**
 *
 * @brief 2D Affine transformation estimation
 *        This function can be used in order to estimate the affine transformation
 *        between two sets of points with known 2D correspondences.
 *
 * @param[in] x1 The first 2xN matrix of euclidean points
 * @param[in] x2 The second 2xN matrix of euclidean points
 * @param[out] M The 3x3 affine transformation matrix (6 dof)
 *          with the following parametrization
 *              |a b tx|
 *          M = |c d ty|
 *              |0 0 1 |
 *          such that    x2 = M * x1
 * @param[in] expected_precision The expected precision in order for instance
 *        to accept almost affine matrices.
 *
 * @return true if the transformation estimation has succeeded
 *
 * @note Need at least 3 non aligned points
 * @note Points coordinates must be normalized (euclidean)
 */
bool affine2DFromCorrespondencesLinear(const Mat& x1, const Mat& x2, Mat3* M, double expected_precision = EigenDoubleTraits::dummy_precision());

/**
 * @brief 3D Affine transformation estimation
 *        This function can be used in order to estimate the affine transformation
 *        between two sets of points with known 3D correspondences.
 *
 * @param[in] x1 The first 3xN matrix of euclidean points
 * @param[in] x2 The second 3xN matrix of euclidean points
 * @param[out] M The 3x3 affine transformation matrix (12 dof)
 *               with the following parametrization
 *              |a b c tx|
 *          M = |d e f ty|
 *              |g h i tz|
 *              |0 0 0 1 |
 *          such that    x2 = M * x1
 * @param[in] expected_precision The expected precision in order for instance
 *        to accept almost affine matrices.
 *
 * @return true if the transformation estimation has succeeded
 *
 * @note Need at least 4 non coplanar points
 * @note Points coordinates must be normalized (euclidean)
 */
bool affine3DFromCorrespondencesLinear(const Mat& x1, const Mat& x2, Mat4* M, double expected_precision = EigenDoubleTraits::dummy_precision());

} // namespace multiview
} // namespace aliceVision
