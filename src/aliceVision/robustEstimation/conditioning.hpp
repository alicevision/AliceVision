// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

/**
 * @brief Implementation of normalized coordinates.
 *        Normalization improve accuracy of results and provide benefits
 *        that make scale and coordinate origin invariant.
 *        The implementation follows @see Algorithm 4.2 from HZ page 109.
 */

namespace aliceVision {
namespace robustEstimation {

/**
 * @brief Point conditioning
 */
void preconditionerFromPoints(const Mat& points, Mat3* T);

/**
 * @brief Normalize input point for a given T transform matrix
 */
void applyTransformationToPoints(const Mat& points, const Mat3& T, Mat* transformed_points);

/**
 * @brief Normalize point in [-.5, .5] and return transformation matrix
 */
void normalizePoints(const Mat& points, Mat* normalized_points, Mat3* T);

/**
 * @brief Point conditioning (compute Transformation matrix)
 */
void preconditionerFromImageSize(int width, int height, Mat3 *T);

/**
 * @brief Normalize point rom image coordinates to [-.5, .5]
 */
void normalizePointsFromImageSize(const Mat& points, Mat* normalized_points, Mat3* T, int width, int height);

} //namespace robustEstimation
} //namespace aliceVision
