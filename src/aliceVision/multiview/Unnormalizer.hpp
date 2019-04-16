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
 * @brief Unnormalize using Inverse
 */
struct UnnormalizerI
{
  /**
   * @brief Denormalize the results.
   * @see HZ page 109.
   */
  static void unnormalize(const Mat3& T1, const Mat3& T2, Mat3* H);
};

/**
 * @brief Unnormalize using Transpose
 */
struct UnnormalizerT
{
  /**
   * @brief Denormalize the results.
   * @see HZ page 109.
   */
  static void unnormalize(const Mat3& T1, const Mat3& T2, Mat3* H);
};

/**
 * @brief Unnormalize for resection
 */
struct UnnormalizerResection
{
  static void unnormalize(const Mat& T, const Mat& U, Mat34* P);
};

} //namespace multiview
} //namespace aliceVision
