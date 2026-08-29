// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>

namespace aliceVision
{
namespace uncertainty
{

/**
 * @brief Compute the maximum absolute value of a device array and write it to a device scalar.
 *
 * Launches a parallel reduction over @p n elements of @p d_data.
 * The result is written to @p d_result (device pointer, 1 element).
 * Must be called on @p stream so it is sequenced with other MAGMA operations.
 */
void launchMaxAbsReduction(const double* d_data, double* d_result, int n, cudaStream_t stream);

/**
 * @brief Conditionally accumulate k * d_src into d_dst on GPU, skipping when the series diverges.
 *
 * The guard computes cur = abs(k) * (*d_cur_maxabs).
 * If cur <= *d_prev_norm the axpy is performed (series still converging) and *d_prev_norm is
 * updated to cur. Otherwise the axpy is skipped and *d_prev_norm is left unchanged so that
 * subsequent iterations also fail the check and are also skipped.
 *
 * Both d_cur_maxabs and d_prev_norm are device pointers (1 double each).
 * Must be called on @p stream so it is sequenced with the preceding maxAbsReduction kernel.
 */
void launchConditionalAxpy(double k, const double* d_src, double* d_dst,
                           const double* d_cur_maxabs, double* d_prev_norm,
                           int n, cudaStream_t stream);

} // namespace uncertainty
} // namespace aliceVision
