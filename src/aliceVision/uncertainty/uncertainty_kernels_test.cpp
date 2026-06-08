// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE uncertainty_kernels

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

#include "uncertainty_kernels.hpp"

#include <cuda_runtime.h>

#include <cmath>
#include <limits>
#include <vector>

using namespace aliceVision::uncertainty;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace
{

/// Allocate device memory, copy host data in, return the device pointer.
double* toDevice(const std::vector<double>& host)
{
    double* d = nullptr;
    cudaMalloc(&d, host.size() * sizeof(double));
    cudaMemcpy(d, host.data(), host.size() * sizeof(double), cudaMemcpyHostToDevice);
    return d;
}

/// Copy a single device double back to the host.
double fromDevice(const double* d)
{
    double h = 0.0;
    cudaMemcpy(&h, d, sizeof(double), cudaMemcpyDeviceToHost);
    return h;
}

/// Allocate a device scalar and initialise it to val.
double* deviceScalar(double val)
{
    double* d = nullptr;
    cudaMalloc(&d, sizeof(double));
    cudaMemcpy(d, &val, sizeof(double), cudaMemcpyHostToDevice);
    return d;
}

} // namespace

// ---------------------------------------------------------------------------
// launchMaxAbsReduction tests
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(MaxAbsReduction_AllPositive)
{
    // max(|v|) = 5.0
    const std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, static_cast<int>(data.size()), nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_CLOSE(fromDevice(d_result), 5.0, 1e-10);

    cudaFree(d_data);
    cudaFree(d_result);
}

BOOST_AUTO_TEST_CASE(MaxAbsReduction_AllNegative)
{
    // max(|v|) = 7.0
    const std::vector<double> data = {-7.0, -3.0, -1.0};
    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, static_cast<int>(data.size()), nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_CLOSE(fromDevice(d_result), 7.0, 1e-10);

    cudaFree(d_data);
    cudaFree(d_result);
}

BOOST_AUTO_TEST_CASE(MaxAbsReduction_MixedSigns)
{
    // max(|v|) = 6.0 (from -6.0)
    const std::vector<double> data = {1.0, -6.0, 4.0, -2.0, 5.0};
    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, static_cast<int>(data.size()), nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_CLOSE(fromDevice(d_result), 6.0, 1e-10);

    cudaFree(d_data);
    cudaFree(d_result);
}

BOOST_AUTO_TEST_CASE(MaxAbsReduction_SingleElement)
{
    const std::vector<double> data = {-42.5};
    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, 1, nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_CLOSE(fromDevice(d_result), 42.5, 1e-10);

    cudaFree(d_data);
    cudaFree(d_result);
}

BOOST_AUTO_TEST_CASE(MaxAbsReduction_LargeArray)
{
    // Fill with values 1..N; max = N
    const int N = 100'000;
    std::vector<double> data(N);
    for (int i = 0; i < N; ++i)
        data[i] = static_cast<double>(i + 1);

    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, N, nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_CLOSE(fromDevice(d_result), static_cast<double>(N), 1e-10);

    cudaFree(d_data);
    cudaFree(d_result);
}

BOOST_AUTO_TEST_CASE(MaxAbsReduction_AllZeros)
{
    const std::vector<double> data(64, 0.0);
    double* d_data   = toDevice(data);
    double* d_result = deviceScalar(0.0);

    launchMaxAbsReduction(d_data, d_result, static_cast<int>(data.size()), nullptr);
    cudaDeviceSynchronize();

    BOOST_CHECK_SMALL(fromDevice(d_result), 1e-15);

    cudaFree(d_data);
    cudaFree(d_result);
}

// ---------------------------------------------------------------------------
// launchConditionalAxpy tests
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(ConditionalAxpy_ConvergingAccumulates)
{
    // lambda * cur_maxabs (0.5) <= prev_norm (1.0)  => axpy should execute.
    // dst[i] = 1.0, src[i] = 2.0, k = 0.5  =>  dst[i] += 0.5 * 2.0 = 2.0
    const int n = 4;
    const std::vector<double> src(n, 2.0);
    const std::vector<double> dst_init(n, 1.0);

    double* d_src         = toDevice(src);
    double* d_dst         = toDevice(dst_init);
    double* d_cur_maxabs  = deviceScalar(0.5);   // lambda * 0.5 = 0.25 <= prev_norm 1.0
    double* d_prev_norm   = deviceScalar(1.0);

    const double lambda = 0.5;
    const double k      = 0.5;

    launchConditionalAxpy(k, d_src, d_dst, d_cur_maxabs, d_prev_norm, lambda, n, nullptr);
    cudaDeviceSynchronize();

    // axpy executed: dst[i] = 1.0 + 0.5 * 2.0 = 2.0
    std::vector<double> result(n);
    cudaMemcpy(result.data(), d_dst, n * sizeof(double), cudaMemcpyDeviceToHost);
    for (int i = 0; i < n; ++i)
        BOOST_CHECK_CLOSE(result[i], 2.0, 1e-10);

    // d_prev_norm updated to lambda * cur_maxabs = 0.5 * 0.5 = 0.25
    BOOST_CHECK_CLOSE(fromDevice(d_prev_norm), 0.25, 1e-10);

    cudaFree(d_src);
    cudaFree(d_dst);
    cudaFree(d_cur_maxabs);
    cudaFree(d_prev_norm);
}

BOOST_AUTO_TEST_CASE(ConditionalAxpy_DivergingSkips)
{
    // lambda * cur_maxabs (10.0) = 5.0 > prev_norm (1.0)  => axpy must be skipped.
    const int n = 4;
    const std::vector<double> src(n, 2.0);
    const std::vector<double> dst_init(n, 1.0);

    double* d_src         = toDevice(src);
    double* d_dst         = toDevice(dst_init);
    double* d_cur_maxabs  = deviceScalar(10.0);  // lambda * 10.0 = 5.0 > prev_norm 1.0
    double* d_prev_norm   = deviceScalar(1.0);

    const double lambda = 0.5;
    const double k      = 0.5;

    launchConditionalAxpy(k, d_src, d_dst, d_cur_maxabs, d_prev_norm, lambda, n, nullptr);
    cudaDeviceSynchronize();

    // axpy skipped: dst[i] remains 1.0
    std::vector<double> result(n);
    cudaMemcpy(result.data(), d_dst, n * sizeof(double), cudaMemcpyDeviceToHost);
    for (int i = 0; i < n; ++i)
        BOOST_CHECK_CLOSE(result[i], 1.0, 1e-10);

    // d_prev_norm must NOT have been updated (still 1.0)
    BOOST_CHECK_CLOSE(fromDevice(d_prev_norm), 1.0, 1e-10);

    cudaFree(d_src);
    cudaFree(d_dst);
    cudaFree(d_cur_maxabs);
    cudaFree(d_prev_norm);
}

BOOST_AUTO_TEST_CASE(ConditionalAxpy_InitialInfAlwaysPasses)
{
    // d_prev_norm = +inf  =>  any lambda * cur_maxabs passes on the first call.
    const int n = 4;
    const std::vector<double> src(n, 3.0);
    const std::vector<double> dst_init(n, 0.0);

    double* d_src         = toDevice(src);
    double* d_dst         = toDevice(dst_init);
    double* d_cur_maxabs  = deviceScalar(100.0);
    double* d_prev_norm   = deviceScalar(std::numeric_limits<double>::infinity());

    const double lambda = 1e-3;
    const double k      = 2.0;

    launchConditionalAxpy(k, d_src, d_dst, d_cur_maxabs, d_prev_norm, lambda, n, nullptr);
    cudaDeviceSynchronize();

    // axpy executed: dst[i] = 0.0 + 2.0 * 3.0 = 6.0
    std::vector<double> result(n);
    cudaMemcpy(result.data(), d_dst, n * sizeof(double), cudaMemcpyDeviceToHost);
    for (int i = 0; i < n; ++i)
        BOOST_CHECK_CLOSE(result[i], 6.0, 1e-10);

    // d_prev_norm updated to lambda * cur_maxabs = 1e-3 * 100.0 = 0.1
    BOOST_CHECK_CLOSE(fromDevice(d_prev_norm), 0.1, 1e-10);

    cudaFree(d_src);
    cudaFree(d_dst);
    cudaFree(d_cur_maxabs);
    cudaFree(d_prev_norm);
}

BOOST_AUTO_TEST_CASE(ConditionalAxpy_ExactEqualityPasses)
{
    // lambda * cur_maxabs == prev_norm (exactly equal) must also accumulate.
    const int n = 2;
    const std::vector<double> src(n, 1.0);
    const std::vector<double> dst_init(n, 0.0);

    // lambda=0.5, cur=2.0  =>  lambda*cur = 1.0 == prev_norm=1.0  => pass
    double* d_src         = toDevice(src);
    double* d_dst         = toDevice(dst_init);
    double* d_cur_maxabs  = deviceScalar(2.0);
    double* d_prev_norm   = deviceScalar(1.0);

    launchConditionalAxpy(1.0, d_src, d_dst, d_cur_maxabs, d_prev_norm, 0.5, n, nullptr);
    cudaDeviceSynchronize();

    std::vector<double> result(n);
    cudaMemcpy(result.data(), d_dst, n * sizeof(double), cudaMemcpyDeviceToHost);
    for (int i = 0; i < n; ++i)
        BOOST_CHECK_CLOSE(result[i], 1.0, 1e-10);

    cudaFree(d_src);
    cudaFree(d_dst);
    cudaFree(d_cur_maxabs);
    cudaFree(d_prev_norm);
}
