// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "uncertainty_kernels.hpp"

#include <cuda_runtime.h>

namespace aliceVision
{
namespace uncertainty
{

// ---------------------------------------------------------------------------
// Max-abs reduction kernel
// ---------------------------------------------------------------------------
// One block per tile of BLOCK_SIZE elements; each block reduces to a single
// value via shared memory, then atomically updates the global result.
// The caller must zero d_result before launching.

static constexpr int BLOCK_SIZE = 256;

// atomicMax for doubles via CAS loop (not natively supported before Volta)
__device__ __forceinline__ void atomicMaxDouble(double* addr, double val)
{
    unsigned long long* addr_ull = reinterpret_cast<unsigned long long*>(addr);
    unsigned long long old_ull = *addr_ull;
    unsigned long long new_ull;
    do
    {
        double old_val = __longlong_as_double(static_cast<long long>(old_ull));
        if (old_val >= val) return;
        new_ull = static_cast<unsigned long long>(__double_as_longlong(val));
        old_ull = atomicCAS(addr_ull, old_ull, new_ull);
    } while (__longlong_as_double(static_cast<long long>(old_ull)) < val);
}

__global__ void maxAbsReductionKernel(const double* __restrict__ data, double* result, int n)
{
    __shared__ double sdata[BLOCK_SIZE];

    const int tid  = threadIdx.x;
    const int gid  = blockIdx.x * blockDim.x + tid;

    sdata[tid] = (gid < n) ? fabs(data[gid]) : 0.0;
    __syncthreads();

    // Tree reduction within block
    for (int s = blockDim.x / 2; s > 0; s >>= 1)
    {
        if (tid < s)
            sdata[tid] = fmax(sdata[tid], sdata[tid + s]);
        __syncthreads();
    }

    if (tid == 0)
        atomicMaxDouble(result, sdata[0]);
}

// ---------------------------------------------------------------------------
// Conditional axpy kernel
// ---------------------------------------------------------------------------
// A scalar pre-kernel updates d_prev_norm when the current term is accepted.
// The axpy kernel then reads only d_prev_norm, relying on stream ordering
// between kernel launches instead of inter-block synchronization.
//
// Layout: one scalar kernel launch followed by one axpy kernel launch with
// enough blocks to cover n elements.

__global__ void updatePreviousNormKernel(double k, const double* d_cur_maxabs, double* d_prev_norm)
{
    const double termNorm = fabs(k) * (*d_cur_maxabs);
    if (termNorm <= *d_prev_norm)
    {
        *d_prev_norm = termNorm;
    }
}

__global__ void conditionalAxpyKernel(double k,
                                       const double* __restrict__ src,
                                       double* __restrict__ dst,
                                       const double* d_cur_maxabs,
                                       const double* d_prev_norm,
                                       int n)
{
    const double termNorm = fabs(k) * (*d_cur_maxabs);
    if (!(termNorm <= *d_prev_norm)) return;

    const int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n)
        dst[idx] += k * src[idx];
}

// ---------------------------------------------------------------------------
// Host-callable wrappers
// ---------------------------------------------------------------------------

void launchMaxAbsReduction(const double* d_data, double* d_result, int n, cudaStream_t stream)
{
    // Zero the result first (done via a small cudaMemsetAsync)
    cudaMemsetAsync(d_result, 0, sizeof(double), stream);

    const int blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    maxAbsReductionKernel<<<blocks, BLOCK_SIZE, 0, stream>>>(d_data, d_result, n);
}

void launchConditionalAxpy(double k, const double* d_src, double* d_dst,
                            const double* d_cur_maxabs, double* d_prev_norm,
                            int n, cudaStream_t stream)
{
    if (n <= 0) return;

    const int blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;

    updatePreviousNormKernel<<<1, 1, 0, stream>>>(k, d_cur_maxabs, d_prev_norm);
    conditionalAxpyKernel<<<blocks, BLOCK_SIZE, 0, stream>>>(
        k, d_src, d_dst, d_cur_maxabs, d_prev_norm, n);
}

} // namespace uncertainty
} // namespace aliceVision
