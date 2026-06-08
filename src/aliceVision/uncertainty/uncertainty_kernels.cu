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
// Thread 0 in block 0 decides whether to execute the axpy.
// The decision is broadcast via shared memory so all threads in the grid
// read it from L1 cache rather than global memory.
//
// Layout: one kernel launch with enough blocks to cover n elements.
// Block 0 additionally runs the comparison and update logic in thread 0.

__global__ void conditionalAxpyKernel(double k,
                                       const double* __restrict__ src,
                                       double* __restrict__ dst,
                                       const double* d_cur_maxabs,
                                       double* d_prev_norm,
                                       double lambda,
                                       int n)
{
    // Block 0, thread 0: compare and update d_prev_norm
    __shared__ int do_axpy;
    if (blockIdx.x == 0 && threadIdx.x == 0)
    {
        double cur = lambda * (*d_cur_maxabs);
        do_axpy = (cur <= *d_prev_norm) ? 1 : 0;
        if (do_axpy)
        {
            *d_prev_norm = cur;
        }
    }

    // All threads in block 0 wait for the decision; other blocks read it
    // from global memory via the shared flag after block 0 has written it.
    // Since blocks can execute in any order we use a device-side broadcast:
    // block 0 writes do_axpy to d_cur_maxabs[1] (we repurpose a spare slot)
    // — but that requires an extra device buffer.
    //
    // Simpler correct approach: re-read d_prev_norm comparison in every block.
    // (One extra global read per block, negligible vs the axpy bandwidth.)
    __syncthreads(); // sync within block 0

    // Non-block-0 threads recompute the flag from global memory
    int local_do_axpy;
    if (blockIdx.x == 0)
    {
        local_do_axpy = do_axpy;
    }
    else
    {
        // d_prev_norm was updated by block 0 only if do_axpy; check the
        // pre-update value by comparing directly (re-read is safe: block 0
        // has already written d_prev_norm before this block can read it
        // because grids execute sequentially per stream).
        local_do_axpy = (lambda * (*d_cur_maxabs) <= *d_prev_norm) ? 1 : 0;
    }

    if (!local_do_axpy) return;

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
                            double lambda, int n, cudaStream_t stream)
{
    const int blocks = (n + BLOCK_SIZE - 1) / BLOCK_SIZE;
    conditionalAxpyKernel<<<blocks, BLOCK_SIZE, 0, stream>>>(
        k, d_src, d_dst, d_cur_maxabs, d_prev_norm, lambda, n);
}

} // namespace uncertainty
} // namespace aliceVision
