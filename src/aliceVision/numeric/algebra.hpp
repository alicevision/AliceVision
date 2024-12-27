// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Solve the linear system Ax = 0 via SVD. Store the solution in x, such that
// ||x|| = 1.0. Return the singular value corresponding to the solution.
// Destroys A and resizes x if necessary.

// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2007 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// AliceVision does not support Eigen with alignment, unless C++17 aligned new feature is enabled.
// So ensure Eigen is used with the correct flags.
#ifndef ALICEVISION_EIGEN_REQUIRE_ALIGNMENT
    #ifndef EIGEN_MAX_ALIGN_BYTES
        #error "EIGEN_MAX_ALIGN_BYTES is not defined"
    #elif EIGEN_MAX_ALIGN_BYTES != 0
        #error "EIGEN_MAX_ALIGN_BYTES is defined but not 0"
    #endif

    #ifndef EIGEN_MAX_STATIC_ALIGN_BYTES
        #error "EIGEN_MAX_STATIC_ALIGN_BYTES is not defined"
    #elif EIGEN_MAX_STATIC_ALIGN_BYTES != 0
        #error "EIGEN_MAX_STATIC_ALIGN_BYTES is defined but not 0"
    #endif
#endif

#include <Eigen/Core>
#include <Eigen/SVD>

namespace aliceVision {

template<typename TMat, typename TVec>
double Nullspace(const TMat& A, TVec& nullspace)
{
    Eigen::JacobiSVD<TMat> svd(A, Eigen::ComputeFullV);
    const auto& vec = svd.singularValues();
    const auto& V = svd.matrixV();

    nullspace = V.col(V.cols() - 1);

    if (vec.size() < V.cols())
    {
        return 0.0;
    }

    return vec(vec.rows() - 1);
}

/// Solve the linear system Ax = 0 via SVD. Finds two solutions, x1 and x2, such
/// that x1 is the best solution and x2 is the next best solution (in the L2
/// norm sense). Store the solution in x1 and x2, such that ||x|| = 1.0. Return
/// the singular value corresponding to the solution x1. Destroys A and resizes
/// x if necessary.

template<typename TMat, typename TVec1, typename TVec2>
inline double Nullspace2(const TMat& A, TVec1& x1, TVec2& x2)
{
    Eigen::JacobiSVD<TMat> svd(A, Eigen::ComputeFullV);
    const auto& vec = svd.singularValues();
    const auto& V = svd.matrixV();

    x1 = V.col(V.cols() - 1);
    x2 = V.col(V.cols() - 2);

    if (vec.size() < V.cols())
    {
        return 0.0;
    }

    return vec(vec.rows() - 1);
}

template<size_t M, size_t N>
Eigen::Matrix<double, M * N, M * N> getJacobian_At_wrt_A()
{
    Eigen::Matrix<double, M * N, M * N> ret;

    /** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
    /** vec(IAtB) = kron(B.t, I) * vec(A) */
    /** dvec(IAtB)/dA = kron(B.t, I) * dvec(At)/dA */

    ret.fill(0);

    size_t pos_at = 0;
    for (size_t i = 0; i < M; i++)
    {
        for (size_t j = 0; j < N; j++)
        {
            size_t pos_a = N * j + i;
            ret(pos_at, pos_a) = 1;

            pos_at++;
        }
    }

    return ret;
}

template<size_t M, size_t N, size_t K>
Eigen::Matrix<double, M * K, M * N> getJacobian_AB_wrt_A(const Eigen::Matrix<double, M, N>& A, const Eigen::Matrix<double, N, K>& B)
{
    Eigen::Matrix<double, M * K, M * N> ret;

    /** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
    /** vec(IAB) = kron(B.t, I) * vec(A) */
    /** dvec(IAB)/dA = kron(B.t, I) * dvec(A)/dA */
    /** dvec(IAB)/dA = kron(B.t, I) */

    ret.fill(0);

    Eigen::Matrix<double, K, N> Bt = B.transpose();

    for (size_t row = 0; row < K; row++)
    {
        for (size_t col = 0; col < N; col++)
        {
            ret.template block<M, M>(row * M, col * M) = Bt(row, col) * Eigen::Matrix<double, M, M>::Identity();
        }
    }

    return ret;
}

template<size_t M, size_t N, size_t K>
Eigen::Matrix<double, M * K, M * N> getJacobian_AtB_wrt_A(const Eigen::Matrix<double, M, N>& A, const Eigen::Matrix<double, M, K>& B)
{
    return getJacobian_AB_wrt_A<M, N, K>(A.transpose(), B) * getJacobian_At_wrt_A<M, N>();
}

template<size_t M, size_t N, size_t K>
Eigen::Matrix<double, M * K, N * K> getJacobian_AB_wrt_B(const Eigen::Matrix<double, M, N>& A, const Eigen::Matrix<double, N, K>& B)
{
    Eigen::Matrix<double, M * K, N * K> ret;

    /** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
    /** vec(ABI) = kron(I, A) * vec(B) */
    /** dvec(ABI)/dB = kron(I, A) * dvec(B)/dB */
    /** dvec(ABI)/dB = kron(I, A) */

    ret.fill(0);

    for (size_t index = 0; index < K; index++)
    {
        ret.template block<M, N>(M * index, N * index) = A;
    }

    return ret;
}

}  // namespace aliceVision