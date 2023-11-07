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

}  // namespace aliceVision