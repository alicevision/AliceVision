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
#ifndef SWIG
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
#endif

//--
// Eigen
// http://eigen.tuxfamily.org/dox-devel/QuickRefPage.html
//--
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SparseCore>


#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>

namespace aliceVision {

// Check MSVC
#if _WIN32 || _WIN64
    #if _WIN64
        #define ENV64BIT
    #else
        #define ENV32BIT
    #endif
#endif

// Check GCC
#if __GNUC__
    #if __x86_64__ || __ppc64__ || _LP64
        #define ENV64BIT
    #else
        #define ENV32BIT
    #endif
#endif

using Eigen::Map;

using EigenDoubleTraits = Eigen::NumTraits<double>;

using Vec3 = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;
using Vec3f = Eigen::Vector3f;

using Vec2i = Eigen::Vector2i;
using Vec2f = Eigen::Vector2f;

using Vec9 = Eigen::Matrix<double, 9, 1>;

using Quaternion = Eigen::Quaternion<double>;

using Mat3 = Eigen::Matrix<double, 3, 3>;

#if defined(ENV32BIT)
using Mat23 = Eigen::Matrix<double, 2, 3, Eigen::DontAlign>;
using Mat34 = Eigen::Matrix<double, 3, 4, Eigen::DontAlign>;
using Vec2 = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;
using Vec4 = Eigen::Matrix<double, 4, 1, Eigen::DontAlign>;
using Vec6 = Eigen::Matrix<double, 6, 1, Eigen::DontAlign>;
#else  // 64 bits compiler
using Mat23 = Eigen::Matrix<double, 2, 3>;
using Mat34 = Eigen::Matrix<double, 3, 4>;
using Vec2 = Eigen::Vector2d;
using Vec4 = Eigen::Vector4d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
#endif

using Mat4 = Eigen::Matrix<double, 4, 4>;
using Matu = Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>;

using RMat3 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

//-- General purpose Matrix and Vector
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
using Vecu = Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>;
using Matf = Eigen::MatrixXf;
using Vecf = Eigen::VectorXf;
using Vecb = Eigen::Matrix<bool, Eigen::Dynamic, 1>;

using Mat2X = Eigen::Matrix<double, 2, Eigen::Dynamic>;
using Mat3X = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using Mat4X = Eigen::Matrix<double, 4, Eigen::Dynamic>;

using MatX9 = Eigen::Matrix<double, Eigen::Dynamic, 9>;
using Mat9 = Eigen::Matrix<double, 9, 9>;

//-- Sparse Matrix (Column major, and row major)
using sMat = Eigen::SparseMatrix<double>;
using sRMat = Eigen::SparseMatrix<double, Eigen::RowMajor>;


}  // namespace aliceVision

#include <aliceVision/numeric/NumericFunctions.hpp>