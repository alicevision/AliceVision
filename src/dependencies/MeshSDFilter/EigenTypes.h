// BSD 3-Clause License
//
// Copyright (c) 2017, Bailin Deng
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef EIGENTYPES_H
#define EIGENTYPES_H

#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace SDFilter
{

// Define eigen matrix types
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3X;
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> Matrix2X;
typedef Eigen::Matrix<int, 2, Eigen::Dynamic> Matrix2Xi;
typedef Eigen::Matrix<double, Eigen::Dynamic, 3> MatrixX3;
typedef Eigen::Matrix<double, Eigen::Dynamic, 2> MatrixX2;
typedef Eigen::Matrix<int, 3, Eigen::Dynamic> Matrix3Xi;
typedef Eigen::Matrix<Eigen::Index, 2, Eigen::Dynamic> Matrix2XIdx;
typedef Eigen::Matrix<Eigen::Index, Eigen::Dynamic, 1> VectorXIdx;
typedef Eigen::SparseMatrix<double> SparseMatrixXd;
typedef Eigen::Triplet<double> Triplet;

// Conversion between a 3d vector type to Eigen::Vector3d
template<typename Vec_T>
inline Eigen::Vector3d to_eigen_vec3d(const Vec_T &vec)
{
    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}


template<typename Vec_T>
inline Vec_T from_eigen_vec3d(const Eigen::Vector3d &vec)
{
    Vec_T v;
    v[0] = vec(0);
    v[1] = vec(1);
    v[2] = vec(2);

    return v;
}

}


#endif // EIGENTYPES_H
