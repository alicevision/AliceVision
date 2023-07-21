// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (C) 2013  LGG, EPFL
//   "Sparse Iterative Closest Point" by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

namespace aliceVision {
namespace registration {

/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace rigidMotionEstimator {

/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Confidence weights
template <typename Derived1, typename Derived2, typename Derived3>
Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                               Eigen::MatrixBase<Derived2>& Y,
                               const Eigen::MatrixBase<Derived3>& w) {
    /// Normalize weight vector
    Eigen::VectorXd w_normalized = w/w.sum();
    /// De-mean
    Eigen::Vector3d X_mean, Y_mean;
    for(int i=0; i<3; ++i) {
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
    }
    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
        Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
        transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    } else {
        transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
    }
    transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
    /// Apply transformation
    X = transformation*X;
    /// Re-apply mean
    X.colwise() += X_mean;
    Y.colwise() += Y_mean;
    /// Return transformation
    return transformation;
}
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
template <typename Derived1, typename Derived2>
inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                      Eigen::MatrixBase<Derived2>& Y) {
    return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
}
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Confidence weights
/// @param Right hand side
template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                               Eigen::MatrixBase<Derived2>& Y,
                               Eigen::MatrixBase<Derived3>& N,
                               const Eigen::MatrixBase<Derived4>& w,
                               const Eigen::MatrixBase<Derived5>& u) {
    typedef Eigen::Matrix<double, 6, 6> Matrix66;
    typedef Eigen::Matrix<double, 6, 1> Vector6;
    typedef Eigen::Block<Matrix66, 3, 3> Block33;
    /// Normalize weight vector
    Eigen::VectorXd w_normalized = w/w.sum();
    /// De-mean
    Eigen::Vector3d X_mean;
    for(int i=0; i<3; ++i)
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
    X.colwise() -= X_mean;
    Y.colwise() -= X_mean;
    /// Prepare LHS and RHS
    Matrix66 LHS = Matrix66::Zero();
    Vector6 RHS = Vector6::Zero();
    Block33 TL = LHS.topLeftCorner<3,3>();
    Block33 TR = LHS.topRightCorner<3,3>();
    Block33 BR = LHS.bottomRightCorner<3,3>();
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
    #pragma omp parallel
    {
        #pragma omp for
        for(int i=0; i<X.cols(); i++) {
            C.col(i) = X.col(i).cross(N.col(i));
        }
        #pragma omp sections nowait
        {
            #pragma omp section
            for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
            #pragma omp section
            for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
            #pragma omp section
            for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
            #pragma omp section
            for(int i=0; i<C.cols(); i++) {
                double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
                RHS.head<3>() += C.col(i)*dist_to_plane;
                RHS.tail<3>() += N.col(i)*dist_to_plane;
            }
        }
    }
    LHS = LHS.selfadjointView<Eigen::Upper>();
    /// Compute transformation
    Eigen::Affine3d transformation;
    Eigen::LDLT<Matrix66> ldlt(LHS);
    RHS = ldlt.solve(RHS);
    transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
    transformation.translation() = RHS.tail<3>();
    /// Apply transformation
    X = transformation*X;
    /// Re-apply mean
    X.colwise() += X_mean;
    Y.colwise() += X_mean;
    /// Return transformation
    return transformation;
}
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Confidence weights
template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                      Eigen::MatrixBase<Derived2>& Yp,
                                      Eigen::MatrixBase<Derived3>& Yn,
                                      const Eigen::MatrixBase<Derived4>& w) {
    return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
}

}
}
}
