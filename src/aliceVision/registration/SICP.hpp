// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (C) 2013  LGG, EPFL
//   "Sparse Iterative Closest Point" by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

/// Sparse Iterative Closest Point (SparseICP)
/// C++ implementation for the paper:
///     "Sparse Iterative Closest Point"
///     Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///     Symposium on Geometry Processing 2013
///     Journal: Computer Graphics Forum.
/// Project webpage: http://lgg.epfl.ch/sparseicp

#include <Eigen/Dense>

#include "nanoflannKDTreeEigenAdaptor.hpp"
#include "rigidMotionEstimator.hpp"
#include "directSimilarityEstimator.hpp"


namespace aliceVision {
namespace registration {

/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {

struct Parameters {
    bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
    double p = 1.0;           /// p norm
    double mu = 10.0;         /// penalty weight
    double alpha = 1.2;       /// penalty increase factor
    double max_mu = 1e5;      /// max penalty
    int max_icp = 100;        /// max ICP iteration
    int max_outer = 100;      /// max outer iteration
    int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
    double stop = 1e-5;       /// stopping criteria
    bool useDirectSimilarity = false;
    bool print_icpn = false;  /// (debug) print ICP iteration
};
/// Shrinkage operator (Automatic loop unrolling using template)
template<unsigned int I>
inline double shrinkage(double mu, double n, double p, double s) {
    return shrinkage<I-1>(mu, n, p, 1.0 - (p/mu)*std::pow(n, p-2.0)*std::pow(s, p-1.0));
}
template<>
inline double shrinkage<0>(double, double, double, double s) {return s;}
/// 3D Shrinkage for point-to-point
template<unsigned int I>
inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p) {
    double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
    double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
    #pragma omp parallel for
    for(int i=0; i<Q.cols(); ++i) {
        double n = Q.col(i).norm();
        double w = 0.0;
        if(n > ha) w = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
        Q.col(i) *= w;
    }
}
/// 1D Shrinkage for point-to-plane
template<unsigned int I>
inline void shrink(Eigen::VectorXd& y, double mu, double p) {
    double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
    double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
    #pragma omp parallel for
    for(int i=0; i<y.rows(); ++i) {
        double n = std::abs(y(i));
        double s = 0.0;
        if(n > ha) s = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
        y(i) *= s;
    }
}
/// Sparse ICP with point to point
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Parameters
template <typename Derived1, typename Derived2>
Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                    Eigen::MatrixBase<Derived2>& Y,
                    Parameters par = Parameters()) {
    /// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    /// Buffers
    Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    Eigen::Affine3d finalTransform = Eigen::Affine3d::Identity();
    /// ICP
    for(int icp=0; icp<par.max_icp; ++icp) {
        if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i) {
            Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
        }
        /// Computer rotation and translation
        double mu = par.mu;
        for(int outer=0; outer<par.max_outer; ++outer) {
            double dual = 0.0;
            for(int inner=0; inner<par.max_inner; ++inner) {
                /// Z update (shrinkage)
                Z = X-Q+C/mu;
                shrink<3>(Z, mu, par.p);
                /// Rotation and translation update
                Eigen::Matrix3Xd U = Q+Z-C/mu;
                Eigen::Affine3d transform;
                if (par.useDirectSimilarity)
                    transform = directSimilarityEstimator::point_to_point(X, U);
                else
                    transform = rigidMotionEstimator::point_to_point(X, U);

                finalTransform = transform * finalTransform;
                /// Stopping criteria
                dual = (X-Xo1).colwise().norm().maxCoeff();
                Xo1 = X;
                if(dual < par.stop) break;
            }
            /// C update (lagrange multipliers)
            Eigen::Matrix3Xd P = X-Q-Z;
            if(!par.use_penalty) C.noalias() += mu*P;
            /// mu update (penalty)
            if(mu < par.max_mu) mu *= par.alpha;
            /// Stopping criteria
            double primal = P.colwise().norm().maxCoeff();
            if(primal < par.stop && dual < par.stop) break;
        }
        /// Stopping criteria
        double stop = (X-Xo2).colwise().norm().maxCoeff();
        Xo2 = X;
        if(stop < par.stop) break;
    }
    return finalTransform;
}
/// Sparse ICP with point to plane
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Target normals (one 3D normal per column)
/// @param Parameters
template <typename Derived1, typename Derived2, typename Derived3>
Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                    Eigen::MatrixBase<Derived2>& Y,
                    Eigen::MatrixBase<Derived3>& N,
                    Parameters par = Parameters()) {
    /// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    /// Buffers
    Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::VectorXd Z = Eigen::VectorXd::Zero(X.cols());
    Eigen::VectorXd C = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    Eigen::Affine3d finalTransform = Eigen::Affine3d::Identity();
    /// ICP
    for(int icp=0; icp<par.max_icp; ++icp) {
        if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;

        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i) {
            int id = kdtree.closest(X.col(i).data());
            Qp.col(i) = Y.col(id);
            Qn.col(i) = N.col(id);
        }
        /// Computer rotation and translation
        double mu = par.mu;
        for(int outer=0; outer<par.max_outer; ++outer) {
            double dual = 0.0;
            for(int inner=0; inner<par.max_inner; ++inner) {
                /// Z update (shrinkage)
                Z = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()+C.array()/mu;
                shrink<3>(Z, mu, par.p);
                /// Rotation and translation update
                Eigen::VectorXd U = Z-C/mu;
                Eigen::Affine3d transform = rigidMotionEstimator::point_to_plane(X, Qp, Qn, Eigen::VectorXd::Ones(X.cols()), U);
                finalTransform = transform * finalTransform;
                /// Stopping criteria
                dual = (X-Xo1).colwise().norm().maxCoeff();
                Xo1 = X;
                if(dual < par.stop) break;
            }
            /// C update (lagrange multipliers)
            Eigen::VectorXf P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
            if(!par.use_penalty) C.noalias() += mu*P;
            /// mu update (penalty)
            if(mu < par.max_mu) mu *= par.alpha;
            /// Stopping criteria
            double primal = P.array().abs().maxCoeff();
            if(primal < par.stop && dual < par.stop) break;
        }
        /// Stopping criteria
        double stop = (X-Xo2).colwise().norm().maxCoeff();
        Xo2 = X;
        if(stop < par.stop) break;
    }
    return finalTransform;
}

}
}
}
