// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (C) 2013  LGG, EPFL
//   "Sparse Iterative Closest Point" by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <Eigen/Dense>

#include "nanoflannKDTreeEigenAdaptor.hpp"
#include "rigidMotionEstimator.hpp"
#include "directSimilarityEstimator.hpp"


namespace aliceVision {
namespace registration {

/// ICP implementation using iterative reweighting
namespace ICP {

enum Function {
    PNORM,
    TUKEY,
    FAIR,
    LOGISTIC,
    TRIMMED,
    NONE
};
struct Parameters {
    Function f = NONE;    /// robust function type
    double p = 1.0; // 0.1;       /// paramter of the robust function
    int max_icp = 100;    /// max ICP iteration
    int max_outer = 100;  /// max outer iteration
    double stop = 1e-5;   /// stopping criteria
    bool useDirectSimilarity = false; /// use direct similarity instead of rigid motion
};
/// Weight functions
/// @param Residuals
/// @param Parameter
void uniform_weight(Eigen::VectorXd& r) {
    r = Eigen::VectorXd::Ones(r.rows());
}
/// @param Residuals
/// @param Parameter
void pnorm_weight(Eigen::VectorXd& r, double p, double reg=1e-8) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = p/(std::pow(r(i),2-p) + reg);
    }
}
/// @param Residuals
/// @param Parameter
void tukey_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        if(r(i) > p) r(i) = 0.0;
        else r(i) = std::pow((1.0 - std::pow(r(i)/p,2.0)), 2.0);
    }
}
/// @param Residuals
/// @param Parameter
void fair_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = 1.0/(1.0 + r(i)/p);
    }
}
/// @param Residuals
/// @param Parameter
void logistic_weight(Eigen::VectorXd& r, double p) {
    for(int i=0; i<r.rows(); ++i) {
        r(i) = (p/r(i))*std::tanh(r(i)/p);
    }
}
struct sort_pred {
    bool operator()(const std::pair<int,double> &left,
                    const std::pair<int,double> &right) {
        return left.second < right.second;
    }
};
/// @param Residuals
/// @param Parameter
void trimmed_weight(Eigen::VectorXd& r, double p) {
    std::vector<std::pair<int, double> > sortedDist(r.rows());
    for(int i=0; i<r.rows(); ++i) {
        sortedDist[i] = std::pair<int, double>(i,r(i));
    }
    std::sort(sortedDist.begin(), sortedDist.end(), sort_pred());
    r.setZero();
    int nbV = r.rows()*p;
    for(int i=0; i<nbV; ++i) {
        r(sortedDist[i].first) = 1.0;
    }
}
/// @param Function type
/// @param Residuals
/// @param Parameter
void robust_weight(Function f, Eigen::VectorXd& r, double p) {
    switch(f) {
        case PNORM: pnorm_weight(r,p); break;
        case TUKEY: tukey_weight(r,p); break;
        case FAIR: fair_weight(r,p); break;
        case LOGISTIC: logistic_weight(r,p); break;
        case TRIMMED: trimmed_weight(r,p); break;
        case NONE: uniform_weight(r); break;
        default: uniform_weight(r); break;
    }
}
/// Reweighted ICP with point to point
/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Parameters
Eigen::Affine3d point_to_point(Eigen::Matrix3Xd& X,
                    Eigen::Matrix3Xd& Y,
                    Parameters par = Parameters()) {
    /// Build kd-tree
    nanoflann::KDTreeAdaptor<Eigen::Matrix3Xd, 3, nanoflann::metric_L2_Simple> kdtree(Y);
    /// Buffers
    Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
    Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    Eigen::Affine3d finalTransform = Eigen::Affine3d::Identity();
    /// ICP
    for(int icp=0; icp<par.max_icp; ++icp) {
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i) {
            Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
        }
        /// Computer rotation and translation
        for(int outer=0; outer<par.max_outer; ++outer) {
            /// Compute weights
            W = (X-Q).colwise().norm();
            robust_weight(par.f, W, par.p);
            /// Rotation and translation update
            Eigen::Affine3d transform;
            if(par.useDirectSimilarity)
                transform = directSimilarityEstimator::point_to_point(X, Q, W);
            else
                transform = rigidMotionEstimator::point_to_point(X, Q, W);
            std::cout << "Intermediate transform:\n" << transform.matrix() << std::endl;
            finalTransform = transform * finalTransform;
            /// Stopping criteria
            double stop1 = (X-Xo1).colwise().norm().maxCoeff();
            Xo1 = X;
            if(stop1 < par.stop)
            {
                std::cout << "stop1 < par.stop:\n" << stop1 << ", " << par.stop << std::endl;
                break;
            }
        }
        /// Stopping criteria
        double stop2 = (X-Xo2).colwise().norm().maxCoeff();
        Xo2 = X;
        if(stop2 < par.stop)
        {
            std::cout << "stop2 < par.stop:\n" << stop2 << ", " << par.stop << std::endl;
            std::cout << "icp iteration:\n" << icp << std::endl;
            break;
        }
    }
    return finalTransform;
}
/// Reweighted ICP with point to plane
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
    Eigen::VectorXd W = Eigen::VectorXd::Zero(X.cols());
    Eigen::Matrix3Xd Xo1 = X;
    Eigen::Matrix3Xd Xo2 = X;
    Eigen::Affine3d finalTransform = Eigen::Affine3d::Identity();
    /// ICP
    for(int icp=0; icp<par.max_icp; ++icp) {
        /// Find closest point
        #pragma omp parallel for
        for(int i=0; i<X.cols(); ++i) {
            int id = kdtree.closest(X.col(i).data());
            Qp.col(i) = Y.col(id);
            Qn.col(i) = N.col(id);
        }
        /// Computer rotation and translation
        for(int outer=0; outer<par.max_outer; ++outer) {
            /// Compute weights
            W = (Qn.array()*(X-Qp).array()).colwise().sum().abs().transpose();
            robust_weight(par.f, W, par.p);
            /// Rotation and translation update
            Eigen::Affine3d transform = rigidMotionEstimator::point_to_plane(X, Qp, Qn, W);
            finalTransform = transform * finalTransform;
            /// Stopping criteria
            double stop1 = (X-Xo1).colwise().norm().maxCoeff();
            Xo1 = X;
            if(stop1 < par.stop) break;
        }
        /// Stopping criteria
        double stop2 = (X-Xo2).colwise().norm().maxCoeff() ;
        Xo2 = X;
        if(stop2 < par.stop) break;
    }
    return finalTransform;
}

}
}
}
