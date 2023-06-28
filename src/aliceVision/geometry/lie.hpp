// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <Eigen/Dense>

namespace aliceVision {

namespace SO2 {

using Matrix = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>;

/**
 * @brief Compute the exponential map of the given algebra on the group.
 * @param algebra the 1D vector
 * @return a 2*2 S0(2) matrix
 */
inline Eigen::Matrix2d expm(double algebra) {
    Eigen::Matrix2d ret;

    ret(0, 0) = cos(algebra);
    ret(0, 1) = -sin(algebra);
    ret(1, 0) = sin(algebra);
    ret(1, 1) = cos(algebra);

    return ret;
}
} //namespace SO2

namespace SO3 {

using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

/**
 * @brief Compute the skew symmetric matrix of the given vector 3D.
 * @param in the 3D vector
 * @return a skew symmetric matrix
 */
inline Eigen::Matrix3d skew(const Eigen::Vector3d& in) {
    Eigen::Matrix3d ret;

    ret.fill(0);

    ret(0, 1) = -in(2);
    ret(1, 0) = in(2);
    ret(0, 2) = in(1);
    ret(2, 0) = -in(1);
    ret(1, 2) = -in(0);
    ret(2, 1) = in(0);

    return ret;
}

/**
 * @brief Compute the exponential map of the given algebra on the group.
 * @param algebra the 3D vector
 * @return a 3*3 SO(3) matrix
 */
inline Eigen::Matrix3d expm(const Eigen::Vector3d& algebra) {
    const double angle = algebra.norm();

    if (angle < std::numeric_limits<double>::epsilon()) {
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Matrix3d omega = skew(algebra);

    Eigen::Matrix3d ret;
    ret = Eigen::Matrix3d::Identity() + (sin(angle) / angle) * omega + ((1.0 - cos(angle)) / (angle * angle)) * omega * omega;

    return ret;
}

/**
 * @brief Compute the algebra related to a given rotation matrix.
 * @param R the input rotation matrix
 * @return the algebra
 */
inline Eigen::Vector3d logm(const Eigen::Matrix3d& R) {
    Eigen::Vector3d ret;

    const double p1 = R(2, 1) - R(1, 2);
    const double p2 = R(0, 2) - R(2, 0);
    const double p3 = R(1, 0) - R(0, 1);

    double costheta = (R.trace() - 1.0) / 2.0;
    if (costheta < -1.0) {
        costheta = -1.0;
    }

    if (costheta > 1.0) {
        costheta = 1.0;
    }

    if (1.0 - costheta < 1e-24) {
        ret.fill(0);
        return ret;
    }

    const double theta = acos(costheta);
    const double scale = theta / (2.0 * sin(theta));

    ret(0) = scale * p1;
    ret(1) = scale * p2;
    ret(2) = scale * p3;

    return ret;
}
} //namespace SO3


namespace SE3 {

using Matrix = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;

/**
 * @brief Compute the exponential map of the given algebra on the group.
 * @param algebra the 6D vector
 * @return a 4*4 SE(3) matrix
 */
inline Eigen::Matrix4d expm(const Eigen::Matrix<double, 6, 1>& algebra) {
    Eigen::Matrix4d ret;
    ret.setIdentity();

    const Eigen::Vector3d vecR = algebra.block<3, 1>(0, 0);
    const Eigen::Vector3d vecT = algebra.block<3, 1>(3, 0);

    double angle = vecR.norm();
    if (angle < std::numeric_limits<double>::epsilon()) {
        ret.setIdentity();
        ret.block<3, 1>(0, 3) = vecT;
        return ret;
    }

    const Eigen::Matrix3d omega = SO3::skew(vecR);
    const Eigen::Matrix3d V = Eigen::Matrix3d::Identity() + ((1.0 - cos(angle)) / (angle * angle))
        * omega + ((angle - sin(angle)) / (angle * angle * angle)) * omega * omega;

    ret.block<3, 3>(0, 0) = SO3::expm(vecR);
    ret.block<3, 1>(0, 3) = V * vecT;

    return ret;
}

} //namespace SE3

} //namespace aliceVision