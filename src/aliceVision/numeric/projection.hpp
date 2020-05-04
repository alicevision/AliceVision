// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

/**
 * Collection of function related to the classic Projection matrix used
 * in computer vision. P = K[R|t] with [t]=[-RC] Cf HZ
 */

namespace aliceVision {

/**
 * @brief Compute P = K[R|t]
 */
void P_from_KRt(const Mat3& K, const Mat3& R, const Vec3& t, Mat34* P);

/**
 * @brief Compute P = K[R|t]
 * @return P
 */
Mat34 P_from_KRt(const Mat3& K, const Mat3& R, const Vec3& t);

/**
 * @brief Decompose using the RQ decomposition HZ A4.1.1 pag.579.
 */
void KRt_from_P(const Mat34& P, Mat3* Kp, Mat3* Rp, Vec3* tp);

/**
 * @brief Compute a fundamental matrix from projection matrices
 */
Mat3 F_from_P(const Mat34& P1, const Mat34& P2);

/**
 * @brief Compute the depth of the X point. R*X[2]+t[2].
 */
double Depth(const Mat3& R, const Vec3& t, const Vec3& X);

/**
 * @brief Test whether the given points are in front of the camera.
 * @param[in] R the camera rotation.
 * @param[in] t the camera translation.
 * @param[in] X the 3D points to test.
 * @return A vector of boolean of the same size as the number ot points. The corresponding value is true if the point
 * is in front of the camera, false otherwise.
 */
Vecb cheiralityTest(const Mat3 &R, const Vec3 &t, const Mat3X &X);

/**
 * @brief Test whether all the given points are in front of the camera.
 * @param[in] R the camera rotation.
 * @param[in] t the camera translation.
 * @param[in] X the 3D points to test.
 * @return true if all the points is in front of the camera, false otherwise.
 */
bool cheiralityTestAll(const Mat3 &R, const Vec3 &t, const Mat3X &X);

/**
 * @brief Compute P*[X|1.0]. Transformed from homogeneous to euclidean coordinates.
 */
Vec2 project(const Mat34& P, const Vec3& X);

/**
 * @brief Compute P*[X|1.0] for the X list of point (3D point).
 */
void project(const Mat34& P, const Mat3X& X, Mat2X* x);

/**
 * @brief Compute P*[X|1.0] for the X list of point (4D point).
 */
void project(const Mat34& P, const Mat4X& X, Mat2X* x);

/**
 * @brief Return P*[X|1.0] for the X list of point (3D point).
 */
Mat2X project(const Mat34& P, const Mat3X& X);

/**
 * @brief Return P*[X|1.0] for the X list of point (4D point).
 */
Mat2X project(const Mat34& P, const Mat4X& X);

/**
 * @brief Change homogeneous coordinates to euclidean.
 */
void homogeneousToEuclidean(const Vec4& H, Vec3* X);

/**
 * @brief Change euclidean coordinates to homogeneous.
 */
void euclideanToHomogeneous(const Mat& X, Mat* H);

/**
 * @brief Change euclidean coordinates to homogeneous.
 */
Vec3 euclideanToHomogeneous(const Vec2& x);

/**
 * @brief Change homogeneous coordinates to euclidean.
 */
void homogeneousToEuclidean(const Mat& H, Mat* X);

/**
 * @brief Change euclidean coordinates to homogeneous.
 */
Mat3X euclideanToHomogeneous(const Mat2X& x);

/**
 * @brief Change euclidean coordinates to homogeneous.
 */
void euclideanToHomogeneous(const Mat2X& x, Mat3X* h);

/**
 * @brief Change homogeneous coordinates to euclidean.
 */
void homogeneousToEuclidean(const Mat3X& h, Mat2X* e);

/**
 * @brief Project x point in camera coordinates
 */
void euclideanToNormalizedCamera(const Mat2X& x, const Mat3& K, Mat2X* n);

/**
 * @brief Project x point in camera coordinates
 */
void homogeneousToNormalizedCamera(const Mat3X& x, const Mat3& K, Mat2X* n);

/**
 * @brief Estimates the root mean square error (2D)
 */
double reprojectionErrorRMSE(const Mat2X& x_image, const Mat4X& X_world, const Mat34& P);

/**
 * @brief Estimates the root mean square error (2D)
 */
double reprojectionErrorRMSE(const Mat2X& x_image, const Mat3X& X_world, const Mat3& K, const Mat3& R, const Vec3& t);

} // namespace aliceVision
