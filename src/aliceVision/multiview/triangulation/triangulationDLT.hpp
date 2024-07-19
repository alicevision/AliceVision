// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace multiview {

/**
 * Triangulate a point given a set of observations
 * and associated projection matrices (in pixels)
 * @param P1 a projection matrix K (R | t)
 * @param x1 a 2d observation vector
 * @param P2 a projection matrix K (R | t)
 * @param x2 a 2d observation vector
 * @param X_homogeneous a homogeneous 3d point
 */
void TriangulateDLT(const Mat34& P1, const Vec2& x1, const Mat34& P2, const Vec2& x2, Vec4& X_homogeneous);

/**
 * Triangulate a point given a set of observations
 * and associated projection matrices (in pixels)
 * @param P1 a projection matrix K (R | t)
 * @param x1 a 2d observation vector
 * @param P2 a projection matrix K (R | t)
 * @param x2 a 2d observation vector
 * @param X_homogeneous a 3d point
 */
void TriangulateDLT(const Mat34& P1, const Vec2& x1, const Mat34& P2, const Vec2& x2, Vec3& X_euclidean);

/**
 * Triangulate a point given a set of bearing vectors
 * and associated projection matrices (in meters)
 * @param T1 a transformation matrix (R | t)
 * @param x1 a unit bearing vector
 * @param T2 a transformation matrix (R | t)
 * @param x2 a unit bearing vector
 * @param X_homogeneous a homogeneous 3d point
 */
void TriangulateSphericalDLT(const Mat4& T1, const Vec3& x1, const Mat4& T2, const Vec3& x2, Vec4& X_homogeneous);

/**
 * Triangulate a point given a set of bearing vectors
 * and associated projection matrices (in meters)
 * @param T1 a projection matrix (R | t)
 * @param x1 a unit bearing vector
 * @param T2 a projection matrix K (R | t)
 * @param x2 a unit bearing vector
 * @param X_homogeneous a 3d point
 */
void TriangulateSphericalDLT(const Mat4& T1, const Vec3& x1, const Mat4& T2, const Vec3& x2, Vec3& X_euclidean);

}  // namespace multiview
}  // namespace aliceVision
