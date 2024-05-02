// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/numeric/algebra.hpp>

namespace aliceVision {
namespace multiview {

// HZ 12.2 pag.312
void TriangulateDLT(const Mat34& P1, const Vec2& x1, const Mat34& P2, const Vec2& x2, Vec4& X_homogeneous)
{
    Mat4 design;
    for (int i = 0; i < 4; ++i)
    {
        design(0, i) = x1[0] * P1(2, i) - P1(0, i);
        design(1, i) = x1[1] * P1(2, i) - P1(1, i);
        design(2, i) = x2[0] * P2(2, i) - P2(0, i);
        design(3, i) = x2[1] * P2(2, i) - P2(1, i);
    }
    Nullspace(design, X_homogeneous);
}

void TriangulateDLT(const Mat34& P1, const Vec2& x1, const Mat34& P2, const Vec2& x2, Vec3& X_euclidean)
{
    Vec4 X_homogeneous;
    TriangulateDLT(P1, x1, P2, x2, X_homogeneous);
    homogeneousToEuclidean(X_homogeneous, X_euclidean);
}

// Solve:
// [cross(x0,P0) X = 0]
// [cross(x1,P1) X = 0]
void TriangulateSphericalDLT(const Mat4& T1, const Vec3& x1, const Mat4& T2, const Vec3& x2, Vec4& X_homogeneous)
{
    Mat design(6, 4);
    for (int i = 0; i < 4; ++i)
    {
        design(0, i) = -x1[2] * T1(1, i) + x1[1] * T1(2, i);
        design(1, i) = x1[2] * T1(0, i) - x1[0] * T1(2, i);
        design(2, i) = -x1[1] * T1(0, i) + x1[0] * T1(1, i);

        design(3, i) = -x2[2] * T2(1, i) + x2[1] * T2(2, i);
        design(4, i) = x2[2] * T2(0, i) - x2[0] * T2(2, i);
        design(5, i) = -x2[1] * T2(0, i) + x2[0] * T2(1, i);
    }
    Nullspace(design, X_homogeneous);
}

void TriangulateSphericalDLT(const Mat4 & T1, const Vec3& x1, const Mat4& T2, const Vec3& x2, Vec3& X_euclidean)
{
    Vec4 X_homogeneous;
    TriangulateSphericalDLT(T1, x1, T2, x2, X_homogeneous);
    homogeneousToEuclidean(X_homogeneous, X_euclidean);
}

}  // namespace multiview
}  // namespace aliceVision
