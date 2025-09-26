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

/**
* let v = R.transpose() * x; (to get a world coordinates direction)
* then lambda * v = X - center
* (v) cross (X - center) = 0
* [v]x*X - [V]x*center) = 0
*/
void TriangulateSphericalDLT(const Mat4& T1, const Vec3& x1, const Mat4& T2, const Vec3& x2, Vec4& X_homogeneous)
{
    const Mat3 R1 = T1.block<3, 3>(0, 0);
    const Mat3 R2 = T2.block<3, 3>(0, 0);
    const Vec3 t1 = T1.block<3, 1>(0, 3);
    const Vec3 t2 = T2.block<3, 1>(0, 3);
    const Vec3 c1 = -R1.transpose() * t1;
    const Vec3 c2 = -R2.transpose() * t2;

    const Vec3 v1 = R1.transpose() * x1;
    const Vec3 v2 = R2.transpose() * x2;

    Mat4 design;
    
    //We keep last 2 rows of the equation for each point
    // [ 0  -z   y]
    // [ z   0  -x]
    // [-y   x   0]
    // --> 
    // [ z   0  -x]
    // [-y   x   0]

    // [ z   0  -x] * [cx cy cz]^t = z * cx - x * cz
    // [-y   x   0] * [cx cy cz]^t = -y * cx + x * cy

    //[v]x
    design(0, 0) = v1.z();
    design(0, 1) = 0;
    design(0, 2) = -v1.x();    
    design(1, 0) = - v1.y();
    design(1, 1) = v1.x();
    design(1, 2) = 0;

    //-[V]x*center
    design(0, 3) = - (v1.z() * c1.x() - v1.x() * c1.z());
    design(1, 3) = - (-v1.y() * c1.x() + v1.x() * c1.y());

    //[v]x
    design(2, 0) = v2.z();
    design(2, 1) = 0;
    design(2, 2) = -v2.x();    
    design(3, 0) = - v2.y();
    design(3, 1) = v2.x();
    design(3, 2) = 0;

    //-[V]x*center
    design(2, 3) = - (v2.z() * c2.x() - v2.x() * c2.z());
    design(3, 3) = - (-v2.y() * c2.x() + v2.x() * c2.y());

    // Solve AX=0, where X is the homogeneous coordinates vector of the 3d point
    // in the reference frame
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
