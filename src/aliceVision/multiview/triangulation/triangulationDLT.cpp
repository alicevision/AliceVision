// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

namespace aliceVision {

// HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous) {
  Mat4 design;
  for (int i = 0; i < 4; ++i) {
    design(0,i) = x1[0] * P1(2,i) - P1(0,i);
    design(1,i) = x1[1] * P1(2,i) - P1(1,i);
    design(2,i) = x2[0] * P2(2,i) - P2(0,i);
    design(3,i) = x2[1] * P2(2,i) - P2(1,i);
  }
  Nullspace(&design, X_homogeneous);
}

void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean) {
  Vec4 X_homogeneous;
  TriangulateDLT(P1, x1, P2, x2, &X_homogeneous);
  HomogeneousToEuclidean(X_homogeneous, X_euclidean);
}

}  // namespace aliceVision
