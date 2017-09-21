// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).
#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {

/// Linear DLT triangulation: HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous);

/// Linear DLT triangulation: HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean);

} // namespace aliceVision
