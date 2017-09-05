// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_MULTIVIEW_TRIANGULATION_H_
#define OPENMVG_MULTIVIEW_TRIANGULATION_H_

#include "openMVG/numeric/numeric.h"

namespace openMVG {

/// Linear DLT triangulation: HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec4 *X_homogeneous);

/// Linear DLT triangulation: HZ 12.2 pag.312
void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
                    const Mat34 &P2, const Vec2 &x2,
                    Vec3 *X_euclidean);

} // namespace openMVG

#endif  // OPENMVG_MULTIVIEW_TRIANGULATION_H_
