// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_matrix.hpp"

void matrix4x4::doprintf()
{
    printf("%f %f %f %f\n", m11, m12, m13, m14);
    printf("%f %f %f %f\n", m21, m22, m23, m24);
    printf("%f %f %f %f\n", m31, m32, m33, m34);
    printf("%f %f %f %f\n", m41, m42, m43, m44);
};
