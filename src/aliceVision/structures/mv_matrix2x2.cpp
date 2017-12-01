// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_matrix2x2.hpp"
#include <cstdio>

void matrix2x2::doprintf()
{
    printf("%f %f\n", m11, m12);
    printf("%f %f\n", m21, m22);
}
