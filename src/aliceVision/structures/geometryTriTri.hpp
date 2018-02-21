// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {

int tri_tri_intersect(double V0[3], double V1[3], double V2[3], double U0[3], double U1[3], double U2[3]);

int tri_tri_intersect_with_isectline(double V0[3], double V1[3], double V2[3], double U0[3], double U1[3], double U2[3],
                                     int* coplanar, double isectpt1[3], double isectpt2[3]);

} // namespace aliceVision
