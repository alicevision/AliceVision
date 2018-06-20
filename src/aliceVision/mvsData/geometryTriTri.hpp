// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {

int tri_tri_intersect(const double V0[3], const double V1[3], const double V2[3], const double U0[3], const double U1[3], const double U2[3]);

int tri_tri_intersect_with_isectline(const double V0[3], const double V1[3], const double V2[3], const double U0[3], const double U1[3], const double U2[3],
                                     int* coplanar, double isectpt1[3], double isectpt2[3]);

} // namespace aliceVision
