// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/structures/mv_geometry.hpp>
#include <aliceVision/common/PreMatchCams.hpp>

//float confKernelVoting(staticVector<float>* confs, float c);
void filterLargeEdgeTriangles(Mesh* me, float avelthr);
