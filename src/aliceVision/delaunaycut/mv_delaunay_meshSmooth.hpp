// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/common/MultiViewParams.hpp>
#include <aliceVision/structures/mv_staticVector.hpp>
#include <aliceVision/mesh/mv_mesh.hpp>

class point3d;
class mv_delaunay_GC;
class mv_prematch_cams;

void meshPostProcessing(mv_mesh*& inout_mesh, staticVector<staticVector<int>*>*& inout_ptsCams, staticVector<int>& usedCams,
                      multiviewParams& mp, mv_prematch_cams& pc,
                      const std::string& resultFolderName,
                      staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d* hexah);
