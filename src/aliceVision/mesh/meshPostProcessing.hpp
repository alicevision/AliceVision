// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/MultiViewParams.hpp>
#include <aliceVision/mesh/Mesh.hpp>

namespace aliceVision {

class Point3d;

namespace common {
class PreMatchCams;
} // namespace common

namespace mesh {

void meshPostProcessing(Mesh*& inout_mesh, StaticVector<StaticVector<int>*>*& inout_ptsCams, StaticVector<int>& usedCams,
                      common::MultiViewParams& mp, common::PreMatchCams& pc,
                      const std::string& resultFolderName,
                      StaticVector<Point3d>* hexahsToExcludeFromResultingMesh, Point3d* hexah);

} // namespace mesh
} // namespace aliceVision
