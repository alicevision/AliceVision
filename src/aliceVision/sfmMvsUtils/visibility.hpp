// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


namespace aliceVision {

namespace mesh {
class Mesh;
}
namespace sfmData {
class SfMData;
}

namespace mvsUtils {

class MultiViewParams;

void createRefMeshFromDenseSfMData(mesh::Mesh& outRefMesh, const sfmData::SfMData& sfmData, const mvsUtils::MultiViewParams& mp);

} // namespace mvsUtils
} // namespace aliceVision
