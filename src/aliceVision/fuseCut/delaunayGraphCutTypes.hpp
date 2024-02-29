// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>

#include <array>

namespace aliceVision {
namespace fuseCut {

struct GC_cellInfo
{
    float cellSWeight = 0.0f;
    float cellTWeight = 0.0f;
    std::array<float, 4> gEdgeVisWeight{{0.0f, 0.0f, 0.0f, 0.0f}};
    float emptinessScore = 0.0f;
    float on = 0.0f;
};

struct GC_vertexInfo
{
    float pixSize = 0.0f;
    
    /// Number of cameras which have contributed to the refinement of the vertex position, so nrc >= cams.size().
    int nrc = 0;

    /// All cameras having a visibility of this vertex. Some of them may not have contributed to the vertex position
    StaticVector<int> cams;

    /**
     * @brief Is the vertex a virtual point without associated camera? Like helper points or camera points.
     */
    inline bool isVirtual() const { return cams.empty(); }
    inline bool isReal() const { return !cams.empty(); }
    inline std::size_t getNbCameras() const { return cams.size(); }
    inline int getCamera(std::size_t index) const { return cams[index]; }
};



}  // namespace fuseCut
}  // namespace aliceVision
