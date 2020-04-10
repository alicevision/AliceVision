// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mesh/Mesh.hpp>

#include <vector>

namespace aliceVision {
namespace mesh {

class UVAtlas
{
public:
    struct Edge
    {
        std::pair<int, int> pointIDs;
        std::vector<int> triangleIDs;
        bool operator<(const Edge& other) const { return pointIDs < other.pointIDs; }
        bool operator==(const Edge& other) const { return pointIDs == other.pointIDs; }
    };

    struct Chart
    {
        int refCameraID = -1;                                   // refCamera, used to project all contained triangles
        std::vector<int> commonCameraIDs;                       // list of common cameras
        std::vector<int> triangleIDs;                           // list of all contained triangles
        Pixel sourceLU;                                         // left-up pixel coordinates (in refCamera space)
        Pixel sourceRD;                                         // right-down pixel coordinates (in refCamera space)
        Pixel targetLU;                                         // left-up pixel coordinates (in uvatlas texture)
        float downscale = 1.0f;                                 // downscale factor applied to this chart
        int mergedWith = -1;                                    // ID of target chart, or -1 (not merged)

        /// Chart width in refCamera space
        int sourceWidth() const { return (sourceRD.x - sourceLU.x); }
        /// Chart height in refCamera space
        int sourceHeight() const { return (sourceRD.y - sourceLU.y); }
        /// Chart target width (uv space, taking downscale into account)
        int targetWidth() const { return sourceWidth() * downscale; }
        /// Chart target height (uv space, taking downscale into account)
        int targetHeight() const { return sourceHeight() * downscale; }
    };

    struct ChartRect
    {
        Chart* c = nullptr;
        ChartRect* child[2] {nullptr, nullptr};
        Pixel LU;
        Pixel RD;
        void clear();
        ChartRect* insert(Chart& chart, size_t gutter);
    };

public:
    UVAtlas(const Mesh& mesh, mvsUtils::MultiViewParams& mp,
                    unsigned int textureSide, unsigned int gutterSize);

public:
    const std::vector<std::vector<Chart>>& atlases() const { return _atlases; }
    const std::vector<int>& visibleCameras(int triangleID) const { return _triangleCameraIDs[triangleID]; }
    int textureSide() const { return _textureSide; }
    const Mesh& mesh() const { return _mesh; }
    inline int chartMaxSize() const { return (_textureSide - 1) - _gutterSize * 2; }

private:
    void createCharts(std::vector<Chart>& charts, mvsUtils::MultiViewParams& mp);
    void packCharts(std::vector<Chart>& charts, mvsUtils::MultiViewParams& mp);
    void finalizeCharts(std::vector<Chart>& charts, mvsUtils::MultiViewParams& mp);
    void createTextureAtlases(std::vector<Chart>& charts, mvsUtils::MultiViewParams& mp);

private:
    std::vector<std::vector<Chart>> _atlases;
    std::vector<std::vector<int>> _triangleCameraIDs;
    int _textureSide;
    int _gutterSize;
    const Mesh& _mesh;
};

} // namespace mesh
} // namespace aliceVision
