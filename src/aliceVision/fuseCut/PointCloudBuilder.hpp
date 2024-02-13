// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/fuseCut/Octree.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
}

namespace fuseCut {


class PointCloudBuilder
{
public:
    PointCloudBuilder(mvsUtils::MultiViewParams& mp);

    void createDensePointCloud(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData & sfmData);

    void getNonEmptyNodes(std::vector<Node::ptr> & nodes)
    {
        nodes.clear();        
        
        if (_octree)
        {
            _octree->visit(nodes);
        }
    }

private:
    void addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData);
    void addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist);
    void addPointsToPreventSingularities(const Point3d Voxel[8], float minDist);
    void densifyWithHelperPoints(int nbFront, int nbBack, double scale);
    void addGridHelperPoints(int helperPointsGridSize, const Point3d Voxel[8], float minDist);

public:
    mvsUtils::MultiViewParams& _mp;
    std::unique_ptr<Node> _octree;

    std::vector<Point3d> _verticesCoords;
    std::vector<GC_vertexInfo> _verticesAttr;
    std::vector<int> _camsVertexes;
};

}
}