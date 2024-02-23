// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace fuseCut {

struct PointCloudFuseParams
{
    /// Max input points loaded from images
    int maxInputPoints = 50000000;
    /// Max points at the end of the depth maps fusion
    int maxPoints = 5000000;
    /// The step used to load depth values from depth maps is computed from maxInputPts. Here we define the minimal value for this step,
    /// so on small datasets we will not spend too much time at the beginning loading all depth values.
    int minStep = 2;
    /// After fusion, filter points based on their number of observations
    int minVis = 2;

    float simFactor = 15.0f;
    float angleFactor = 15.0f;
    double pixSizeMarginInitCoef = 2.0;
    double pixSizeMarginFinalCoef = 1.0;
    float voteMarginFactor = 4.0f;
    float contributeMarginFactor = 2.0f;
    float simGaussianSizeInit = 10.0f;
    float simGaussianSize = 10.0f;
    double minAngleThreshold = 0.1;
    bool refineFuse = true;
    // Weight for helper points from mask. Do not create helper points if zero.
    float maskHelperPointsWeight = 0.0;
    int maskBorderSize = 1;
};

class PointCloud
{
public:
    PointCloud(mvsUtils::MultiViewParams& mp);

    void createDensePointCloud(const Point3d hexah[8],
                                const StaticVector<int>& cams,
                                const sfmData::SfMData* sfmData,
                                const PointCloudFuseParams* depthMapsFuseParams);

    const std::vector<Point3d> & getVertices() const
    {
        return _verticesCoords;
    }

    const std::vector<GC_vertexInfo> & getVerticesAttrs() const
    {
        return _verticesAttr;
    }

    const std::vector<int> & getCameraIndices() const
    {
        return _camsVertexes;
    }

    void createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams);
    

public:
    void addPointsFromSfM(const Point3d hexah[8], const StaticVector<int>& cams, const sfmData::SfMData& sfmData);
    void addPointsFromCameraCenters(const StaticVector<int>& cams, float minDist);

private:
    void fuseFromDepthMaps(const StaticVector<int>& cams, const Point3d voxel[8], const PointCloudFuseParams& params);
    void addPointsToPreventSingularities(const Point3d Voxel[8], float minDist);
    void densifyWithHelperPoints(int nbFront, int nbBack, double scale);
    void addGridHelperPoints(int helperPointsGridSize, const Point3d Voxel[8], float minDist);
    void addMaskHelperPoints(const Point3d voxel[8], const StaticVector<int>& cams, const PointCloudFuseParams& params);

private:
    std::vector<Point3d> _verticesCoords;
    std::vector<GC_vertexInfo> _verticesAttr;
    std::vector<int> _camsVertexes;

    mvsUtils::MultiViewParams& _mp;
};

}
}