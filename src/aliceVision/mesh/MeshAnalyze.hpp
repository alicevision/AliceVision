// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mesh/MeshClean.hpp>

namespace aliceVision {
namespace mesh {

class MeshAnalyze : public MeshClean
{
public:
    MeshAnalyze(mvsUtils::MultiViewParams* _mp);
    ~MeshAnalyze();

    double getCotanOfAngle(Point3d& vo, Point3d& v1, Point3d& v2);
    double getRegionArea(int vertexIdInTriangle, int triId);
    int getVertexIdInTriangleForPtId(int ptId, int triId);
    bool getVertexMeanCurvatureNormal(int ptId, Point3d& Kh);
    void getVertexPrincipalCurvatures(double Kh, double Kg, double& K1, double& K2);
    bool applyLaplacianOperator(int ptId, const StaticVector<Point3d>& ptsToApplyLaplacianOp, Point3d& ln);
    bool getLaplacianSmoothingVector(int ptId, Point3d& ln);
    bool getBiLaplacianSmoothingVector(int ptId, const StaticVector<Point3d>& ptsLaplacian, Point3d& tp);
    bool getMeanCurvAndLaplacianSmoothing(int ptId, Point3d& F, float epsilon);
    bool getVertexSurfaceNormal(int ptId, Point3d& N);
};

} // namespace mesh
} // namespace aliceVision
