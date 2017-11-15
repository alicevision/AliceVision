// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_mesh_clean.hpp"
#include <aliceVision/output3D/mv_output3D.hpp>

class mv_mesh_analyze : public mv_mesh_clean
{
public:
    mv_mesh_analyze(multiviewParams* _mp);
    ~mv_mesh_analyze();

    point3d getCotAlphaCotBetaCotGammaForTriangle(int i);
    point2d getCotAlphaijAndCotBetaij(int i, int j, staticVector<int>* ptNeighPtsOrdered);
    float AreaVor(int i, staticVector<int>* ptNeighPtsOrdered);
    point3d meanCurvVorAtPti(int i, staticVector<int>* ptNeighPtsOrdered);

    double getCotanOfAngle(point3d& vo, point3d& v1, point3d& v2);
    double getAngleFromCotan(point3d& vo, point3d& v1, point3d& v2);
    double getRegionArea(int vertexIdInTriangle, int triId);
    int getVertexIdInTriangleForPtId(int ptId, int triId);

    bool getVertexMeanCurvatureNormal(int ptId, point3d& Kh);
    bool getVertexGaussianCurvature(int ptId, double& Kg);
    void getVertexPrincipalCurvatures(double Kh, double Kg, double& K1, double& K2);

    bool applyLaplacianOperator(int ptId, staticVector<point3d>* ptsToApplyLaplacianOp, point3d& ln);
    bool getLaplacianSmoothingVector(int ptId, point3d& ln);
    bool getBiLaplacianSmoothingVector(int ptId, staticVector<point3d>* ptsLaplacian, point3d& tp);
    bool getBiLaplacianSmoothingVectorAndPrincipalCurvatures(int ptId, staticVector<point3d>* ptsLaplacian,
                                                             point3d& smoothingVector,
                                                             point3d& smoothingVectorNormalized,
                                                             point3d& normalVectorNormalized,
                                                             double &smoothingVectorSize, double& K1, double& K2,
                                                             double& area, double& avNeighEdegeLenth);

    bool getMeanCurvAndLaplacianSmoothing(int ptId, point3d& F, float epsilon);

    bool getVertexSurfaceNormal(int ptId, point3d& N);
};
