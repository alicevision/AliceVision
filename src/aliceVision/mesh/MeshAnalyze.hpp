// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/mesh/MeshClean.hpp>

class MeshAnalyze : public MeshClean
{
public:
    MeshAnalyze(MultiViewParams* _mp);
    ~MeshAnalyze();

    Point3d getCotAlphaCotBetaCotGammaForTriangle(int i);
    Point2d getCotAlphaijAndCotBetaij(int i, int j, StaticVector<int>* ptNeighPtsOrdered);
    float AreaVor(int i, StaticVector<int>* ptNeighPtsOrdered);
    Point3d meanCurvVorAtPti(int i, StaticVector<int>* ptNeighPtsOrdered);

    double getCotanOfAngle(Point3d& vo, Point3d& v1, Point3d& v2);
    double getAngleFromCotan(Point3d& vo, Point3d& v1, Point3d& v2);
    double getRegionArea(int vertexIdInTriangle, int triId);
    int getVertexIdInTriangleForPtId(int ptId, int triId);

    bool getVertexMeanCurvatureNormal(int ptId, Point3d& Kh);
    bool getVertexGaussianCurvature(int ptId, double& Kg);
    void getVertexPrincipalCurvatures(double Kh, double Kg, double& K1, double& K2);

    bool applyLaplacianOperator(int ptId, StaticVector<Point3d>* ptsToApplyLaplacianOp, Point3d& ln);
    bool getLaplacianSmoothingVector(int ptId, Point3d& ln);
    bool getBiLaplacianSmoothingVector(int ptId, StaticVector<Point3d>* ptsLaplacian, Point3d& tp);
    bool getBiLaplacianSmoothingVectorAndPrincipalCurvatures(int ptId, StaticVector<Point3d>* ptsLaplacian,
                                                             Point3d& smoothingVector,
                                                             Point3d& smoothingVectorNormalized,
                                                             Point3d& normalVectorNormalized,
                                                             double &smoothingVectorSize, double& K1, double& K2,
                                                             double& area, double& avNeighEdegeLenth);

    bool getMeanCurvAndLaplacianSmoothing(int ptId, Point3d& F, float epsilon);

    bool getVertexSurfaceNormal(int ptId, Point3d& N);
};
