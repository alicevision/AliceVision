// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshAnalyze.hpp"
#include <aliceVision/mvsData/geometry.hpp>

namespace aliceVision {
namespace mesh {

MeshAnalyze::MeshAnalyze(mvsUtils::MultiViewParams* _mp)
    : MeshClean(_mp)
{}

MeshAnalyze::~MeshAnalyze() = default;

double MeshAnalyze::getCotanOfAngle(Point3d& vo, Point3d& v1, Point3d& v2)
{
    /* cf. Appendix B of [Meyer et al 2002] */

    Point3d u = v1 - vo;
    Point3d v = v2 - vo;

    double udotv = dot(u, v);
    double denom = sqrt(dot(u, u) * dot(v, v) - udotv * udotv);

    /* denom can be zero if u==v.  Returning 0 is acceptable, based on
     * the callers of this function below. */
    if(denom == 0.0)
        return (0.0);

    return (udotv / denom);
}

double MeshAnalyze::getRegionArea(int vertexIdInTriangle, int triId)
{
    /* cf. Section 3.3 of [Meyer et al 2002] */
    double triArea = computeTriangleArea(triId);
    if(triArea == 0.0)
        return (0.0);

    if(isTriangleObtuse(triId))
    {
        if(isTriangleAngleAtVetexObtuse(vertexIdInTriangle, triId))
            return (triArea / 2.0);

        return (triArea / 4.0);
    }
    else
    {
        Point3d A = pts[tris[triId].v[(vertexIdInTriangle + 0) % 3]];
        Point3d B = pts[tris[triId].v[(vertexIdInTriangle + 1) % 3]];
        Point3d C = pts[tris[triId].v[(vertexIdInTriangle + 2) % 3]];
        return (getCotanOfAngle(B, A, C) * (A - C).size2() + getCotanOfAngle(C, A, B) * (A - B).size2()) / 8.0;
    }
}

int MeshAnalyze::getVertexIdInTriangleForPtId(int ptId, int triId)
{
    for(int i = 0; i < 3; i++)
    {
        if(tris[triId].v[i] == ptId)
        {
            return i;
        }
    }
    return -1;
}

bool MeshAnalyze::getVertexSurfaceNormal(int ptId, Point3d& N)
{
    StaticVector<int>& ptNeighPtsOrdered = ptsNeighPtsOrdered[ptId];
    StaticVector<int>& ptNeighTris = ptsNeighTrisSortedAsc[ptId];
    if((isIsBoundaryPt(ptId)) || ptNeighPtsOrdered.empty() || ptNeighTris.empty() )
    {
        return false;
    }

    N = Point3d();
    for(int i = 0; i < ptNeighTris.size(); i++)
    {
        int triId = ptNeighTris[i];
        N = N + computeTriangleNormal(triId);
    }
    N = N / (float)ptNeighTris.size();

    return true;
}

// gts_vertex_mean_curvature_normal [Meyer et al 2002]
bool MeshAnalyze::getVertexMeanCurvatureNormal(int ptId, Point3d& Kh)
{
    StaticVector<int>& ptNeighPtsOrdered = ptsNeighPtsOrdered[ptId];
    StaticVector<int>& ptNeighTris = ptsNeighTrisSortedAsc[ptId];
    if((isIsBoundaryPt(ptId)) || ptNeighPtsOrdered.empty() || ptNeighTris.empty())
    {
        return false;
    }

    double area = 0.0;
    for(int i = 0; i < ptNeighTris.size(); i++)
    {
        int triId = ptNeighTris[i];
        int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
        area += getRegionArea(vertexIdInTriangle, triId);
    }

    Kh = Point3d(0.0f, 0.0f, 0.0f);

    for(int i = 0; i < ptNeighPtsOrdered.size(); i++)
    {
        int ip1 = i + 1;
        if(ip1 >= ptNeighPtsOrdered.size())
        {
            ip1 = 0;
        }
        Point3d v = pts[ptId];
        Point3d v1 = pts[ptNeighPtsOrdered[i]];
        Point3d v2 = pts[ptNeighPtsOrdered[ip1]];

        float temp = getCotanOfAngle(v1, v, v2);
        Kh = Kh + (v2 - v) * temp;

        temp = getCotanOfAngle(v2, v, v1);
        Kh = Kh + (v1 - v) * temp;
    }

    if(area > 0.0)
    {
        Kh = Kh / (2.0f * area);
    }
    else
    {
        return false;
    }

    return true;
}

// gts_vertex_principal_curvatures [Meyer et al 2002]
void MeshAnalyze::getVertexPrincipalCurvatures(double Kh, double Kg, double& K1, double& K2)
{
    double temp = Kh * Kh - Kg;
    if(temp < 0.0)
        temp = 0.0;
    temp = sqrt(temp);
    K1 = Kh + temp;
    K2 = Kh - temp;
}

bool MeshAnalyze::applyLaplacianOperator(int ptId, const StaticVector<Point3d>& ptsToApplyLaplacianOp, Point3d& ln)
{
    StaticVector<int>& ptNeighPtsOrdered = ptsNeighPtsOrdered[ptId];
    if(ptNeighPtsOrdered.empty())
    {
        return false;
    }

    ln = Point3d(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < ptNeighPtsOrdered.size(); i++)
    {
        Point3d npt = ptsToApplyLaplacianOp[ptNeighPtsOrdered[i]];

        if((npt.x == 0.0f) && (npt.y == 0.0f) && (npt.z == 0.0f))
        {
            ALICEVISION_LOG_WARNING("MeshAnalyze::applyLaplacianOperator: zero neighb pt");
            return false;
        }
        ln = ln + npt;
    }
    ln = (ln / (float)ptNeighPtsOrdered.size()) - ptsToApplyLaplacianOp[ptId];

    Point3d n = ln;
    float d = n.size();
    n = n.normalize();
    if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) // check if is not NaN
    {
        ALICEVISION_LOG_WARNING("MeshAnalyze::applyLaplacianOperator: nan");
        return false;
    }

    return true;
}

// othake et al 00 Polyhedral Surface Smoothing with Simultaneous Mesh Regularization
// page 3 eq (3)
bool MeshAnalyze::getLaplacianSmoothingVector(int ptId, Point3d& ln)
{
    return applyLaplacianOperator(ptId, pts, ln);
}

// kobbelt kampagna 98 Interactive Multi-Resolution Modeling on Arbitrary Meshes
// page 5:
// U1 - laplacian is obtained when apply to original points,
// U2 - bi-laplacian is obtained when apply to laplacian points
bool MeshAnalyze::getBiLaplacianSmoothingVector(int ptId, const StaticVector<Point3d>& ptsLaplacian, Point3d& tp)
{
    if(applyLaplacianOperator(ptId, ptsLaplacian, tp))
    {
        StaticVector<int>& ptNeighPtsOrdered = ptsNeighPtsOrdered[ptId];
        StaticVector<int>& ptNeighTris = ptsNeighTrisSortedAsc[ptId];
        if(ptNeighPtsOrdered.empty() || ptNeighTris.empty() )
        {
            return false;
        }

        float sum = 0.0f;
        for(int i = 0; i < sizeOfStaticVector<int>(ptNeighPtsOrdered); i++)
        {
            int neighValence = sizeOfStaticVector<int>(ptsNeighPtsOrdered[ptNeighPtsOrdered[i]]);
            if(neighValence > 0)
            {
                sum += 1.0f / (float)neighValence;
            }
        }
        float v = 1.0f + (1.0f / (float)sizeOfStaticVector<int>(ptNeighPtsOrdered)) * sum;

        tp = Point3d(0.0f, 0.0f, 0.0f) - tp * (1.0f / v);

        Point3d n = tp;
        float d = n.size();
        n = n.normalize();
        if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z)) // check if is not NaN
        {
            return false;
        }
        // page 6 eq (8)

        return true;
    }

    return false;
}

// othake et al 00 Polyhedral Surface Smoothing with Simultaneous Mesh Regularization
// page 6 eq (13)
bool MeshAnalyze::getMeanCurvAndLaplacianSmoothing(int ptId, Point3d& F, float epsilon)
{
    Point3d Hn;
    if(!getVertexMeanCurvatureNormal(ptId, Hn))
    {
        return false;
    }

    Point3d U0;

    if(!getLaplacianSmoothingVector(ptId, U0))
    {
        return false;
    }

    Point3d m = U0.normalize();
    float absH = Hn.size();

    float cosTheta = dot(m, Hn) / absH;

    if(cosTheta > epsilon)
    {
        F = (m * absH) / cosTheta;
        return true;
    }

    if(cosTheta < -epsilon)
    {
        F = Hn * 2.0f - (m * absH) / cosTheta;
        return true;
    }

    if(fabs(cosTheta) <= epsilon)
    {
        F = Point3d(0.0f, 0.0f, 0.0f);
        return true;
    }

    return false;
}

} // namespace mesh
} // namespace aliceVision
