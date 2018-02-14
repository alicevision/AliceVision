// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshAnalyze.hpp"
#include <aliceVision/structures/geometry.hpp>

MeshAnalyze::MeshAnalyze(MultiViewParams* _mp)
    : MeshClean(_mp)
{}

MeshAnalyze::~MeshAnalyze() = default;


Point2d MeshAnalyze::getCotAlphaijAndCotBetaij(int i, int j, StaticVector<int>* ptNeighPtsOrdered)
{
    int jp1 = j + 1;
    if(jp1 >= ptNeighPtsOrdered->size())
    {
        jp1 = 0;
    }
    int jm1 = j - 1;
    if(jm1 < 0)
    {
        jm1 = ptNeighPtsOrdered->size() - 1;
    }

    Point3d pti = (*pts)[i];
    Point3d ptjm1 = (*pts)[(*ptNeighPtsOrdered)[jm1]];
    Point3d ptj = (*pts)[(*ptNeighPtsOrdered)[j]];
    Point3d ptjp1 = (*pts)[(*ptNeighPtsOrdered)[jp1]];

    float cotalpha = 1.0f / tan(angleBetwABandAC(ptjm1, ptj, pti) * (M_PI / 180.0f));
    float cotbeta = 1.0f / tan(angleBetwABandAC(ptjp1, ptj, pti) * (M_PI / 180.0f));

    return Point2d(cotalpha, cotbeta);
}

float MeshAnalyze::AreaVor(int i, StaticVector<int>* ptNeighPtsOrdered)
{
    float A = 0.0f;
    for(int j = 0; j < ptNeighPtsOrdered->size(); j++)
    {
        Point2d cab = getCotAlphaijAndCotBetaij(i, j, ptNeighPtsOrdered);
        A += ((*pts)[i] - (*pts)[(*ptNeighPtsOrdered)[j]]).size2() * (cab.x + cab.y);
    }

    A *= 1.0f / 8.0f;

    return A;
}

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

double MeshAnalyze::getAngleFromCotan(Point3d& vo, Point3d& v1, Point3d& v2)
{
    /* cf. Appendix B and the caption of Table 1 from [Meyer et al 2002] */
    Point3d u = v1 - vo;
    Point3d v = v2 - vo;

    double udotv = dot(u, v);
    double denom = sqrt(dot(u, u) * dot(v, v) - udotv * udotv);

    // tan = denom/udotv = y/x
    return (fabs(atan2(denom, udotv)));
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
        Point3d A = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 0) % 3]];
        Point3d B = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 1) % 3]];
        Point3d C = (*pts)[(*tris)[triId].i[(vertexIdInTriangle + 2) % 3]];
        return (getCotanOfAngle(B, A, C) * (A - C).size2() + getCotanOfAngle(C, A, B) * (A - B).size2()) / 8.0;
    }
}

int MeshAnalyze::getVertexIdInTriangleForPtId(int ptId, int triId)
{
    for(int i = 0; i < 3; i++)
    {
        if((*tris)[triId].i[i] == ptId)
        {
            return i;
        }
    }
    return -1;
}

bool MeshAnalyze::getVertexSurfaceNormal(int ptId, Point3d& N)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr) ||
       (ptNeighTris->size() == 0))
    {
        return false;
    }

    N = Point3d();
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        N = N + computeTriangleNormal(triId);
    }
    N = N / (float)ptNeighTris->size();

    return true;
}

// gts_vertex_mean_curvature_normal [Meyer et al 2002]
bool MeshAnalyze::getVertexMeanCurvatureNormal(int ptId, Point3d& Kh)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
    {
        return false;
    }

    double area = 0.0;
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
        area += getRegionArea(vertexIdInTriangle, triId);
    }

    Kh = Point3d(0.0f, 0.0f, 0.0f);

    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        int ip1 = i + 1;
        if(ip1 >= ptNeighPtsOrdered->size())
        {
            ip1 = 0;
        }
        Point3d v = (*pts)[ptId];
        Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
        Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

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

// gts_vertex_gaussian_curvature [Meyer et al 2002]
bool MeshAnalyze::getVertexGaussianCurvature(int ptId, double& Kg)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
    if((isIsBoundaryPt(ptId)) || (ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
    {
        return false;
    }

    double area = 0.0;
    for(int i = 0; i < ptNeighTris->size(); i++)
    {
        int triId = (*ptNeighTris)[i];
        int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
        area += getRegionArea(vertexIdInTriangle, triId);
    }

    double angle_sum = 0.0;
    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        int ip1 = i + 1;
        if(ip1 >= ptNeighPtsOrdered->size())
        {
            ip1 = 0;
        }
        Point3d v = (*pts)[ptId];
        Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
        Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

        angle_sum += getAngleFromCotan(v, v1, v2);
    }

    Kg = (2.0 * M_PI - angle_sum) / area;

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

bool MeshAnalyze::applyLaplacianOperator(int ptId, StaticVector<Point3d>* ptsToApplyLaplacianOp, Point3d& ln)
{
    StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
    if(ptNeighPtsOrdered == nullptr)
    {
        return false;
    }

    ln = Point3d(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
    {
        Point3d npt = (*ptsToApplyLaplacianOp)[(*ptNeighPtsOrdered)[i]];

        if((npt.x == 0.0f) && (npt.y == 0.0f) && (npt.z == 0.0f))
        {
            // printf("zero neighb pt\n");
            return false;
        }
        ln = ln + npt;
    }
    ln = (ln / (float)ptNeighPtsOrdered->size()) - (*ptsToApplyLaplacianOp)[ptId];

    Point3d n = ln;
    float d = n.size();
    n = n.normalize();
    if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
       (n.y != n.y) || (n.z != n.z)) // check if is not NaN
    {
        // printf("nan\n");
        return false;
    }

    if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
       (n.y != n.y) || (n.z != n.z)) // check if is not NaN
    {
        // printf("nan\n");
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
// page 5 - U1 - laplacian is obtained wnen apply to origina pts , U2 - bi-laplacian is obtained when apply to laplacian
// pts
bool MeshAnalyze::getBiLaplacianSmoothingVector(int ptId, StaticVector<Point3d>* ptsLaplacian, Point3d& tp)
{
    if(applyLaplacianOperator(ptId, ptsLaplacian, tp))
    {
        StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
        if((ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
        {
            return false;
        }

        float sum = 0.0f;
        for(int i = 0; i < sizeOfStaticVector<int>(ptNeighPtsOrdered); i++)
        {
            int neighValence = sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[(*ptNeighPtsOrdered)[i]]);
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
        if(std::isnan(d) || std::isnan(n.x) || std::isnan(n.y) || std::isnan(n.z) || (d != d) || (n.x != n.x) ||
           (n.y != n.y) || (n.z != n.z)) // check if is not NaN
        {
            return false;
        }
        // page 6 eq (8)

        return true;
    }

    return false;
}

// kobbelt kampagna 98 Interactive Multi-Resolution Modeling on Arbitrary Meshes
// page 5 - U1 - laplacian is obtained wnen apply to origina pts , U2 - bi-laplacian is obtained when apply to laplacian
// pts
bool MeshAnalyze::getBiLaplacianSmoothingVectorAndPrincipalCurvatures(
    int ptId, StaticVector<Point3d>* ptsLaplacian, Point3d& smoothingVector, Point3d& smoothingVectorNormalized,
    Point3d& normalVectorNormalized, double& smoothingVectorSize, double& K1, double& K2, double& area,
    double& avNeighEdegeLenth)
{
    if(applyLaplacianOperator(ptId, ptsLaplacian, smoothingVector))
    {
        StaticVector<int>* ptNeighPtsOrdered = (*ptsNeighPtsOrdered)[ptId];
        StaticVector<int>* ptNeighTris = (*ptsNeighTrisSortedAsc)[ptId];
        if((ptNeighPtsOrdered == nullptr) || (ptNeighTris == nullptr))
        {
            return false;
        }

        double sum = 0.0;
        for(int i = 0; i < sizeOfStaticVector<int>(ptNeighPtsOrdered); i++)
        {
            int neighValence = sizeOfStaticVector<int>((*ptsNeighPtsOrdered)[(*ptNeighPtsOrdered)[i]]);
            if(neighValence > 0)
            {
                sum += 1.0 / (double)neighValence;
            }
        }
        double v = 1.0 + (1.0 / (double)sizeOfStaticVector<int>(ptNeighPtsOrdered)) * sum;

        smoothingVector = Point3d(0.0, 0.0, 0.0) - smoothingVector * (1.0 / v);

        smoothingVectorNormalized = smoothingVector;
        smoothingVectorSize = smoothingVectorNormalized.size();
        smoothingVectorNormalized = smoothingVectorNormalized.normalize();
        if(std::isnan(smoothingVector.x) || std::isnan(smoothingVector.y) || std::isnan(smoothingVector.z) ||
           std::isnan(smoothingVectorSize) || std::isnan(smoothingVectorNormalized.x) ||
           std::isnan(smoothingVectorNormalized.y) || std::isnan(smoothingVectorNormalized.z) ||
           (smoothingVectorSize != smoothingVectorSize) || (smoothingVector.x != smoothingVector.x) ||
           (smoothingVector.y != smoothingVector.y) || (smoothingVector.z != smoothingVector.z) ||
           (smoothingVectorNormalized.x != smoothingVectorNormalized.x) ||
           (smoothingVectorNormalized.y != smoothingVectorNormalized.y) ||
           (smoothingVectorNormalized.z != smoothingVectorNormalized.z)) // check if is not NaN
        {
            return false;
        }
        // page 6 eq (8)

        // compute K1, K2
        area = 0.0;
        normalVectorNormalized = Point3d(0.0f, 0.0f, 0.0f);
        for(int i = 0; i < ptNeighTris->size(); i++)
        {
            int triId = (*ptNeighTris)[i];
            int vertexIdInTriangle = getVertexIdInTriangleForPtId(ptId, triId);
            area += getRegionArea(vertexIdInTriangle, triId);
            normalVectorNormalized = normalVectorNormalized + computeTriangleNormal(triId);
        }
        normalVectorNormalized = normalVectorNormalized / (double)ptNeighTris->size();

        double angle_sum = 0.0;
        Point3d KhVect = Point3d(0.0f, 0.0f, 0.0f);
        avNeighEdegeLenth = 0.0f;
        for(int i = 0; i < ptNeighPtsOrdered->size(); i++)
        {
            int ip1 = i + 1;
            if(ip1 >= ptNeighPtsOrdered->size())
            {
                ip1 = 0;
            }
            Point3d v = (*pts)[ptId];
            Point3d v1 = (*pts)[(*ptNeighPtsOrdered)[i]];
            Point3d v2 = (*pts)[(*ptNeighPtsOrdered)[ip1]];

            double temp = getCotanOfAngle(v1, v, v2);
            KhVect = KhVect + (v2 - v) * temp;

            temp = getCotanOfAngle(v2, v, v1);
            KhVect = KhVect + (v1 - v) * temp;

            angle_sum += getAngleFromCotan(v, v1, v2);

            avNeighEdegeLenth += (v - v1).size();
        }
        avNeighEdegeLenth /= (double)ptNeighPtsOrdered->size();

        if(area > 0.0)
        {
            KhVect = KhVect / (2.0f * area);
        }
        else
        {
            return false;
        }

        double Kh = KhVect.size();
        double Kg = (2.0 * M_PI - angle_sum) / area;

        getVertexPrincipalCurvatures(Kh, Kg, K1, K2);

        if(std::isnan(K1) || std::isnan(K2) || (K1 != K1) || (K2 != K2)) // check if is not NaN
        {
            return false;
        }

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
