#pragma once

#define _USE_MATH_DEFINES
#include "mv_structures.hpp"

#include <cmath>


#define GETICAMVALUE(iCamArr, iCam, x, y) iCamArr[iCam * 3 * 3 + x * 3 + y]

#define ORIENT_2D(a, b, c) ((a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x))

#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2)                                                               \
    {                                                                                                                  \
        if(ORIENT_2D(R2, P2, Q1) > 0.0f)                                                                               \
            if(ORIENT_2D(R2, Q2, Q1) < 0.0f)                                                                           \
                if(ORIENT_2D(P1, P2, Q1) > 0.0f)                                                                       \
                {                                                                                                      \
                    if(ORIENT_2D(P1, Q2, Q1) < 0.0f)                                                                   \
                        return true;                                                                                   \
                    else                                                                                               \
                        return false;                                                                                  \
                }                                                                                                      \
                else                                                                                                   \
                {                                                                                                      \
                    if(ORIENT_2D(P1, P2, R1) > 0.0f)                                                                   \
                        if(ORIENT_2D(Q1, R1, P2) > 0.0f)                                                               \
                            return true;                                                                               \
                        else                                                                                           \
                            return false;                                                                              \
                    else                                                                                               \
                        return false;                                                                                  \
                }                                                                                                      \
            else if(ORIENT_2D(P1, Q2, Q1) < 0.0f)                                                                      \
                if(ORIENT_2D(R2, Q2, R1) < 0.0f)                                                                       \
                    if(ORIENT_2D(Q1, R1, Q2) > 0.0f)                                                                   \
                        return true;                                                                                   \
                    else                                                                                               \
                        return false;                                                                                  \
                else                                                                                                   \
                    return false;                                                                                      \
            else                                                                                                       \
                return false;                                                                                          \
        else if(ORIENT_2D(R2, P2, R1) > 0.0f)                                                                          \
            if(ORIENT_2D(Q1, R1, R2) > 0.0f)                                                                           \
                if(ORIENT_2D(P1, P2, R1) > 0.0f)                                                                       \
                    return true;                                                                                       \
                else                                                                                                   \
                    return false;                                                                                      \
            else if(ORIENT_2D(Q1, R1, Q2) > 0.0f)                                                                      \
            {                                                                                                          \
                if(ORIENT_2D(R2, R1, Q2) > 0.0f)                                                                       \
                    return true;                                                                                       \
                else                                                                                                   \
                    return false;                                                                                      \
            }                                                                                                          \
            else                                                                                                       \
                return false;                                                                                          \
        else                                                                                                           \
            return false;                                                                                              \
    \
};

#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2)                                                                 \
    {                                                                                                                  \
        if(ORIENT_2D(R2, P2, Q1) > 0.0f)                                                                               \
        {                                                                                                              \
            if(ORIENT_2D(P1, P2, Q1) > 0.0f)                                                                           \
            {                                                                                                          \
                if(ORIENT_2D(P1, Q1, R2) > 0.0f)                                                                       \
                    return true;                                                                                       \
                else                                                                                                   \
                    return false;                                                                                      \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                if(ORIENT_2D(Q1, R1, P2) > 0.0f)                                                                       \
                {                                                                                                      \
                    if(ORIENT_2D(R1, P1, P2) > 0.0f)                                                                   \
                        return true;                                                                                   \
                    else                                                                                               \
                        return false;                                                                                  \
                }                                                                                                      \
                else                                                                                                   \
                    return false;                                                                                      \
            }                                                                                                          \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            if(ORIENT_2D(R2, P2, R1) > 0.0f)                                                                           \
            {                                                                                                          \
                if(ORIENT_2D(P1, P2, R1) > 0.0f)                                                                       \
                {                                                                                                      \
                    if(ORIENT_2D(P1, R1, R2) > 0.0f)                                                                   \
                        return true;                                                                                   \
                    else                                                                                               \
                    {                                                                                                  \
                        if(ORIENT_2D(Q1, R1, R2) > 0.0f)                                                               \
                            return true;                                                                               \
                        else                                                                                           \
                            return false;                                                                              \
                    }                                                                                                  \
                }                                                                                                      \
                else                                                                                                   \
                    return false;                                                                                      \
            }                                                                                                          \
            else                                                                                                       \
                return false;                                                                                          \
        }                                                                                                              \
    }

bool ccw_tri_tri_intersection_2d(const point2d& p1, const point2d& q1, const point2d& r1, const point2d& p2,
                                 const point2d& q2, const point2d& r2);
bool TrianglesOverlap(const point2d* t1, const point2d* t2);

double pointLineDistance3D(const point3d& point, const point3d& linePoint, const point3d& lineVectNormalized);
point3d closestPointToLine3D(const point3d* point, const point3d* linePoint, const point3d* lineVectNormalized);
point2d closestPointToLine2D(const point2d& point, const point2d& linePoint, const point2d& lineVectNormalized);
point2d closestPointToLine2D(const point2d& point, const point3d& line);
float pointLineDistance2DPAB(const point2d& point, const point2d& lineA, const point2d& lineB);
float pointLineDistance2DPPv(const point2d& point, const point2d& linePoint, const point2d& lineVect);
float pointPlaneDistance(const point3d& point, const point3d& planePoint, const point3d& planeNormal);
point3d closestPointOnPlaneToPoint(const point3d& point, const point3d& planePoint,
                                   const point3d& planeNormalNormalized);
double orientedPointPlaneDistance(const point3d& point, const point3d& planePoint, const point3d& planeNormal);
point2d planeCoords(const point3d& pointOnPlane, const point3d& planePoint, const point3d& xaxNormalized,
                    const point3d& yaxNormalized);
point2d rectangleCoords(const point3d& P, const point3d& A, const point3d& B, const point3d& C);
void computeRotCS(point3d* xax, point3d* yax, const point3d* n);
bool lineLineIntersect(float* k, float* l, point3d* llis, point3d* lli1, point3d* lli2, const point3d& p1,
                       const point3d& p2, const point3d& p3, const point3d& p4);
bool lineLineIntersectLeft(point3d& out, const point3d& p1, const point3d& p2, const point3d& p3, const point3d& p4);
bool lineLineIntersect(point3d& out, const point3d& p1, const point3d& v1, const point3d& p2, const point3d& v2);
point3d linePlaneIntersect(const point3d& linePoint, const point3d& lineVect, const point3d& planePoint,
                           const point3d& planeNormal);
void linePlaneIntersect(point3d* out, const point3d* linePoint, const point3d* lineVect, const point3d* planePoint,
                        const point3d* planeNormal);
bool lineSegmentPlaneIntersect(point3d* lp, const point3d& linePointA, const point3d& linePointB,
                               const point3d& planePoint, const point3d& planeNormal);
// this angle is always between 0 and 90
float notOrientedangleBetwV1andV2(const point3d& iV1, const point3d& iV2);
// this angle is always between -180 and 180
float angleBetwUnitV1andUnitV2(const point3d& V1, const point3d& V2);
float angleBetwV1andV2(const point3d& iV1, const point3d& iV2);
float angleBetwABandAC(const point3d& A, const point3d& B, const point3d& C);
// return number from 0 to 360
// N has to be ortogonal to V1 and V2
float signedAngleBetwV1andV2(const point3d& iV1, const point3d& iV2, const point3d& Nnormalized);
void rotPointAroundVect(double* out, const double* X, const double* vect, const double angle);
void rotPointAroundVect(point3d* out, const point3d* X, const point3d* vect, const float angle);
point2d getLineTriangleIntersectBarycCoords(point3d* P, const point3d* A, const point3d* B, const point3d* C,
                                            const point3d* linePoint, const point3d* lineVect);
bool isLineInTriangle(point3d* P, const point3d* A, const point3d* B, const point3d* C, const point3d* linePoint,
                      const point3d* lineVect);
bool isLineSegmentInTriangle(point3d& lpi, const point3d& A, const point3d& B, const point3d& C,
                             const point3d& linePoint1, const point3d& linePoint2);

// P = A + u * (C - A) + v * (B - A)
point2d computeBarycentricCoordinates(const point2d& A, const point2d& B, const point2d& C, const point2d& P);
bool isPointInTriangle(const point2d& barycUv);
bool isPointInTriangle(const point2d& A, const point2d& B, const point2d& C, const point2d& P);
bool lineSegmentsIntersect2DTest(const point2d* A, const point2d* B, const point2d* C, const point2d* D);
bool lineSegmentsIntersect2DTest(point2d* S, const point2d* A, const point2d* B, const point2d* C, const point2d* D);

inline void lineLineIntersect2D(point2d& out, const point2d& A, const point2d& B, const point2d& C, const point2d& D)
{
    float r = ((A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y)) /
              ((B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x));
    // float s = ((A.y-C.y)*(B.x-A.x)-(A.x-C.x)*(B.y-A.y))/((B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x));
    // return *A+(*B-*A)*r;
    out.x = A.x + (B.x - A.x) * r;
    out.y = A.y + (B.y - A.y) * r;
}

inline void lineLineIntersect2D(point2d& out, const point2d& A, const point2d& B, const point3d& line)
{
    float r = -line.z / (line.x * (A.x + (B.x - A.x)) + line.y * (A.y + (B.y - A.y)));
    out.x = A.x + (B.x - A.x) * r;
    out.y = A.y + (B.y - A.y) * r;
}

inline void lineLineIntersect2D(point2d& out, const point3d& line1, const point3d& line2)
{
    out.y = (-line2.z / line2.y - (line2.x / line2.y) * (-line1.z / line1.x)) /
            (1.0f - (line2.x / line2.y) * (line1.y / line1.x));
    out.x = (-line1.z - line1.y * out.y) / line1.x;
}

void getOrientedPlaneFor4Points(double* a, double* b, double* c, double* d, point3d* planeA, point3d* planeB,
                                point3d* planeC, point3d* dir);
void getOrientedPlaneForPlaneAndPoint(double* a, double* b, double* c, double* d, point3d* planePoint,
                                      point3d* planeNormal, point3d* dir);

bool isPointInTetrahedron(const point3d& p, const point3d* tet);
bool interectsTriangleTriangle(point3d* tri1, point3d* tri2);
bool interectsTriangleTetrahedron(point3d* tri, point3d* tet);
bool interectsTetrahedronTetrahedron(point3d* tet1, point3d* tet2);
point2d circleCircleIntersection(float d, float r1, float r2);
bool halfPlaneTest(point2d& pt, point2d& linePt, point2d& lineNormal);
