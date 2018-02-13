// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#define _USE_MATH_DEFINES
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>

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

bool ccw_tri_tri_intersection_2d(const Point2d& p1, const Point2d& q1, const Point2d& r1, const Point2d& p2,
                                 const Point2d& q2, const Point2d& r2);
bool TrianglesOverlap(const Point2d* t1, const Point2d* t2);

double pointLineDistance3D(const Point3d& point, const Point3d& linePoint, const Point3d& lineVectNormalized);
Point3d closestPointToLine3D(const Point3d* point, const Point3d* linePoint, const Point3d* lineVectNormalized);
Point2d closestPointToLine2D(const Point2d& point, const Point2d& linePoint, const Point2d& lineVectNormalized);
Point2d closestPointToLine2D(const Point2d& point, const Point3d& line);
float pointLineDistance2DPAB(const Point2d& point, const Point2d& lineA, const Point2d& lineB);
float pointLineDistance2DPPv(const Point2d& point, const Point2d& linePoint, const Point2d& lineVect);
float pointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal);
Point3d closestPointOnPlaneToPoint(const Point3d& point, const Point3d& planePoint,
                                   const Point3d& planeNormalNormalized);
double orientedPointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal);
Point2d planeCoords(const Point3d& pointOnPlane, const Point3d& planePoint, const Point3d& xaxNormalized,
                    const Point3d& yaxNormalized);
Point2d rectangleCoords(const Point3d& P, const Point3d& A, const Point3d& B, const Point3d& C);
void computeRotCS(Point3d* xax, Point3d* yax, const Point3d* n);
bool lineLineIntersect(float* k, float* l, Point3d* llis, Point3d* lli1, Point3d* lli2, const Point3d& p1,
                       const Point3d& p2, const Point3d& p3, const Point3d& p4);
bool lineLineIntersectLeft(Point3d& out, const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4);
bool lineLineIntersect(Point3d& out, const Point3d& p1, const Point3d& v1, const Point3d& p2, const Point3d& v2);
Point3d linePlaneIntersect(const Point3d& linePoint, const Point3d& lineVect, const Point3d& planePoint,
                           const Point3d& planeNormal);
void linePlaneIntersect(Point3d* out, const Point3d* linePoint, const Point3d* lineVect, const Point3d* planePoint,
                        const Point3d* planeNormal);
bool lineSegmentPlaneIntersect(Point3d* lp, const Point3d& linePointA, const Point3d& linePointB,
                               const Point3d& planePoint, const Point3d& planeNormal);
// this angle is always between 0 and 90
float notOrientedangleBetwV1andV2(const Point3d& iV1, const Point3d& iV2);
// this angle is always between -180 and 180
float angleBetwUnitV1andUnitV2(const Point3d& V1, const Point3d& V2);
float angleBetwV1andV2(const Point3d& iV1, const Point3d& iV2);
float angleBetwABandAC(const Point3d& A, const Point3d& B, const Point3d& C);
// return number from 0 to 360
// N has to be ortogonal to V1 and V2
float signedAngleBetwV1andV2(const Point3d& iV1, const Point3d& iV2, const Point3d& Nnormalized);
void rotPointAroundVect(double* out, const double* X, const double* vect, const double angle);
void rotPointAroundVect(Point3d* out, const Point3d* X, const Point3d* vect, const float angle);
Point2d getLineTriangleIntersectBarycCoords(Point3d* P, const Point3d* A, const Point3d* B, const Point3d* C,
                                            const Point3d* linePoint, const Point3d* lineVect);
bool isLineInTriangle(Point3d* P, const Point3d* A, const Point3d* B, const Point3d* C, const Point3d* linePoint,
                      const Point3d* lineVect);
bool isLineSegmentInTriangle(Point3d& lpi, const Point3d& A, const Point3d& B, const Point3d& C,
                             const Point3d& linePoint1, const Point3d& linePoint2);

// P = A + u * (C - A) + v * (B - A)
Point2d computeBarycentricCoordinates(const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& P);
bool isPointInTriangle(const Point2d& barycUv);
bool isPointInTriangle(const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& P);
bool lineSegmentsIntersect2DTest(const Point2d* A, const Point2d* B, const Point2d* C, const Point2d* D);
bool lineSegmentsIntersect2DTest(Point2d* S, const Point2d* A, const Point2d* B, const Point2d* C, const Point2d* D);

inline void lineLineIntersect2D(Point2d& out, const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& D)
{
    float r = ((A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y)) /
              ((B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x));
    // float s = ((A.y-C.y)*(B.x-A.x)-(A.x-C.x)*(B.y-A.y))/((B.x-A.x)*(D.y-C.y)-(B.y-A.y)*(D.x-C.x));
    // return *A+(*B-*A)*r;
    out.x = A.x + (B.x - A.x) * r;
    out.y = A.y + (B.y - A.y) * r;
}

inline void lineLineIntersect2D(Point2d& out, const Point2d& A, const Point2d& B, const Point3d& line)
{
    float r = -line.z / (line.x * (A.x + (B.x - A.x)) + line.y * (A.y + (B.y - A.y)));
    out.x = A.x + (B.x - A.x) * r;
    out.y = A.y + (B.y - A.y) * r;
}

inline void lineLineIntersect2D(Point2d& out, const Point3d& line1, const Point3d& line2)
{
    out.y = (-line2.z / line2.y - (line2.x / line2.y) * (-line1.z / line1.x)) /
            (1.0f - (line2.x / line2.y) * (line1.y / line1.x));
    out.x = (-line1.z - line1.y * out.y) / line1.x;
}

void getOrientedPlaneFor4Points(double* a, double* b, double* c, double* d, Point3d* planeA, Point3d* planeB,
                                Point3d* planeC, Point3d* dir);
void getOrientedPlaneForPlaneAndPoint(double* a, double* b, double* c, double* d, Point3d* planePoint,
                                      Point3d* planeNormal, Point3d* dir);

bool isPointInTetrahedron(const Point3d& p, const Point3d* tet);
bool interectsTriangleTriangle(Point3d* tri1, Point3d* tri2);
bool interectsTriangleTetrahedron(Point3d* tri, Point3d* tet);
bool interectsTetrahedronTetrahedron(Point3d* tet1, Point3d* tet2);
Point2d circleCircleIntersection(float d, float r1, float r2);
bool halfPlaneTest(Point2d& pt, Point2d& linePt, Point2d& lineNormal);
