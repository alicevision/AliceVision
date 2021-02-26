// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>

#include <cmath>

namespace aliceVision {

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
double pointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal);

double orientedPointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal);
void computeRotCS(Point3d* xax, Point3d* yax, const Point3d* n);
bool lineLineIntersect(double* k, double* l, Point3d* llis, Point3d* lli1, Point3d* lli2, const Point3d& p1,
                       const Point3d& p2, const Point3d& p3, const Point3d& p4);
bool lineLineIntersectLeft(Point3d& out, const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4);
bool lineLineIntersect(Point3d& out, const Point3d& p1, const Point3d& v1, const Point3d& p2, const Point3d& v2);
Point3d linePlaneIntersect(const Point3d& linePoint, const Point3d& lineVect, const Point3d& planePoint,
                           const Point3d& planeNormal);
void linePlaneIntersect(Point3d* out, const Point3d* linePoint, const Point3d* lineVect, const Point3d* planePoint,
                        const Point3d* planeNormal);

double angleBetwV1andV2(const Point3d& iV1, const Point3d& iV2);
double angleBetwABandAC(const Point3d& A, const Point3d& B, const Point3d& C);

void rotPointAroundVect(double* out, const double* X, const double* vect, const double angle);
void rotPointAroundVect(Point3d* out, const Point3d* X, const Point3d* vect, const double angle);
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
bool lineSegmentsIntersect2DTest(const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& D);
bool lineSegmentsIntersect2DTest(Point2d& S, const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& D);

bool interectsTriangleTriangle(const Point3d* tri1, const Point3d* tri2);

} // namespace aliceVision
