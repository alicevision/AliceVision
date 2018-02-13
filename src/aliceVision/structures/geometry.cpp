// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "geometry.hpp"
#include <aliceVision/structures/geometryTriTri.hpp>

#include <cfloat>


double pointLineDistance3D(const Point3d& point, const Point3d& linePoint, const Point3d& lineVectNormalized)
{
    return cross(lineVectNormalized, linePoint - point).size();
}

Point3d closestPointToLine3D(const Point3d* point, const Point3d* linePoint, const Point3d* lineVectNormalized)
{
    return (*linePoint) + (*lineVectNormalized) * dot((*lineVectNormalized), (*point) - (*linePoint));
}

Point2d closestPointToLine2D(const Point2d& point, const Point2d& linePoint, const Point2d& lineVectNormalized)
{
    return linePoint + lineVectNormalized * dot(lineVectNormalized, point - linePoint);
}

Point2d closestPointToLine2D(const Point2d& point, const Point3d& line)
{
    Point2d linePoint, lineVectNormalized;
    lineVectNormalized.x = line.y;
    lineVectNormalized.y = -line.x;
    lineVectNormalized = lineVectNormalized.normalize();

    if(line.y > 0.00000001f)
    {
        linePoint.x = 0.0f;
        linePoint.y = -line.z / line.y;
    }
    else
    {
        linePoint.y = 0.0f;
        linePoint.x = -line.z / line.x;
    }

    return linePoint + lineVectNormalized * dot(lineVectNormalized, point - linePoint);
}

float pointLineDistance2DPAB(const Point2d& point, const Point2d& lineA, const Point2d& lineB)
{
    float x0 = lineA.x;
    float y0 = lineA.y;
    float x1 = lineB.x;
    float y1 = lineB.y;
    float x = point.x;
    float y = point.y;

    float v[2], d;
    v[0] = x1 - x0;
    v[1] = y1 - y0;
    d = sqrt(v[0] * v[0] + v[1] * v[1]);

    v[0] /= d;
    v[1] /= d;

    float a = v[1];
    float b = -v[0];
    float c = -(a * x0 + b * y0);

    d = fabs(a * x + b * y + c) / sqrt(a * a + b * b);

    return d;
}

float pointLineDistance2DPPv(const Point2d& point, const Point2d& linePoint, const Point2d& lineVect)
{
    float a = lineVect.y;
    float b = -lineVect.x;
    float c = -(a * linePoint.x + b * linePoint.y);
    return fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}

float pointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal)
{
    return fabs(dot(point, planeNormal) - dot(planePoint, planeNormal)) / sqrt(dot(planeNormal, planeNormal));
}

Point3d closestPointOnPlaneToPoint(const Point3d& point, const Point3d& planePoint,
                                   const Point3d& planeNormalNormalized)
{
    return point - planeNormalNormalized * dot(planeNormalNormalized, point - planePoint);
}

double orientedPointPlaneDistance(const Point3d& point, const Point3d& planePoint, const Point3d& planeNormal)
{
    return (dot(point, planeNormal) - dot(planePoint, planeNormal)) / std::sqrt(dot(planeNormal, planeNormal));
}

Point2d planeCoords(const Point3d& pointOnPlane, const Point3d& planePoint, const Point3d& xaxNormalized,
                    const Point3d& yaxNormalized)
{
    Point2d p;
    p.x = orientedPointPlaneDistance(pointOnPlane, planePoint, xaxNormalized);
    p.y = orientedPointPlaneDistance(pointOnPlane, planePoint, yaxNormalized);
    return p;
}

Point2d rectangleCoords(const Point3d& P, const Point3d& A, const Point3d& B, const Point3d& C)
{
    Point3d v0 = B - A;
    Point3d v1 = C - A;

    Point2d p;
    p.x = orientedPointPlaneDistance(P, A, v0.normalize()) / v0.size();
    p.y = orientedPointPlaneDistance(P, A, v1.normalize()) / v1.size();

    return p;

    /*
            Point3d v0 = B - A;
            Point3d v1 = C - A;

            Point3d n = cross(v0.normalize(), v1.normalize()).normalize();


            // Compute vectors
            Point3d v2 = P - A;

            // Compute dot products
            float dot00 = dot(v0, v0);
            float dot01 = dot(v0, v1);
            float dot02 = dot(v0, v2);
            float dot11 = dot(v1, v1);
            float dot12 = dot(v1, v2);

            // Compute barycentric coordinates
            float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
            float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            return Point2d(u,v);
            */
}

void computeRotCS(Point3d* xax, Point3d* yax, const Point3d* n)
{
    xax->x = -n->y + n->z; // get any cross product
    xax->y = +n->x + n->z;
    xax->z = -n->x - n->y;
    if(fabs(xax->x) < 0.0000001 && fabs(xax->y) < 0.0000001 && fabs(xax->z) < 0.0000001)
    {
        xax->x = -n->y - n->z; // get any cross product (complementar)
        xax->y = +n->x - n->z;
        xax->z = +n->x + n->y;
    }
    *xax = xax->normalize();

    *yax = cross(*n, *xax);
}

bool lineLineIntersect(float* k, float* l, Point3d* llis, Point3d* lli1, Point3d* lli2, const Point3d& p1,
                       const Point3d& p2, const Point3d& p3, const Point3d& p4)
{
    /*
    %  [pa, pb, mua, mub] = LineLineIntersect(p1,p2,p3,p4)
    %
    %   Calculates the line segment pa_pb that is the shortest route
    %   between two lines p1_p2 and p3_p4. Calculates also the values of
    %   mua and mub where
    %        pa = p1 + mua (p2 - p1)
    %        pb = p3 + mub (p4 - p3)
    %
    %   Returns a MATLAB error if no solution exists.
    %
    %   This a simple conversion to MATLAB of the C code posted by Paul
    %   Bourke at
    %   http://astronomy.swin.edu.au/~pbourke/geometry/lineline3d/. The
    %   author of this all too imperfect translation is Cristian Dima
    %   (csd@cmu.edu)
    */

    float d1343, d4321, d1321, d4343, d2121, denom, numer, p13[3], p43[3], p21[3], pa[3], pb[3], muab[2];

    p13[0] = p1.x - p3.x;
    p13[1] = p1.y - p3.y;
    p13[2] = p1.z - p3.z;

    p43[0] = p4.x - p3.x;
    p43[1] = p4.y - p3.y;
    p43[2] = p4.z - p3.z;

    if((fabs(p43[0]) < FLT_EPSILON) && (fabs(p43[1]) < FLT_EPSILON) && (fabs(p43[2]) < FLT_EPSILON))
    {
        return false;
    }

    p21[0] = p2.x - p1.x;
    p21[1] = p2.y - p1.y;
    p21[2] = p2.z - p1.z;

    if((fabs(p21[0]) < FLT_EPSILON) && (fabs(p21[1]) < FLT_EPSILON) && (fabs(p21[2]) < FLT_EPSILON))
    {
        return false;
    }

    d1343 = p13[0] * p43[0] + p13[1] * p43[1] + p13[2] * p43[2];
    d4321 = p43[0] * p21[0] + p43[1] * p21[1] + p43[2] * p21[2];
    d1321 = p13[0] * p21[0] + p13[1] * p21[1] + p13[2] * p21[2];
    d4343 = p43[0] * p43[0] + p43[1] * p43[1] + p43[2] * p43[2];
    d2121 = p21[0] * p21[0] + p21[1] * p21[1] + p21[2] * p21[2];

    denom = d2121 * d4343 - d4321 * d4321;

    if(fabs(denom) < FLT_EPSILON)
    {
        return false;
    }

    numer = d1343 * d4321 - d1321 * d4343;

    muab[0] = numer / denom;
    muab[1] = (d1343 + d4321 * muab[0]) / d4343;

    pa[0] = p1.x + muab[0] * p21[0];
    pa[1] = p1.y + muab[0] * p21[1];
    pa[2] = p1.z + muab[0] * p21[2];

    pb[0] = p3.x + muab[1] * p43[0];
    pb[1] = p3.y + muab[1] * p43[1];
    pb[2] = p3.z + muab[1] * p43[2];

    Point3d S;
    llis->x = (pa[0] + pb[0]) / 2.0;
    llis->y = (pa[1] + pb[1]) / 2.0;
    llis->z = (pa[2] + pb[2]) / 2.0;

    *k = muab[0];
    *l = muab[1];

    lli1->x = pa[0];
    lli1->y = pa[1];
    lli1->z = pa[2];

    lli2->x = pb[0];
    lli2->y = pb[1];
    lli2->z = pb[2];

    return true;
}

bool lineLineIntersect(Point3d& out, const Point3d& p1, const Point3d& v1, const Point3d& p2, const Point3d& v2)
{
    float k, l;
    Point3d lli1, lli2;
    Point3d p1v1 = p1 + v1;
    Point3d p2v2 = p2 + v2;
    return lineLineIntersect(&k, &l, &out, &lli1, &lli2, p1, p1v1, p2, p2v2);
}

bool lineLineIntersectLeft(Point3d& out, const Point3d& p1, const Point3d& p2, const Point3d& p3, const Point3d& p4)
{
    float k, l;
    Point3d llis, lli2;
    return lineLineIntersect(&k, &l, &llis, &out, &lli2, p1, p2, p3, p4);
}

Point3d linePlaneIntersect(const Point3d& linePoint, const Point3d& lineVect, const Point3d& planePoint,
                           const Point3d& planeNormal)
{
    float planePoint_x = planePoint.x;
    float planePoint_y = planePoint.y;
    float planePoint_z = planePoint.z;
    float planeNormal_x = planeNormal.x;
    float planeNormal_y = planeNormal.y;
    float planeNormal_z = planeNormal.z;
    float linePoint_x = linePoint.x;
    float linePoint_y = linePoint.y;
    float linePoint_z = linePoint.z;
    float lineVect_x = lineVect.x;
    float lineVect_y = lineVect.y;
    float lineVect_z = lineVect.z;
    float k = ((planePoint_x * planeNormal_x + planePoint_y * planeNormal_y + planePoint_z * planeNormal_z) - (planeNormal_x * linePoint_x + planeNormal_y * linePoint_y + planeNormal_z * linePoint_z)) / (planeNormal_x * lineVect_x + planeNormal_y * lineVect_y + planeNormal_z * lineVect_z);
    return linePoint + lineVect * k;
    //---KP---float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePoint)) / dot(planeNormal, lineVect);
    //---KP---return linePoint + lineVect * k;
}

void linePlaneIntersect(Point3d* out, const Point3d* linePoint, const Point3d* lineVect, const Point3d* planePoint,
                        const Point3d* planeNormal)
{
    float planePoint_x = planePoint->x;
    float planePoint_y = planePoint->y;
    float planePoint_z = planePoint->z;
    float planeNormal_x = planeNormal->x;
    float planeNormal_y = planeNormal->y;
    float planeNormal_z = planeNormal->z;
    float linePoint_x = linePoint->x;
    float linePoint_y = linePoint->y;
    float linePoint_z = linePoint->z;
    float lineVect_x = lineVect->x;
    float lineVect_y = lineVect->y;
    float lineVect_z = lineVect->z;
    float k = ((planePoint_x * planeNormal_x + planePoint_y * planeNormal_y + planePoint_z * planeNormal_z) - (planeNormal_x * linePoint_x + planeNormal_y * linePoint_y + planeNormal_z * linePoint_z)) / (planeNormal_x * lineVect_x + planeNormal_y * lineVect_y + planeNormal_z * lineVect_z);
    out->x = linePoint_x + (lineVect_x) * k;
    out->y = linePoint_y + (lineVect_y) * k;
    out->z = linePoint_z + (lineVect_z) * k;
    //---KP---float k = (dot(*planePoint, *planeNormal) - dot(*planeNormal, *linePoint)) / dot(*planeNormal, *lineVect);
    //---KP---*out = *linePoint + (*lineVect) * k;
}

bool lineSegmentPlaneIntersect(Point3d* lp, const Point3d& linePointA, const Point3d& linePointB,
                               const Point3d& planePoint, const Point3d& planeNormal)
{
    Point3d lineVect = (linePointB - linePointA).normalize();
    float size = (linePointB - linePointA).size();

    float k = (dot(planePoint, planeNormal) - dot(planeNormal, linePointA)) / dot(planeNormal, lineVect);
    *lp = linePointA + lineVect * k;

    return (k >= 0.0f) && (k <= size);
}

// this angle is always between 0 and 90
float notOrientedangleBetwV1andV2(const Point3d& iV1, const Point3d& iV2)
{
    Point3d V1, V2;
    V1 = iV1.normalize();
    V2 = iV2.normalize();

    float a = fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI / 180.0));
    if(a > 90.0f)
    {
        a = 180.0f - a;
    }

    return a;
}

float angleBetwUnitV1andUnitV2(Point3d& V1, Point3d& V2)
{
    return fabs(acos(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z) / (M_PI / 180.0f));
}

// this angle is always between 0 and 180
float angleBetwV1andV2(const Point3d& iV1, const Point3d& iV2)
{
    Point3d V1, V2;
    V1 = iV1.normalize();
    V2 = iV2.normalize();

    float a = acos((float)(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z));
    if(std::isnan(a))
    {
        a = 0.0f;
    }

    return fabs(a / (M_PI / 180.0));
}

float angleBetwABandAC(const Point3d& A, const Point3d& B, const Point3d& C)
{
    Point3d V1, V2;
    V1 = B - A;
    V2 = C - A;
    V1 = V1.normalize();
    V2 = V2.normalize();

    float a = acos((float)(V1.x * V2.x + V1.y * V2.y + V1.z * V2.z));
    if(std::isnan(a))
    {
        a = 0.0f;
    }

    return fabs(a) / (M_PI / 180.0);
}

float cosAngleBetwABandAC(Point3d& A, Point3d& B, Point3d& C)
{
    Point3d V1, V2;
    V1 = B - A;
    V2 = C - A;
    V1 = V1.normalize();
    V2 = V2.normalize();

    return V1.x * V2.x + V1.y * V2.y + V1.z * V2.z;
}

// return number from 0 to 360
// N has to be ortogonal to V1 and V2
float signedAngleBetwV1andV2(const Point3d& iV1, const Point3d& iV2, const Point3d& Nnormalized)
{
    Point3d V1, V2, N;
    V1 = iV1.normalize();
    V2 = iV2.normalize();

    // signed_angle = atan2(  N * ( V1 x V2 ), V1 * V2  );
    // where * is dot product and x is cross product
    // N is the normal to the polygon
    // ALL vectors: N, V1, V2 must be normalized

    float a = atan2(dot(Nnormalized, cross(V1, V2)), dot(V1, V2)) / (M_PI / 180.0);
    if(a < 0.0f)
    {
        a = 360.0f + a;
    }
    return a;

    /*
    float angle = acos(dot(V1, V2))/ (M_PI/180.0f);
    Point3d cr = cross(V1, V2);
    if (dot(Nnormalized, cr) < 0.0f) { // Or > 0
      angle = 360.0f-angle;
    };

    return angle;
    */
}

void rotPointAroundVect(double* out, const double* X, const double* vect, const double angle)
{
    double ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    double sizeX = sqrt(X[0] * X[0] + X[1] * X[1] + X[2] * X[2]);
    x = X[0] / sizeX;
    y = X[1] / sizeX;
    z = X[2] / sizeX;
    u = vect[0];
    v = vect[1];
    w = vect[2];

    /*Rotate the point (x,y,z) around the vector (u,v,w)*/
    ux = u * x;
    uy = u * y;
    uz = u * z;
    vx = v * x;
    vy = v * y;
    vz = v * z;
    wx = w * x;
    wy = w * y;
    wz = w * z;
    sa = sin((double)angle * (M_PI / 180.0));
    ca = cos((double)angle * (M_PI / 180.0));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    u = sqrt(x * x + y * y + z * z);
    x /= u;
    y /= u;
    z /= u;

    out[0] = x * sizeX;
    out[1] = y * sizeX;
    out[2] = z * sizeX;
}

void rotPointAroundVect(Point3d* out, const Point3d* X, const Point3d* vect, const float angle)
{
    double o[3];
    double dX[3] = {(double)X->x, (double)X->y, (double)X->z};
    double dvect[3] = {(double)vect->x, (double)vect->y, (double)vect->z};
    rotPointAroundVect(o, dX, dvect, angle);
    out->x = o[0];
    out->y = o[1];
    out->z = o[2];
}

Point2d getLineTriangleIntersectBarycCoords(Point3d* P, const Point3d* A, const Point3d* B, const Point3d* C,
                                            const Point3d* linePoint, const Point3d* lineVect)
{
    float A_x = A->x;
    float A_y = A->y;
    float A_z = A->z;
    float linePoint_x = linePoint->x;
    float linePoint_y = linePoint->y;
    float linePoint_z = linePoint->z;
    float lineVect_x = lineVect->x;
    float lineVect_y = lineVect->y;
    float lineVect_z = lineVect->z;
    float v0_x = C->x - A_x;
    float v0_y = C->y - A_y;
    float v0_z = C->z - A_z;
    float v1_x = B->x - A_x;
    float v1_y = B->y - A_y;
    float v1_z = B->z - A_z;
    float _n_x = v0_y * v1_z - v0_z * v1_y;
    float _n_y = v0_z * v1_x - v0_x * v1_z;
    float _n_z = v0_x * v1_y - v0_y * v1_x;
    float k = ((A_x * _n_x + A_y * _n_y + A_z * _n_z) - (_n_x * linePoint_x + _n_y * linePoint_y + _n_z * linePoint_z)) / (_n_x * lineVect_x + _n_y * lineVect_y + _n_z * lineVect_z);
    float P_x = linePoint_x + lineVect_x * k;
    float P_y = linePoint_y + lineVect_y * k;
    float P_z = linePoint_z + lineVect_z * k;
    // Compute vectors
    float v2_x = P_x - A_x;
    float v2_y = P_y - A_y;
    float v2_z = P_z - A_z;
    // Compute dot products
    float dot00 = (v0_x * v0_x + v0_y * v0_y + v0_z * v0_z);
    float dot01 = (v0_x * v1_x + v0_y * v1_y + v0_z * v1_z);
    float dot02 = (v0_x * v2_x + v0_y * v2_y + v0_z * v2_z);
    float dot11 = (v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);
    float dot12 = (v1_x * v2_x + v1_y * v2_y + v1_z * v2_z);
    // Compute barycentric coordinates
    float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
    P->x = P_x;
    P->y = P_y;
    P->z = P_z;

    //---KP---Point3d v0 = *C - *A;
    //---KP---Point3d v1 = *B - *A;
    //---KP---Point3d n = cross(v0.normalize(), v1.normalize()).normalize();
    //---KP---linePlaneIntersect(P, linePoint, lineVect, A, &n);
    //---KP---// Compute vectors
    //---KP---Point3d v2 = *P - *A;
    //---KP---// Compute dot products
    //---KP---float dot00 = dot(v0, v0);
    //---KP---float dot01 = dot(v0, v1);
    //---KP---float dot02 = dot(v0, v2);
    //---KP---float dot11 = dot(v1, v1);
    //---KP---float dot12 = dot(v1, v2);
    //---KP---// Compute barycentric coordinates
    //---KP---float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    //---KP---float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    //---KP---float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // if ((u==0.0f)||(v==0.0f)||(u+v==1.0f)) {
    //	printf("WARNNG : isLineInTriangle - is on border! \n");
    //};

    // Check if point is in triangle
    return Point2d(u, v);
}

bool isLineInTriangle(Point3d* P, const Point3d* A, const Point3d* B, const Point3d* C, const Point3d* linePoint,
                      const Point3d* lineVect)
{
    return isPointInTriangle(getLineTriangleIntersectBarycCoords(P, A, B, C, linePoint, lineVect));
}

bool isLineSegmentInTriangle(Point3d& lpi, const Point3d& A, const Point3d& B, const Point3d& C,
                             const Point3d& linePoint1, const Point3d& linePoint2)
{
    Point3d v0 = C - A;
    Point3d v1 = B - A;

    Point3d n = cross(v0.normalize(), v1.normalize()).normalize();

    Point3d lineVect = linePoint2 - linePoint1;

    // linePlaneIntersect(&P, linePoint, lineVect, A, &n);
    float k = (dot(A, n) - dot(n, linePoint1)) / dot(n, lineVect);
    lpi = linePoint1 + lineVect * k;

    // Compute vectors
    Point3d v2 = lpi - A;

    // Compute dot products
    float dot00 = dot(v0, v0);
    float dot01 = dot(v0, v1);
    float dot02 = dot(v0, v2);
    float dot11 = dot(v1, v1);
    float dot12 = dot(v1, v2);

    // Compute barycentric coordinates
    float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    // Check if point is in triangle
    return (k >= 0.0) && (k <= 1.0) && (u > 0.0) && (v > 0.0) && (u + v < 1.0);
}

// P = A + u * (C - A) + v * (B - A)
Point2d computeBarycentricCoordinates(const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& P)
{
    // http://www.blackpawn.com/texts/pointinpoly/default.html

    Point2d v0 = C - A;
    Point2d v1 = B - A;
    Point2d v2 = P - A;

    // Compute dot products
    float dot00 = dot(v0, v0);
    float dot01 = dot(v0, v1);
    float dot02 = dot(v0, v2);
    float dot11 = dot(v1, v1);
    float dot12 = dot(v1, v2);

    // Compute barycentric coordinates
    float invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return Point2d(u, v);
}

bool isPointInTriangle(const Point2d& barycUv)
{
    // Check if point is in triangle
    return (barycUv.x >= 0.0) && (barycUv.y >= 0.0) && (barycUv.x + barycUv.y <= 1.0);
}

bool isPointInTriangle(const Point2d& A, const Point2d& B, const Point2d& C, const Point2d& P)
{
    // Check if point is in triangle
    return isPointInTriangle(computeBarycentricCoordinates(A, B, C, P));
}

bool lineSegmentsIntersect2DTest(const Point2d* A, const Point2d* B, const Point2d* C, const Point2d* D)
{
    float r = ((A->y - C->y) * (D->x - C->x) - (A->x - C->x) * (D->y - C->y)) /
              ((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
    float s = ((A->y - C->y) * (B->x - A->x) - (A->x - C->x) * (B->y - A->y)) /
              ((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
    return ((r >= 0.0) && (r <= 1.0) && (s >= 0.0) && (s <= 1.0));
}

bool lineSegmentsIntersect2DTest(Point2d* S, const Point2d* A, const Point2d* B, const Point2d* C, const Point2d* D)
{
    float r = ((A->y - C->y) * (D->x - C->x) - (A->x - C->x) * (D->y - C->y)) /
              ((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
    float s = ((A->y - C->y) * (B->x - A->x) - (A->x - C->x) * (B->y - A->y)) /
              ((B->x - A->x) * (D->y - C->y) - (B->y - A->y) * (D->x - C->x));
    *S = *A + (*B - *A) * r;
    return ((r >= 0.0) && (r <= 1.0) && (s >= 0.0) && (s <= 1.0));
}

bool ccw_tri_tri_intersection_2d(const Point2d& p1, const Point2d& q1, const Point2d& r1, const Point2d& p2,
                                 const Point2d& q2, const Point2d& r2)
{
    if(ORIENT_2D(p2, q2, p1) > 0.0f)
    {
        if(ORIENT_2D(q2, r2, p1) > 0.0f)
        {
            if(ORIENT_2D(r2, p2, p1) > 0.0f)
                return true;
            else
                INTERSECTION_TEST_EDGE(p1, q1, r1, p2, q2, r2)
        }
        else
        {
            if(ORIENT_2D(r2, p2, p1) > 0.0f)
                INTERSECTION_TEST_EDGE(p1, q1, r1, r2, p2, q2)
            else
                INTERSECTION_TEST_VERTEX(p1, q1, r1, p2, q2, r2)
        }
    }
    else
    {
        if(ORIENT_2D(q2, r2, p1) > 0.0f)
        {
            if(ORIENT_2D(r2, p2, p1) > 0.0f)
                INTERSECTION_TEST_EDGE(p1, q1, r1, q2, r2, p2)
            else
                INTERSECTION_TEST_VERTEX(p1, q1, r1, q2, r2, p2)
        }
        else
            INTERSECTION_TEST_VERTEX(p1, q1, r1, r2, p2, q2)
    }
}

bool TrianglesOverlap(const Point2d* t1, const Point2d* t2)
{
    if(ORIENT_2D(t1[0], t1[1], t1[2]) < 0.0f)
        if(ORIENT_2D(t2[0], t2[1], t2[2]) < 0.0f)
            return ccw_tri_tri_intersection_2d(t1[0], t1[2], t1[1], t2[0], t2[2], t2[1]);
        else
            return ccw_tri_tri_intersection_2d(t1[0], t1[2], t1[1], t2[0], t2[1], t2[2]);
    else if(ORIENT_2D(t2[0], t2[1], t2[2]) < 0.0f)
        return ccw_tri_tri_intersection_2d(t1[0], t1[1], t1[2], t2[0], t2[2], t2[1]);
    else
        return ccw_tri_tri_intersection_2d(t1[0], t1[1], t1[2], t2[0], t2[1], t2[2]);
}

void getOrientedPlaneFor4Points(double* a, double* b, double* c, double* d, Point3d* planeA, Point3d* planeB,
                                Point3d* planeC, Point3d* dir)
{
    // ABCD ... N = (B-A)x(C-A) if (dot(N,D-A) < 0) N = -N

    Point3d v1 = (*planeB - *planeA).normalize();
    Point3d v2 = (*planeC - *planeA).normalize();
    Point3d n = cross(v1, v2);
    if(dot(n, *dir - *planeA) < 0.0)
    {
        n.x = -n.x;
        n.y = -n.y;
        n.z = -n.z;
    }

    *a = n.x;
    *b = n.y;
    *c = n.z;
    *d = -dot(n, *planeA);
}

void getOrientedPlaneForPlaneAndPoint(double* a, double* b, double* c, double* d, Point3d* planePoint,
                                      Point3d* planeNormal, Point3d* dir)
{
    // ABCD ... N = (B-A)x(C-A) if (dot(N,D-A) < 0) N = -N

    Point3d n = *planeNormal;
    if(dot(n, *dir - *planePoint) < 0.0)
    {
        n.x = -n.x;
        n.y = -n.y;
        n.z = -n.z;
    }

    *a = n.x;
    *b = n.y;
    *c = n.z;
    *d = -dot(n, *planePoint);
}

bool isPointInTetrahedron(const Point3d& p, const Point3d* tet)
{
    for(int i1 = 0; i1 < 4; i1++)
    {
        for(int i2 = i1 + 1; i2 < 4; i2++)
        {
            for(int i3 = i2 + 1; i3 < 4; i3++)
            {
                int i4;
                if((i1 != 0) && (i2 != 0) && (i3 != 0))
                {
                    i4 = 0;
                }
                if((i1 != 1) && (i2 != 1) && (i3 != 1))
                {
                    i4 = 1;
                }
                if((i1 != 2) && (i2 != 2) && (i3 != 2))
                {
                    i4 = 2;
                }
                if((i1 != 3) && (i2 != 3) && (i3 != 3))
                {
                    i4 = 3;
                }

                Point3d a = tet[i1];
                Point3d b = tet[i2];
                Point3d c = tet[i3];
                Point3d d = tet[i4];
                Point3d n = cross((a - b).normalize(), (b - c).normalize()).normalize();
                float d1 = orientedPointPlaneDistance(p, a, n);
                float d2 = orientedPointPlaneDistance(d, a, n);
                if(d1 * d2 < 0.0f)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool interectsTriangleTriangle(Point3d* tri1, Point3d* tri2)
{
    bool ok = (bool)tri_tri_intersect(tri1[0].m, tri1[1].m, tri1[2].m, tri2[0].m, tri2[1].m, tri2[2].m);
    return ok;
}

bool interectsTriangleTetrahedron(Point3d* tri, Point3d* tet)
{
    for(int i1 = 0; i1 < 4; i1++)
    {
        for(int i2 = i1 + 1; i2 < 4; i2++)
        {
            for(int i3 = i2 + 1; i3 < 4; i3++)
            {
                int i4;
                if((i1 != 0) && (i2 != 0) && (i3 != 0))
                {
                    i4 = 0;
                }
                if((i1 != 1) && (i2 != 1) && (i3 != 1))
                {
                    i4 = 1;
                }
                if((i1 != 2) && (i2 != 2) && (i3 != 2))
                {
                    i4 = 2;
                }
                if((i1 != 3) && (i2 != 3) && (i3 != 3))
                {
                    i4 = 3;
                }

                Point3d a = tet[i1];
                Point3d b = tet[i2];
                Point3d c = tet[i3];

                bool ok = (bool)tri_tri_intersect(tri[0].m, tri[1].m, tri[2].m, a.m, b.m, c.m);
                if(ok)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

bool interectsTetrahedronTetrahedron(Point3d* tet1, Point3d* tet2)
{
    for(int i1 = 0; i1 < 4; i1++)
    {
        for(int i2 = i1 + 1; i2 < 4; i2++)
        {
            for(int i3 = i2 + 1; i3 < 4; i3++)
            {
                int i4;
                if((i1 != 0) && (i2 != 0) && (i3 != 0))
                {
                    i4 = 0;
                }
                if((i1 != 1) && (i2 != 1) && (i3 != 1))
                {
                    i4 = 1;
                }
                if((i1 != 2) && (i2 != 2) && (i3 != 2))
                {
                    i4 = 2;
                }
                if((i1 != 3) && (i2 != 3) && (i3 != 3))
                {
                    i4 = 3;
                }

                Point3d tri[3];
                tri[0] = tet1[i1];
                tri[1] = tet1[i2];
                tri[2] = tet1[i3];
                // Point3d d = tet[i4];
                // Point3d n = cross((a-b).normalize(),(b-c).normalize()).normalize();

                bool ok = interectsTriangleTetrahedron(tri, tet2);
                if(ok)
                {
                    return true;
                }
            }
        }
    }

    return false;
}

// http://mathworld.wolfram.com/Circle-CircleIntersection.html
Point2d circleCircleIntersection(float d, float r1, float r2)
{
    float R = r1;
    float r = r2;
    float k = (d * d - r * r + R * R);
    float x = k / (2.0f * d);
    float a = (1.0f / d) * sqrt(4.0f * d * d * R * R - k * k);

    // printf("circleCircleIntersection d %f, r1 %f, r2 %f \n",d,r1,r2);
    // printf("k %f, x %f, a %f \n",k,x,a);

    Point2d o;
    o.x = x;
    o.y = a / 2.0f;

    return o;
}

// http://mrl.nyu.edu/~dzorin/ug-graphics/lectures/lecture5/sld008.htm
// is the point pt on the same side of the half plane defined by the line point and the normal
bool halfPlaneTest(Point2d& pt, Point2d& linePt, Point2d& lineNormal)
{
    return (dot(lineNormal, pt - linePt) > 0.0f);
}
