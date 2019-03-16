// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace depthMap {

struct Patch
{
    float3 p; //< 3d point
    float3 n; //< normal
    float3 x; //< x axis
    float3 y; //< y axis
    float d;  //< pixel size
};

__device__ void rotPointAroundVect(float3& out, float3& X, float3& vect, int angle)
{
    double ux, uy, uz, vx, vy, vz, wx, wy, wz, sa, ca, x, y, z, u, v, w;

    double sizeX = sqrt(dot(X, X));
    x = X.x / sizeX;
    y = X.y / sizeX;
    z = X.z / sizeX;
    u = vect.x;
    v = vect.y;
    w = vect.z;

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
    sa = sin((double)angle * (M_PI / 180.0f));
    ca = cos((double)angle * (M_PI / 180.0f));
    x = u * (ux + vy + wz) + (x * (v * v + w * w) - u * (vy + wz)) * ca + (-wy + vz) * sa;
    y = v * (ux + vy + wz) + (y * (u * u + w * w) - v * (ux + wz)) * ca + (wx - uz) * sa;
    z = w * (ux + vy + wz) + (z * (u * u + v * v) - w * (ux + vy)) * ca + (-vx + uy) * sa;

    u = sqrt(x * x + y * y + z * z);
    x /= u;
    y /= u;
    z /= u;

    out.x = x * sizeX;
    out.y = y * sizeX;
    out.z = z * sizeX;
}

__device__ void rotatePatch(Patch& ptch, int rx, int ry)
{
    float3 n, y, x;

    // rotate patch around x axis by angle rx
    rotPointAroundVect(n, ptch.n, ptch.x, rx);
    rotPointAroundVect(y, ptch.y, ptch.x, rx);
    ptch.n = n;
    ptch.y = y;

    // rotate new patch around y axis by angle ry
    rotPointAroundVect(n, ptch.n, ptch.y, ry);
    rotPointAroundVect(x, ptch.x, ptch.y, ry);
    ptch.n = n;
    ptch.x = x;
}

__device__ void movePatch(Patch& ptch, int pt)
{
    // float3 v = ptch.p-rC;
    // normalize(v);
    float3 v = ptch.n;

    float d = ptch.d * (float)pt;
    float3 p = ptch.p + v * d;
    ptch.p = p;
}

__device__ void computeRotCS(float3& xax, float3& yax, float3& n)
{
    xax.x = -n.y + n.z; // get any cross product
    xax.y = +n.x + n.z;
    xax.z = -n.x - n.y;
    if(fabs(xax.x) < 0.0000001f && fabs(xax.y) < 0.0000001f && fabs(xax.z) < 0.0000001f)
    {
        xax.x = -n.y - n.z; // get any cross product (complementar)
        xax.y = +n.x - n.z;
        xax.z = +n.x + n.y;
    };
    normalize(xax);
    yax = cross(n, xax);
}

} // namespace depthMap
} // namespace aliceVision
