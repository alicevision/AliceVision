// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Point3d.hpp>

namespace aliceVision {

struct Stat3d
{
    double xsum = 0.0;
    double ysum = 0.0;
    double zsum = 0.0;
    double xxsum = 0.0;
    double yysum = 0.0;
    double zzsum = 0.0;
    double xysum = 0.0;
    double xzsum = 0.0;
    double yzsum = 0.0;
    int count = 0;

    double A[3][3], V[3][3], d[3], X[4], vx[3], vy[3], vz[3];

    Stat3d() {}

    void update(Point3d* p)
    {
        xxsum += (double)p->x * (double)p->x;
        yysum += (double)p->y * (double)p->y;
        zzsum += (double)p->z * (double)p->z;
        xysum += (double)p->x * (double)p->y;
        xzsum += (double)p->x * (double)p->z;
        yzsum += (double)p->y * (double)p->z;
        xsum += (double)p->x;
        ysum += (double)p->y;
        zsum += (double)p->z;
        count += 1;
    }

    void add(Stat3d* p)
    {
        xxsum += p->xxsum;
        yysum += p->yysum;
        zzsum += p->zzsum;
        xysum += p->xysum;
        xzsum += p->xzsum;
        yzsum += p->yzsum;
        xsum += p->xsum;
        ysum += p->ysum;
        zsum += p->zsum;
        count += p->count;
    }

    void getEigenVectorsDesc(Point3d& cg, Point3d& v1, Point3d& v2, Point3d& v3, float& d1, float& d2, float& d3)
    {
        float xmean = xsum / (double)count;
        float ymean = ysum / (double)count;
        float zmean = zsum / (double)count;

        A[0][0] = (xxsum - xsum * xmean - xsum * xmean + xmean * xmean * (double)count) / (double)(count);
        A[0][1] = (xysum - ysum * xmean - xsum * ymean + xmean * ymean * (double)count) / (double)(count);
        A[0][2] = (xzsum - zsum * xmean - xsum * zmean + xmean * zmean * (double)count) / (double)(count);
        A[1][0] = (xysum - xsum * ymean - ysum * xmean + ymean * xmean * (double)count) / (double)(count);
        A[1][1] = (yysum - ysum * ymean - ysum * ymean + ymean * ymean * (double)count) / (double)(count);
        A[1][2] = (yzsum - zsum * ymean - ysum * zmean + ymean * zmean * (double)count) / (double)(count);
        A[2][0] = (xzsum - xsum * zmean - zsum * xmean + zmean * xmean * (double)count) / (double)(count);
        A[2][1] = (yzsum - ysum * zmean - zsum * ymean + zmean * ymean * (double)count) / (double)(count);
        A[2][2] = (zzsum - zsum * zmean - zsum * zmean + zmean * zmean * (double)count) / (double)(count);

        /*
        A[0][0] = xxsum;
        A[0][1] = xysum;
        A[0][2] = xzsum;
        A[1][0] = xysum;
        A[1][1] = yysum;
        A[1][2] = yzsum;
        A[2][0] = xzsum;
        A[2][1] = yzsum;
        A[2][2] = zzsum;
        */

        // should be sorted
        eigen_decomposition(A, V[0], V[1], V[2], d);

        v1 = Point3d((float)V[0][2], (float)V[1][2], (float)V[2][2]).normalize();
        v2 = Point3d((float)V[0][1], (float)V[1][1], (float)V[2][1]).normalize();
        v3 = Point3d((float)V[0][0], (float)V[1][0], (float)V[2][0]).normalize();

        cg.x = (float)(xsum / count);
        cg.y = (float)(ysum / count);
        cg.z = (float)(zsum / count);

        d1 = (float)d[2];
        d2 = (float)d[1];
        d3 = (float)d[0];
    }

private:
    static void eigen_decomposition(double A[3][3], double V0[], double V1[], double V2[], double d[]);
};

} // namespace aliceVision
