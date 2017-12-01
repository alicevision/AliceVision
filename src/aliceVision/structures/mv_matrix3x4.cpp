// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_matrix3x4.hpp"

#include <aliceVision/structures/mv_structures.hpp>

#include <cstdio>


void matrix3x4::doprintf() const
{
    printf("%f %f %f %f\n", m11, m12, m13, m14);
    printf("%f %f %f %f\n", m21, m22, m23, m24);
    printf("%f %f %f %f\n", m31, m32, m33, m34);
}

void matrix3x4::saveToFile(std::string fileName) const
{
    FILE* f = fopen(fileName.c_str(), "w");
    fprintf(f, "%f %f %f %f\n", m11, m12, m13, m14);
    fprintf(f, "%f %f %f %f\n", m21, m22, m23, m24);
    fprintf(f, "%f %f %f %f", m31, m32, m33, m34);
    fclose(f);
}

void matrix3x4::loadFromFile(std::string fileName)
{
    FILE* f = fopen(fileName.c_str(), "r");
    if(f != nullptr)
    {
        fscanf(f, "%lf %lf %lf %lf\n", &m11, &m12, &m13, &m14);
        fscanf(f, "%lf %lf %lf %lf\n", &m21, &m22, &m23, &m24);
        fscanf(f, "%lf %lf %lf %lf", &m31, &m32, &m33, &m34);
        fclose(f);
    }
}

void matrix3x4::decomposeProjectionMatrix(matrix3x3& K, matrix3x3& R, point3d& C) const
{
    matrix3x3 H = sub3x3();
    H.RQ(K, R);

    // printf("H\n");
    // H.doprintf();
    // printf("K\n");
    // K.doprintf();
    // printf("R\n");
    // R.doprintf();

    bool cam_affine = (K.m33 == 0);

    if(!cam_affine)
    {
        K = K / fabs(K.m33);
    }
    else
    {
        doprintf();
        printf("WARNING affine camera\n");
        exit(1);
    }

    if(K.m11 < 0.0f)
    {
        matrix3x3 D = diag3x3(-1.0f, -1.0f, 1.0f);
        K = K * D;
        R = D * R;
    }

    if(K.m22 < 0.0f)
    {
        matrix3x3 D = diag3x3(1.0f, -1.0f, -1.0f);
        K = K * D;
        R = D * R;
    }

    if(!cam_affine)
    {
        C = mldivide(-sub3x3(), lastColumn());
    }
    else
    {
        doprintf();
        printf("WARNING affine camera\n");
        exit(1);
    }
}
