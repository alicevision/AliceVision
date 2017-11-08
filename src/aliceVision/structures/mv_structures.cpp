// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mv_structures.hpp"

#include <stdexcept>

point2d operator*(const matrix2x3& M, const point3d& _p)
{
    point2d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23;
    return p;
}

matrix2x3 DPIP(const matrix3x4& M, const point3d& P)
{
    float M3P = M.m31 * P.x + M.m32 * P.y + M.m33 * P.z + M.m34;
    float M3P2 = M3P * M3P;

    matrix2x3 o;
    o.m11 = ((M.m11 * M.m32 - M.m31 * M.m12) * P.y + (M.m11 * M.m33 - M.m31 * M.m13) * P.z +
             (M.m11 * M.m34 - M.m31 * M.m14)) /
            M3P2;
    o.m12 = ((M.m12 * M.m31 - M.m32 * M.m11) * P.x + (M.m12 * M.m33 - M.m32 * M.m13) * P.z +
             (M.m12 * M.m34 - M.m32 * M.m14)) /
            M3P2;
    o.m13 = ((M.m13 * M.m31 - M.m33 * M.m11) * P.x + (M.m13 * M.m32 - M.m33 * M.m12) * P.y +
             (M.m13 * M.m34 - M.m33 * M.m14)) /
            M3P2;

    o.m21 = ((M.m21 * M.m32 - M.m31 * M.m22) * P.y + (M.m21 * M.m33 - M.m31 * M.m23) * P.z +
             (M.m21 * M.m34 - M.m31 * M.m24)) /
            M3P2;
    o.m22 = ((M.m22 * M.m31 - M.m32 * M.m21) * P.x + (M.m22 * M.m33 - M.m32 * M.m23) * P.z +
             (M.m22 * M.m34 - M.m32 * M.m24)) /
            M3P2;
    o.m23 = ((M.m23 * M.m31 - M.m33 * M.m21) * P.x + (M.m23 * M.m32 - M.m33 * M.m22) * P.y +
             (M.m23 * M.m34 - M.m33 * M.m24)) /
            M3P2;

    return o;
}

point3d operator*(const matrix3x3& M, const point2d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33;
    return p;

    /* //result = a*b
    result.v.x = a.v.x*b.m11 + a.v.y*b.m21 + a.v.z*b.m31;
    result.v.y = a.v.x*b.m12 + a.v.y*b.m22 + a.v.z*b.m32;
    result.v.z = a.v.x*b.m13 + a.v.y*b.m23 + a.v.z*b.m33;//
    */
    /*
    __declspec(align(16)) float a[4]; a[0] = _p.x; a[1] = _p.y; a[2] = 1.0; a[3] = 1.0;
    __declspec(align(16)) float b[12];
    b[0]  = M.m11;
    b[1]  = M.m12;
    b[2]  = M.m13;
    b[3]  = 0.0;
    b[4]  = M.m21;
    b[5]  = M.m22;
    b[6]  = M.m23;
    b[7]  = 0.0;
    b[8]  = M.m31;
    b[9]  = M.m32;
    b[10] = M.m33;
    b[11] = 0.0;

    __declspec(align(16)) float p[4];

    __asm {
            mov edi,a
            mov esi,b
            movss xmm0,[edi+0] //x
            shufps xmm0,xmm0,0 //a.v.x to all floats
            movss xmm2,[edi+4] //y
            mulps xmm0,[esi+0] //row[0]*x
            shufps xmm2,xmm2,0 //a.v.y to all floats
            movss xmm1,[edi+8] //z
            mulps xmm2,[esi+16]//row[1]*y
            shufps xmm1,xmm1,0 //a.v.z to all floats
            addps xmm0,xmm2
            mulps xmm1,[esi+32] //row[2]*z
            mov edi,p
            addps xmm0,xmm1
            movaps [edi],xmm0
    };

    point3d vc;
    vc.x = p[0];
    vc.y = p[1];
    vc.z = p[2];

    return vc;
    */
}

point2d operator^(const matrix3x3& M, const point2d& _p)
{
    point3d p = M * _p;
    point2d pix = point2d(p.x / p.z, p.y / p.z);
    return pix;
}

matrix3x4 operator|(const matrix3x3& M, const point3d& p)
{
    matrix3x4 m;
    m.m11 = M.m11;
    m.m12 = M.m12;
    m.m13 = M.m13;
    m.m14 = p.x;
    m.m21 = M.m21;
    m.m22 = M.m22;
    m.m23 = M.m23;
    m.m24 = p.y;
    m.m31 = M.m31;
    m.m32 = M.m32;
    m.m33 = M.m33;
    m.m34 = p.z;

    return m;
}

matrix3x3 outerMultiply(const point3d& a, const point3d& b)
{
    matrix3x3 m;
    m.m11 = a.x * b.x;
    m.m12 = a.x * b.y;
    m.m13 = a.x * b.z;
    m.m21 = a.y * b.x;
    m.m22 = a.y * b.y;
    m.m23 = a.y * b.z;
    m.m31 = a.z * b.x;
    m.m32 = a.z * b.y;
    m.m33 = a.z * b.z;

    return m;
}

point3d operator*(const matrix4x4& M, const point3d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z + M.m14;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z + M.m24;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z + M.m34;
    float w = M.m41 * _p.x + M.m42 * _p.y + M.m43 * _p.z + M.m44;

    p = p / w;

    return p;
}

point3d operator*(const matrix3x3& M, const point3d& _p)
{
    point3d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z;
    p.y = M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z;
    p.z = M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z;
    return p;
}

point3d operator*(const matrix3x4& M, const point3d& _p)
{
    return point3d(M.m11 * _p.x + M.m12 * _p.y + M.m13 * _p.z + M.m14,
                   M.m21 * _p.x + M.m22 * _p.y + M.m23 * _p.z + M.m24,
                   M.m31 * _p.x + M.m32 * _p.y + M.m33 * _p.z + M.m34);
}

point3d mldivide(const matrix3x3& M, const point3d& b)
{
    if(!M.isSingular())
    {
        return M.inverse() * b;
    }

    M.doprintf();
    throw std::runtime_error("Matrix is singular.");
}

point2d operator*(const matrix2x2& M, const point2d& _p)
{
    point2d p;
    p.x = M.m11 * _p.x + M.m12 * _p.y;
    p.y = M.m21 * _p.x + M.m22 * _p.y;
    return p;
}

point2d mldivide(matrix2x2& M, point2d& b)
{
    if(!M.isSingular())
    {
        return M.inverse() * b;
    }

    M.doprintf();
    throw std::runtime_error("Matrix is singular.");
}

int qSortCompareFloatAsc(const void* ia, const void* ib)
{
    float a = *(float*)ia;
    float b = *(float*)ib;
    if(a > b)
    {
        return 1;
    }

    return -1;
}

int qSortCompareIntAsc(const void* ia, const void* ib)
{
    int a = *(int*)ia;
    int b = *(int*)ib;
    if(a > b)
    {
        return 1;
    }

    return -1;
}

int qSortCompareIntDesc(const void* ia, const void* ib)
{
    int a = *(int*)ia;
    int b = *(int*)ib;
    if(a < b)
    {
        return 1;
    }

    return -1;
}

int compareSortedId(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value > b.value)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdDesc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value > b.value)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdAsc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.value < b.value)
    {
        return -1;
    }

    return 1;
}
int qSortCompareVoxelByZDesc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.z > b.z)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByXAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByYAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.y < b.y)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByZAsc(const void* ia, const void* ib)
{
    voxel a = *(voxel*)ia;
    voxel b = *(voxel*)ib;
    if(a.z < b.z)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXDesc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.x > b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXAsc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByYDesc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.y > b.y)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByYAsc(const void* ia, const void* ib)
{
    pixel a = *(pixel*)ia;
    pixel b = *(pixel*)ib;
    if(a.y < b.y)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdByIdAsc(const void* ia, const void* ib)
{
    sortedId a = *(sortedId*)ia;
    sortedId b = *(sortedId*)ib;
    if(a.id < b.id)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByXAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByYAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePoint3dByZAsc(const void* ia, const void* ib)
{
    point3d a = *(point3d*)ia;
    point3d b = *(point3d*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

mv_bites_array::mv_bites_array(int _nbits)
{
    sizeofbits = sizeof(unsigned int) * 8;
    nbits = _nbits;
    allocated = nbits / sizeofbits + 1;
    bits = new unsigned int[allocated];
    clear();
}

mv_bites_array::~mv_bites_array()
{
    delete[] bits;
}

void mv_bites_array::clear()
{
    for(int i = 0; i < allocated; i++)
    {
        bits[i] = 0;
    }
}

bool mv_bites_array::getbit(int bitid)
{
    int i = bitid / sizeofbits;
    int j = bitid % sizeofbits;
    return (bits[i] & (1 << j)) != 0; /* 0 or 1   */
}

void mv_bites_array::setbit(int bitid, bool value)
{
    int i = bitid / sizeofbits;
    int j = bitid % sizeofbits;
    // printf("%i %i %i %i\n",bitid,i,j,sizeofbits);
    if(value)
        bits[i] |= (1 << j); /* set bit  */
    else
        bits[i] &= ~(1 << j); /* clear bit*/
}

void mv_bites_array::ORBits(mv_bites_array* e)
{
    for(int i = 0; i < std::min(allocated, e->allocated); i++)
    {
        bits[i] |= e->bits[i];
    }
}

void mv_bites_array::ANDBits(mv_bites_array* e)
{
    for(int i = 0; i < std::min(allocated, e->allocated); i++)
    {
        bits[i] &= e->bits[i];
    }
}

int mv_bites_array::gentNSumBitsOfANDBits(mv_bites_array* e)
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        out += (int)(getbit(i) && e->getbit(i));
    }
    return out;
}

int mv_bites_array::getNSumBits()
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        out += (int)getbit(i);
    }
    return out;
}

void mv_bites_array::printfBits()
{
    int out = 0;
    for(int i = 0; i < nbits; i++)
    {
        bool b = getbit(i);
        printf("%i", (int)b);
        out += (int)b;
    }
    printf("\n n 1 bits : %i\n", out);
}

int getANDBits(mv_bites_array* a1, mv_bites_array* a2)
{
    mv_bites_array* a3 = new mv_bites_array(a1->nbits);
    a3 = a1;
    a3->ANDBits(a2);
    int out = a3->getNSumBits();
    delete a3;
    return out;
}

rgb rgb_random()
{
    rgb c;

    c.r = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
    c.g = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);
    c.b = (uchar)(((float)rand() / (float)RAND_MAX) * 256.0);

    return c;
}

int indexOfSortedVoxelArrByX(int val, staticVector<voxel>* values, int startId, int stopId)
{
    int lef = startId;
    int rig = stopId;
    int mid = lef + (rig - lef) / 2;
    while((rig - lef) > 1)
    {
        if((val >= (*values)[lef].x) && (val < (*values)[mid].x))
        {
            lef = lef;
            rig = mid;
            mid = lef + (rig - lef) / 2;
        }
        if((val >= (*values)[mid].x) && (val <= (*values)[rig].x))
        {
            lef = mid;
            rig = rig;
            mid = lef + (rig - lef) / 2;
        }
        if((val < (*values)[lef].x) || (val > (*values)[rig].x))
        {
            lef = 0;
            rig = 0;
            mid = 0;
        }
    }

    int id = -1;
    if(val == (*values)[lef].x)
    {
        id = lef;
    }
    if(val == (*values)[rig].x)
    {
        id = rig;
    }

    return id;
}

