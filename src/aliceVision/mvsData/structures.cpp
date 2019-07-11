// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "structures.hpp"
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Voxel.hpp>

namespace aliceVision {

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

int qsortCompareSortedIdDesc(const void* ia, const void* ib)
{
    SortedId a = *(SortedId*)ia;
    SortedId b = *(SortedId*)ib;
    if(a.value > b.value)
    {
        return -1;
    }

    return 1;
}

int qsortCompareSortedIdAsc(const void* ia, const void* ib)
{
    SortedId a = *(SortedId*)ia;
    SortedId b = *(SortedId*)ib;
    if(a.value < b.value)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByXAsc(const void* ia, const void* ib)
{
    Voxel a = *(Voxel*)ia;
    Voxel b = *(Voxel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByYAsc(const void* ia, const void* ib)
{
    Voxel a = *(Voxel*)ia;
    Voxel b = *(Voxel*)ib;
    if(a.y < b.y)
    {
        return -1;
    }

    return 1;
}

int qSortCompareVoxelByZAsc(const void* ia, const void* ib)
{
    Voxel a = *(Voxel*)ia;
    Voxel b = *(Voxel*)ib;
    if(a.z < b.z)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXDesc(const void* ia, const void* ib)
{
    Pixel a = *(Pixel*)ia;
    Pixel b = *(Pixel*)ib;
    if(a.x > b.x)
    {
        return -1;
    }

    return 1;
}

int qSortComparePixelByXAsc(const void* ia, const void* ib)
{
    Pixel a = *(Pixel*)ia;
    Pixel b = *(Pixel*)ib;
    if(a.x < b.x)
    {
        return -1;
    }

    return 1;
}

int indexOfSortedVoxelArrByX(int val, StaticVector<Voxel>& values, int startId, int stopId)
{
    int lef = startId;
    int rig = stopId;
    int mid = lef + (rig - lef) / 2;
    while((rig - lef) > 1)
    {
        if((val >= values[lef].x) && (val < values[mid].x))
        {
            //lef = lef;
            rig = mid;
            mid = lef + (rig - lef) / 2;
        }
        if((val >= values[mid].x) && (val <= values[rig].x))
        {
            lef = mid;
            //rig = rig;
            mid = lef + (rig - lef) / 2;
        }
        if((val < values[lef].x) || (val > values[rig].x))
        {
            lef = 0;
            rig = 0;
            mid = 0;
        }
    }

    int id = -1;
    if(val == values[lef].x)
    {
        id = lef;
    }
    if(val == values[rig].x)
    {
        id = rig;
    }

    return id;
}

} // namespace aliceVision
