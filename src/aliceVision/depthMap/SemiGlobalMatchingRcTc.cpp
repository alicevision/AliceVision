// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRcTc.hpp"
#include <aliceVision/mvsUtils/common.hpp>

namespace aliceVision {
namespace depthMap {

SemiGlobalMatchingRcTc::SemiGlobalMatchingRcTc(StaticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, SemiGlobalMatchingParams* _sp,
                         StaticVectorBool* _rcSilhoueteMap)
{
    sp = _sp;

    rc = _rc;
    tc = _tc;
    scale = _scale;
    step = _step;
    epipShift = 0.0f;

    rcTcDepths = _rcTcDepths;

    w = sp->mp->getWidth(rc) / (scale * step);
    h = sp->mp->getHeight(rc) / (scale * step);

    rcSilhoueteMap = _rcSilhoueteMap;
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

StaticVector<Voxel>* SemiGlobalMatchingRcTc::getPixels()
{
    StaticVector<Voxel>* pixels = new StaticVector<Voxel>();

    pixels->reserve(w * h);

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            if(rcSilhoueteMap == nullptr)
            {
                pixels->push_back(Voxel(x * step, y * step, 0));
            }
            else
            {
                bool isBackgroundPixel = (*rcSilhoueteMap)[y * w + x];
                if(!isBackgroundPixel)
                {
                    pixels->push_back(Voxel(x * step, y * step, 0));
                }
            }
        }
    }
    return pixels;
}

StaticVector<unsigned char>* SemiGlobalMatchingRcTc::computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC,
                                                                   float gammaP)
{
    long tall = clock();

    int volStepXY = step;
    int volDimX = w;
    int volDimY = h;
    int volDimZ = rcTcDepths->size();

    StaticVector<unsigned char>* volume = new StaticVector<unsigned char>();
    volume->reserve(volDimX * volDimY * volDimZ);
    volume->resize_with(volDimX * volDimY * volDimZ, 255);

    StaticVector<int>* volume_tmp = new StaticVector<int>();
    volume_tmp->reserve(volDimX * volDimY * volDimZ);
    volume_tmp->resize_with(volDimX * volDimY * volDimZ, 255.0f);

    StaticVector<int>* tcams = new StaticVector<int>();
    tcams->reserve(1);
    tcams->push_back(tc);

    StaticVector<Voxel>* pixels = getPixels();

    volumeMBinGPUMem =
        sp->cps->sweepPixelsToVolume(rcTcDepths->size(), volume_tmp, volDimX, volDimY, volDimZ, volStepXY, 0, 0, 0,
                                     rcTcDepths, rc, wsh, gammaC, gammaP, pixels, scale, 1, tcams, 0.0f);
    delete pixels;
    delete tcams;

    for( int i=0; i<volDimX * volDimY * volDimZ; i++ )
    {
        (*volume)[i] = (unsigned char)((*volume_tmp)[i]);
    }

    delete volume_tmp;

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingRcTc::computeDepthSimMapVolume ");

    if(sp->P3 > 0)
    {
#pragma omp parallel for
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                (*volume)[(volDimZ - 1) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                (*volume)[(volDimZ - 2) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                (*volume)[(volDimZ - 3) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                (*volume)[(volDimZ - 4) * volDimY * volDimX + y * volDimX + x] = sp->P3;
            }
        }
    }

    return volume;
}

} // namespace depthMap
} // namespace aliceVision
