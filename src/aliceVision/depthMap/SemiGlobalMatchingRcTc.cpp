// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRcTc.hpp"
#include <aliceVision/common/common.hpp>

SemiGlobalMatchingRcTc::SemiGlobalMatchingRcTc(staticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, SemiGlobalMatchingParams* _sp,
                         staticVectorBool* _rcSilhoueteMap)
{
    sp = _sp;

    rc = _rc;
    tc = _tc;
    scale = _scale;
    step = _step;
    epipShift = 0.0f;

    rcTcDepths = _rcTcDepths;

    w = sp->mp->mip->getWidth(rc) / (scale * step);
    h = sp->mp->mip->getHeight(rc) / (scale * step);

    rcSilhoueteMap = _rcSilhoueteMap;
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

staticVector<voxel>* SemiGlobalMatchingRcTc::getPixels()
{
    staticVector<voxel>* pixels = new staticVector<voxel>(w * h);
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            if(rcSilhoueteMap == nullptr)
            {
                pixels->push_back(voxel(x * step, y * step, 0));
            }
            else
            {
                bool isBackgroundPixel = (*rcSilhoueteMap)[y * w + x];
                if(!isBackgroundPixel)
                {
                    pixels->push_back(voxel(x * step, y * step, 0));
                }
            }
        }
    }
    return pixels;
}

staticVector<unsigned char>* SemiGlobalMatchingRcTc::computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC,
                                                                   float gammaP)
{
    long tall = clock();

    int volStepXY = step;
    int volDimX = w;
    int volDimY = h;
    int volDimZ = rcTcDepths->size();

    staticVector<unsigned char>* volume = new staticVector<unsigned char>(volDimX * volDimY * volDimZ);
    volume->resize_with(volDimX * volDimY * volDimZ, 255);

    staticVector<int>* tcams = new staticVector<int>(1);
    tcams->push_back(tc);

    staticVector<voxel>* pixels = getPixels();

    volumeMBinGPUMem =
        sp->cps->sweepPixelsToVolume(rcTcDepths->size(), volume, volDimX, volDimY, volDimZ, volStepXY, 0, 0, 0,
                                     rcTcDepths, rc, wsh, gammaC, gammaP, pixels, scale, 1, tcams, 0.0f);
    delete pixels;
    delete tcams;

    if(sp->mp->verbose)
        printfElapsedTime(tall, "SemiGlobalMatchingRcTc::computeDepthSimMapVolume ");

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
