// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRcTc.hpp"
#include <aliceVision/mvsUtils/common.hpp>

namespace aliceVision {
namespace depthMap {

SemiGlobalMatchingRcTc::SemiGlobalMatchingRcTc(
                         const std::vector<float>& _rcTcDepths,
                         int _rc,
                         int _tc,
                         int scale,
                         int step,
                         SemiGlobalMatchingParams* _sp,
                         StaticVectorBool* _rcSilhoueteMap)
    : rcTcDepths(_rcTcDepths)
    , sp( _sp )
    , rc( _rc )
    , _scale( scale )
    , _step( step )
    , _w( sp->mp->getWidth(rc) / (scale * step) )
    , _h( sp->mp->getHeight(rc) / (scale * step) )
{
    tc = _tc;
    epipShift = 0.0f;

    rcSilhoueteMap = _rcSilhoueteMap;
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

StaticVector<Voxel>* SemiGlobalMatchingRcTc::getPixels()
{
    StaticVector<Voxel>* pixels = new StaticVector<Voxel>();

    pixels->reserve(_w * _h);

    for(int y = 0; y < _h; y++)
    {
        for(int x = 0; x < _w; x++)
        {
            if(rcSilhoueteMap == nullptr)
            {
                pixels->push_back(Voxel(x * _step, y * _step, 0));
            }
            else
            {
                bool isBackgroundPixel = (*rcSilhoueteMap)[y * _w + x];
                if(!isBackgroundPixel)
                {
                    pixels->push_back(Voxel(x * _step, y * _step, 0));
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

    int volStepXY = _step;
    int volDimX = _w;
    int volDimY = _h;
    int volDimZ = rcTcDepths.size();

    StaticVector<unsigned char>* volume = new StaticVector<unsigned char>();
    volume->reserve(volDimX * volDimY * volDimZ);
    volume->resize_with(volDimX * volDimY * volDimZ, 255);

    StaticVector<int>* tcams = new StaticVector<int>();
    tcams->push_back(tc);

    StaticVector<Voxel>* pixels = getPixels();

    volumeMBinGPUMem =
        sp->cps.sweepPixelsToVolume(rcTcDepths.size(), volume, volDimX, volDimY, volDimZ, volStepXY, 0, 0, 0,
                                     &rcTcDepths, rc, wsh, gammaC, gammaP, pixels, _scale, 1, tcams, 0.0f);
    delete pixels;

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
