#include "ps_sgm_rctc.h"
#include "stdafx.h"

ps_sgm_rctc::ps_sgm_rctc(staticVector<float>* _rcTcDepths, int _rc, int _tc, int _scale, int _step, ps_sgm_params* _sp,
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

ps_sgm_rctc::~ps_sgm_rctc()
{
    //
}

staticVector<voxel>* ps_sgm_rctc::getPixels()
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

staticVector<unsigned char>* ps_sgm_rctc::computeDepthSimMapVolume(float& volumeMBinGPUMem, int wsh, float gammaC,
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
        printfElapsedTime(tall, "ps_sgm_rctc::computeDepthSimMapVolume ");

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
