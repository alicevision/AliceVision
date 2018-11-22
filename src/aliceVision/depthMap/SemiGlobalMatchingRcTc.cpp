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
            const std::vector<int>& index_set,
            const std::vector<std::vector<float> >& _rcTcDepths,
            int _rc,
            const StaticVector<int>& _tc,
            int scale,
            int step,
            int zDimsAtATime,
            SemiGlobalMatchingParams* _sp,
            StaticVectorBool* _rcSilhoueteMap)
    : _index_set( index_set )
    , sp( _sp )
    , rc( _rc )
    , tc( _tc )
    , _scale( scale )
    , _step( step )
    , _w( sp->mp->getWidth(rc) / (scale * step) )
    , _h( sp->mp->getHeight(rc) / (scale * step) )
    , rcTcDepths( _rcTcDepths )
    , _zDimsAtATime( zDimsAtATime )
{
    epipShift = 0.0f;

    rcSilhoueteMap = _rcSilhoueteMap;
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

void SemiGlobalMatchingRcTc::computeDepthSimMapVolume(
        std::vector<StaticVector<unsigned char> >& volume,
        std::vector<CudaDeviceMemoryPitched<float, 3>*>& volume_tmp_on_gpu,
        int wsh,
        float gammaC,
        float gammaP)
{
    const long tall = clock();

    const int volStepXY = _step;
    const int volDimX   = _w;
    const int volDimY   = _h;
    int maxDimZ = *_index_set.begin();

    for( auto j : _index_set )
    {
        const int volDimZ = rcTcDepths[j].size();

        volume[j].resize( volDimX * volDimY * volDimZ );

        maxDimZ = std::max( maxDimZ, volDimZ );
    }

    float* volume_buf;
    bool   volume_buf_pinned = true;
    cudaError_t err = cudaMallocHost( &volume_buf, _index_set.size() * volDimX * volDimY * maxDimZ * sizeof(float) );
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_WARNING( "Failed to allocate " << _index_set.size() * volDimX * volDimY * maxDimZ * sizeof(float) << " bytes of CUDA host (pinned) memory, " << cudaGetErrorString(err) );
        volume_buf = new float[ _index_set.size() * volDimX * volDimY * maxDimZ ];
        volume_buf_pinned = false;
    }


    const int volume_offset = volDimX * volDimY * maxDimZ;

    sp->cps.sweepPixelsToVolume( _index_set,
                                 volume_buf,
                                 volume_offset,
                                 volume_tmp_on_gpu,
                                 volDimX, volDimY,
                                 volStepXY,
                                 _zDimsAtATime,
                                 rcTcDepths,
                                 rc, tc,
                                 rcSilhoueteMap,
                                 wsh, gammaC, gammaP, _scale, 1,
                                 0.0f);

    /*
     * TODO: This conversion operation on the host consumes a lot of time,
     *       about 1/3 of the actual computation. Work to avoid it.
     */
    int ct = 0;
    for( auto j : _index_set )
    {
        const int volDimZ = rcTcDepths[j].size();

        for( int i=0; i<volDimX * volDimY * volDimZ; i++ )
        {
            float* ptr = &volume_buf[ct * volDimX * volDimY * maxDimZ];
            volume[j][i] = (unsigned char)( 255.0f * std::max(std::min(ptr[i],1.0f),0.0f) );
        }
        ct++;
    }

    if( volume_buf_pinned )
        cudaFreeHost( volume_buf );
    else
        delete [] volume_buf;

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingRcTc::computeDepthSimMapVolume ");

    for( auto j : _index_set )
    {
        const int volDimZ = rcTcDepths[j].size();

        if(sp->P3 > 0)
        {
#pragma omp parallel for
            for(int y = 0; y < volDimY; y++)
            {
                for(int x = 0; x < volDimX; x++)
                {
                    volume[j][(volDimZ - 1) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                    volume[j][(volDimZ - 2) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                    volume[j][(volDimZ - 3) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                    volume[j][(volDimZ - 4) * volDimY * volDimX + y * volDimX + x] = sp->P3;
                }
            }
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
