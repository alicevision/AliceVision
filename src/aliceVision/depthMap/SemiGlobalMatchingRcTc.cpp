// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingRcTc.hpp"
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/depthMap/cuda/tcinfo.hpp>
#include <algorithm>

namespace aliceVision {
namespace depthMap {

SemiGlobalMatchingRcTc::SemiGlobalMatchingRcTc(
            const std::vector<int>& index_set,
            const std::vector<float>& rcTcDepths,
            const std::vector<Pixel>&  rcTcDepthRanges,
            int rc,
            const StaticVector<int>& tc,
            int scale,
            int step,
            int zDimsAtATime,
            SemiGlobalMatchingParams* _sp,
            StaticVectorBool* rcSilhoueteMap)
    : _index_set( index_set )
    , sp( _sp )
    , _rc( rc )
    , _tc( tc )
    , _scale( scale )
    , _step( step )
    , _w( sp->mp->getWidth(rc) / (scale * step) )
    , _h( sp->mp->getHeight(rc) / (scale * step) )
    , _rcTcDepths( rcTcDepths )
    , _rcTcDepthRanges( rcTcDepthRanges )
    , _zDimsAtATime( zDimsAtATime )
{
    _rcSilhoueteMap = rcSilhoueteMap;
}

SemiGlobalMatchingRcTc::~SemiGlobalMatchingRcTc()
{
    //
}

struct MinOffX
{
    bool operator()( const Pixel& l, const Pixel& r ) const
    {
        return ( l.x < r.x );
    }
};

struct MinOffXplusY
{
    bool operator()( const Pixel& l, const Pixel& r ) const
    {
        return ( l.x+l.y < r.x+r.y );
    }
};

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

    MinOffX      Pixel_x_comp;
    MinOffXplusY Pixel_xy_comp;
    const int startingDepth = std::min_element( _rcTcDepthRanges.begin(),
                                                _rcTcDepthRanges.end(),
                                                Pixel_x_comp )->x;
    auto depth_it = std::max_element( _rcTcDepthRanges.begin(),
                                      _rcTcDepthRanges.end(),
                                      Pixel_xy_comp );
    const int stoppingDepth = depth_it->x + depth_it->y;
    
#if 1
    const int maxDimZ = stoppingDepth - startingDepth;

#else
    int       maxDimZ   = 0; // *_index_set.begin();
    for( auto j : _index_set )
    {
        const int volDimZ = _rcTcDepthRanges[j].y;

        volume[j].resize( volDimX * volDimY * volDimZ );

        maxDimZ = std::max( maxDimZ, volDimZ );
    }
#endif
    
    // volume size for 1 depth layer for 1 tcam
    const int    depth_layer_size  = volDimX * volDimY;

    // volume size for 1 tcam
    const int    volume_offset     = volDimX * volDimY * maxDimZ;

    // volume size for all tcams
    const size_t volume_num_floats = volDimX * volDimY * maxDimZ * _index_set.size();

    for( auto j : _index_set )
    {
        volume[j].resize( volume_offset );
    }

    float* volume_buf;
    bool   volume_buf_pinned = true;
    cudaError_t err = cudaMallocHost( &volume_buf, volume_num_floats * sizeof(float) );
    if( err != cudaSuccess )
    {
        ALICEVISION_LOG_WARNING( "Failed to allocate "
            << volume_num_floats * sizeof(float)
            << " bytes of CUDA host (pinned) memory, " << cudaGetErrorString(err)
            << std::endl
            << "Allocating slower unpinned memory instead." );
        volume_buf = new float[ volume_num_floats ];
        volume_buf_pinned = false;
    }

    // Initialize to a big value
    // move to GPU eventually
    std::fill_n( volume_buf, volume_num_floats, 255.0f );

#warning change sweep to record depth data at _rcTcDepthRanges[j].x-startingDepth instead of 0
    std::vector<OneTC> tcs;
    for( auto j : _index_set )
    {
        tcs.push_back( OneTC( j,
                              _tc[j],
                              _rcTcDepthRanges[j].x,
                              _rcTcDepthRanges[j].y ) );
    }

    for( int ct=0; ct<tcs.size(); ct++ )
    {
        tcs[ct].setVolumeOut( volume_buf + ct * volume_offset,
                              depth_layer_size,
                              startingDepth );
    }

    sp->cps.sweepPixelsToVolume( volume_tmp_on_gpu,
                                 volDimX, volDimY,
                                 volStepXY,
                                 tcs,
                                 _zDimsAtATime,
                                 _rcTcDepths,
                                 _rc, _tc,
                                 _rcSilhoueteMap,
                                 wsh, gammaC, gammaP, _scale, 1,
                                 0.0f);

    /*
     * TODO: This conversion operation on the host consumes a lot of time,
     *       about 1/3 of the actual computation. Work to avoid it.
     */
    for( int ct=0; ct<tcs.size(); ct++ )
    {
        const int j          = tcs[ct].getTCPerm();
        // const int startLayer = tcs[ct].getDepthToStart() - startingDepth;
        const int volDimZ    = tcs[ct].getDepthsToSearch(); // depths_to_search;
        float*    ptr        = tcs[ct].getVolumeOutWithOffset();

        for( int i=0; i<volDimX * volDimY * volDimZ; i++ )
        {
            volume[j][i] = (unsigned char)( 255.0f * std::max(std::min(ptr[i],1.0f),0.0f) );
        }
    }

    if( volume_buf_pinned )
        cudaFreeHost( volume_buf );
    else
        delete [] volume_buf;

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingRcTc::computeDepthSimMapVolume ");

    for( auto j : _index_set )
    {
        const int volDimZ = _rcTcDepthRanges[j].y;

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
