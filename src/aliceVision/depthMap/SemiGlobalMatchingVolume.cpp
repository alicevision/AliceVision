// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingVolume.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsUtils/common.hpp>

namespace aliceVision {
namespace depthMap {

SemiGlobalMatchingVolume::SemiGlobalMatchingVolume(int _volDimX, int _volDimY, int _volDimZ, int zDimsAtATime, SemiGlobalMatchingParams* _sp)
    : sp( _sp )
    , volDimX(  _volDimX )
    , volDimY(  _volDimY )
    , volDimZ(  _volDimZ )
    , volStepZ( 1 )
    , _volume( nullptr )
    , _volumeSecondBest( nullptr )
    , _volumeStepZ( nullptr )
    , _volumeBestZ( nullptr )
{
    {
        Point3d dmi = sp->cps.getDeviceMemoryInfo();

        if(sp->mp->verbose)
        {
            int devid;
            cudaGetDevice( &devid );
            ALICEVISION_LOG_DEBUG( "In " << __FUNCTION__
                << ": GPU memory on device " << devid
                << ": free: " << dmi.x << ", total: " << dmi.y << ", used: " << dmi.z);
        }
        float volumeMB = ( _volDimX * _volDimY * zDimsAtATime * sizeof(float) ) / ( 1024.0f*1024.0f );
        while(volumeMB > dmi.x)
        {
            volStepZ++;
            volumeMB = ( _volDimX * _volDimY * zDimsAtATime/volStepZ * sizeof(float) ) / ( 1024.0f*1024.0f );
        }
        if(sp->mp->verbose)
            ALICEVISION_LOG_DEBUG("GPU memory volume: " <<  volumeMB );

        if(volStepZ > 1)
        {
            if(sp->mp->verbose)
            {
                int devid;
                cudaGetDevice( &devid );
                ALICEVISION_LOG_WARNING("GPU memory on device " << devid << ": free: " << dmi.x << ", total: " << dmi.y << ", used: " << dmi.z << std::endl << "    Low GPU memory volume step Z: " << volStepZ);
            }
        }
    }

    _volume = new StaticVector<unsigned char>( volDimX * volDimY * volDimZ, 255 );

    _volumeSecondBest = new StaticVector<unsigned char>( volDimX * volDimY * volDimZ, 255 );

    _volumeStepZ = new StaticVector<unsigned char>( volDimX * volDimY * (volDimZ / volStepZ), 255 );

    _volumeBestZ = new StaticVector<int>( volDimX * volDimY * (volDimZ / volStepZ), -1 );
}

SemiGlobalMatchingVolume::~SemiGlobalMatchingVolume()
{
    freeMem();
}

void SemiGlobalMatchingVolume::freeMem()
{
    if( _volume )           delete _volume;
    if( _volumeSecondBest ) delete _volumeSecondBest;
    if( _volumeStepZ )      delete _volumeStepZ;
    if( _volumeBestZ )      delete _volumeBestZ;
    _volume           = nullptr;
    _volumeSecondBest = nullptr;
    _volumeStepZ      = nullptr;
    _volumeBestZ      = nullptr;
}

void SemiGlobalMatchingVolume::cloneVolumeSecondStepZ()
{
    long tall = clock();

    _volumeStepZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);
    _volumeBestZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
    unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ->getDataWritable().data();
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                if((z / volStepZ) < (volDimZ / volStepZ))
                {
                    int offs = (z / volStepZ) * volDimX * volDimY + y * volDimX + x;
                    unsigned char oldSim = _volumeStepZPtr[offs];
                    unsigned char newSim = _volumeSecondBestPtr[z * volDimX * volDimY + y * volDimX + x];
                    if(newSim <= oldSim)
                    {
                        _volumeStepZPtr[offs] = newSim;
                        _volumeBestZPtr[offs] = z;
                    }
                }
            }
        }
    }

    if (sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::cloneVolumeSecondStepZ ");
}

/**
 * @param[in] volStepXY step in the image space
 */
void SemiGlobalMatchingVolume::SGMoptimizeVolumeStepZ(int rc, int volStepXY, int scale)
{
    long tall = clock();

    sp->cps.SGMoptimizeSimVolume(rc, _volumeStepZ, volDimX, volDimY, volDimZ / volStepZ, volStepXY,
                                  scale, sp->P1, sp->P2);

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::SGMoptimizeVolumeStepZ");
}

StaticVector<IdValue>* SemiGlobalMatchingVolume::getOrigVolumeBestIdValFromVolumeStepZ(int zborder)
{
    long tall = clock();

    StaticVector<IdValue>* volumeBestIdVal = new StaticVector<IdValue>();
    volumeBestIdVal->reserve(volDimX * volDimY);
    volumeBestIdVal->resize_with(volDimX * volDimY, IdValue(-1, 1.0f));
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ->getDataWritable().data();
    IdValue* volumeBestIdValPtr = volumeBestIdVal->getDataWritable().data();
    for(int z = zborder; z < volDimZ / volStepZ - zborder; z++)
    {
        for(int y = 1; y < volDimY - 1; y++)
        {
            for(int x = 1; x < volDimX - 1; x++)
            {
                int volumeIndex = z * volDimX * volDimY + y * volDimX + x;
                // value from volumeStepZ converted from (0, 255) to (-1, +1)
                float val = (((float)_volumeStepZPtr[volumeIndex]) / 255.0f) * 2.0f - 1.0f;
                int bestZ = _volumeBestZPtr[volumeIndex]; // TODO: what is bestZ?
                IdValue& idVal = volumeBestIdValPtr[y * volDimX + x];
                assert(bestZ >= 0);

                if(idVal.id == -1)
                {
                    // if not initialized, set the value
                    idVal.value = val;
                    idVal.id = bestZ;
                }
                else if (val < idVal.value)
                {
                    // if already initialized, update the value if smaller
                    idVal.value = val;
                    idVal.id = bestZ;
                }
            }
        }
    }

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::getOrigVolumeBestIdValFromVolumeStepZ ");

    return volumeBestIdVal;
}

void SemiGlobalMatchingVolume::copyVolume(const StaticVector<unsigned char>& volume, int zFrom, int nZSteps)
{
    unsigned char*       _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr  = volume.getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] =
                    volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
            }
        }
    }
}

// void addVolumeSecondMin(const StaticVector<unsigned char>& volume, int zFrom, int nZSteps);
void SemiGlobalMatchingVolume::addVolumeSecondMin(
            const std::vector<int>& index_set, 
            const std::vector<StaticVector<unsigned char> >& vols,
            StaticVector<Pixel> z )

{
  for( auto i : index_set )
  {
    const StaticVector<unsigned char>& volume = vols[i];
    const int zFrom   = z[i].x;
    const int nZSteps = z[i].y;

    unsigned char* _volumePtr = _volume->getDataWritable().data();
    unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getDataWritable().data();
    const unsigned char* volumePtr = volume.getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                const int vaIdx = z * volDimY * volDimX + y * volDimX + x;
                const int vnIdx = (z - zFrom) * volDimY * volDimX + y * volDimX + x;
                unsigned char& va = _volumePtr[vaIdx];
                unsigned char& va2 = _volumeSecondBestPtr[vaIdx];
                unsigned char vn = volumePtr[vnIdx];
                if(vn < va)
                {
                    va2 = va;
                    va = vn;
                }
                else if (vn < va2)
                {
                    va2 = vn;
                }
            }
        }
    }
  }
}

} // namespace depthMap
} // namespace aliceVision
