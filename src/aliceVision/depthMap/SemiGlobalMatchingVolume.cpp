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

SemiGlobalMatchingVolume::SemiGlobalMatchingVolume(int _volDimX, int _volDimY, int _volDimZ, SemiGlobalMatchingParams* _sp)
    : sp( _sp )
    , volDimX(  _volDimX )
    , volDimY(  _volDimY )
    , volDimZ(  _volDimZ )
    , volStepZ( 1 )
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
        float volumeMB = ( _volDimX * _volDimY * _volDimZ * sizeof(float) ) / ( 1024.0f*1024.0f );
        while(volumeMB > dmi.x)
        {
            volStepZ++;
            volumeMB = ( _volDimX * _volDimY * _volDimZ / volStepZ * sizeof(float) ) / ( 1024.0f*1024.0f );
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
}

SemiGlobalMatchingVolume::~SemiGlobalMatchingVolume()
{
}


void SemiGlobalMatchingVolume::cloneVolumeSecondStepZ(const StaticVector<unsigned char>& volumeSecondBest)
{
    long tall = clock();

    ALICEVISION_LOG_DEBUG("SemiGlobalMatchingVolume::cloneVolumeSecondStepZ, volume reduction by volStepZ: " << volStepZ);

    _volumeStepZ.resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);
    _volumeBestZ.resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);

    const unsigned char* in_volumeSecondBestPtr = volumeSecondBest.getData().data();
    unsigned char* out_volumeStepZPtr     = _volumeStepZ.getDataWritable().data();
    int*           out_volumeBestZPtr     = _volumeBestZ.getDataWritable().data();
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                if((z / volStepZ) < (volDimZ / volStepZ))
                {
                    int offs = (z / volStepZ) * volDimX * volDimY + y * volDimX + x;
                    unsigned char oldSim = out_volumeStepZPtr[offs];
                    unsigned char newSim = in_volumeSecondBestPtr[z * volDimX * volDimY + y * volDimX + x];
                    if(newSim <= oldSim)
                    {
                        out_volumeStepZPtr[offs] = newSim;
                        out_volumeBestZPtr[offs] = z;
                    }
                }
            }
        }
    }

    if (sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::cloneVolumeSecondStepZ ");
}

void SemiGlobalMatchingVolume::getOrigVolumeBestIdValFromVolumeStepZ(StaticVector<IdValue>& out_volumeBestIdVal, int zborder)
{
    ALICEVISION_LOG_DEBUG("SemiGlobalMatchingVolume::getOrigVolumeBestIdValFromVolumeStepZ");
    long tall = clock();

    out_volumeBestIdVal.resize_with(volDimX * volDimY, IdValue(-1, 1.0f));
    unsigned char* _volumeStepZPtr = _volumeStepZ.getDataWritable().data();
    int* _volumeBestZPtr = _volumeBestZ.getDataWritable().data();
    IdValue* out_volumeBestIdValPtr = out_volumeBestIdVal.getDataWritable().data();
    for(int z = zborder; z < (volDimZ / volStepZ) - zborder; z++)
    {
        for(int y = 1; y < volDimY - 1; y++)
        {
            for(int x = 1; x < volDimX - 1; x++)
            {
                int volumeIndex = z * volDimX * volDimY + y * volDimX + x;
                // value from volumeStepZ converted from (0, 255) to (-1, +1)
                float val = (((float)_volumeStepZPtr[volumeIndex]) / 255.0f) * 2.0f - 1.0f;
                int bestZ = _volumeBestZPtr[volumeIndex];
                IdValue& out_idVal = out_volumeBestIdValPtr[y * volDimX + x];
                assert(bestZ >= 0);

                if(out_idVal.id == -1)
                {
                    // if not initialized, set the value
                    out_idVal.value = val;
                    out_idVal.id = bestZ;
                }
                else if (val < out_idVal.value)
                {
                    // if already initialized, update the value if smaller
                    out_idVal.value = val;
                    out_idVal.id = bestZ;
                }
            }
        }
    }

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::getOrigVolumeBestIdValFromVolumeStepZ ");
}

} // namespace depthMap
} // namespace aliceVision
