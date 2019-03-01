// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SemiGlobalMatchingVolume.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

namespace aliceVision {
namespace depthMap {

SemiGlobalMatchingVolume::SemiGlobalMatchingVolume(float _volGpuMB, int _volDimX, int _volDimY, int _volDimZ, SemiGlobalMatchingParams* _sp)
    : sp( _sp )
    , volDimX(  _volDimX )
    , volDimY(  _volDimY )
    , volDimZ(  _volDimZ )
{

    volGpuMB = _volGpuMB;

    {
        Point3d dmi = sp->cps.getDeviceMemoryInfo();
        if(sp->mp->verbose)
            ALICEVISION_LOG_DEBUG("GPU memory : free: " << dmi.x << ", total: " << dmi.y << ", used: " << dmi.z);
        volStepZ = 1;
        float volumeMB = volGpuMB;
        while(4.0f * volumeMB > dmi.x)
        {
            volStepZ++;
            volumeMB = (volGpuMB / (float)volDimZ) * (volDimZ / volStepZ);
        }
        if(sp->mp->verbose)
            ALICEVISION_LOG_DEBUG("GPU memory volume: " <<  (4.0f * volumeMB));

        if(volStepZ > 1)
        {
            if(sp->mp->verbose)
                ALICEVISION_LOG_WARNING("Low GPU memory volume step Z: " << volStepZ);
        }
    }

    _volume = new StaticVector<unsigned char>( volDimX * volDimY * volDimZ, 255 );

    _volumeSecondBest = new StaticVector<unsigned char>( volDimX * volDimY * volDimZ, 255 );

    _volumeStepZ = new StaticVector<unsigned char>( volDimX * volDimY * (volDimZ / volStepZ), 255 );

    _volumeBestZ = new StaticVector<int>( volDimX * volDimY * (volDimZ / volStepZ), -1 );
}

SemiGlobalMatchingVolume::~SemiGlobalMatchingVolume()
{
    delete _volume;
    delete _volumeSecondBest;
    delete _volumeStepZ;
    delete _volumeBestZ;
}

/**
 * @brief Reduction of the similarity volume on the Z axis.
 *        (X, Y, Z) volume is reduced to (X, Y, Z/step).
 *        Inside each chunk of 'step' values, we keep the best similarity value
 *        in 'volumeStepZ' and store the original Z index in 'volumeBestZ'.
 */
void SemiGlobalMatchingVolume::cloneVolumeStepZ()
{
    long tall = clock();

    //TODO: use a local variable for _volumeStepZ (instead of a member) and replace _volume
    _volumeStepZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), 255);
    _volumeBestZ->resize_with(volDimX * volDimY * (volDimZ / volStepZ), -1);
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    unsigned char* _volumeStepZPtr = _volumeStepZ->getDataWritable().data();
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
                    unsigned char newSim = _volumePtr[z * volDimX * volDimY + y * volDimX + x];
                    if(newSim <= oldSim)
                    {
                        _volumeStepZPtr[offs] = newSim;
                        _volumeBestZPtr[offs] = z;
                    }
                }
            }
        }
    }

    delete _volume;
    _volume = nullptr;

    if(sp->mp->verbose)
        mvsUtils::printfElapsedTime(tall, "SemiGlobalMatchingVolume::cloneVolumeStepZ ");
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

void SemiGlobalMatchingVolume::exportVolume(StaticVector<float>& depths, int camIndex,  int scale, int step, const std::string& filepath) const
{
    sfmData::SfMData pointCloud;
    const unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getData().data();
    const mvsUtils::MultiViewParams* mp = sp->mp;
    int eStep = 10;

    IndexT landmarkId;
    for(int z = 0; z < volDimZ; ++z)
    {
        for(int y = 0; y < volDimY; y+=eStep)
        {
            for(int x = 0; x < volDimX; x+=eStep)
            {
                const double planeDepth = depths[z];
                const Point3d planen = (mp->iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp->CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp->iCamArr[camIndex] * Point2d(x * scale * step, y * scale * step)).normalize();
                const Point3d p = linePlaneIntersect(mp->CArr[camIndex], v, planep, planen);

                const int index = z * volDimX * volDimY + y * volDimX + x;
                const int maxValue = 80;
                if(_volumeSecondBestPtr[index] > maxValue)
                  continue;
                const rgb c = getRGBFromJetColorMap(static_cast<double>(_volumeSecondBestPtr[index]) / double(maxValue));
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

void SemiGlobalMatchingVolume::exportVolumeStep(StaticVector<float>& depths, int camIndex,  int scale, int step, const std::string& filepath) const
{
    sfmData::SfMData pointCloud;
    const unsigned char* volumePtr = _volumeStepZ->getData().data();
    const mvsUtils::MultiViewParams* mp = sp->mp;
    int eStep = 10;

    IndexT landmarkId;
    for(int stepZ = 0; stepZ < (volDimZ / volStepZ); ++stepZ)
    {
        for(int y = 0; y < volDimY; y+=eStep)
        {
            for(int x = 0; x < volDimX; x+=eStep)
            {
                const int z = (*_volumeBestZ)[stepZ * volDimX * volDimY + y * volDimX + x];
                const double planeDepth = depths[z];
                const Point3d planen = (mp->iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp->CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp->iCamArr[camIndex] * Point2d(x * scale * step, y * scale * step)).normalize();
                const Point3d p = linePlaneIntersect(mp->CArr[camIndex], v, planep, planen);

                const int index = stepZ * volDimX * volDimY + y * volDimX + x;
                const int maxValue = 80;
                if(volumePtr[index] > maxValue)
                  continue;
                const rgb c = getRGBFromJetColorMap(static_cast<double>(volumePtr[index]) / double(maxValue));
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

void SemiGlobalMatchingVolume::export9PCSV(StaticVector<float>& depths, int camIndex,  int scale, int step, const std::string& name, const std::string& filepath) const
{
    const unsigned char* volumePtr = _volumeStepZ->getData().data();

    const int xOffset = std::floor(volDimX / 4.0f);
    const int yOffset = std::floor(volDimY / 4.0f);

    std::array<std::vector<double>, 9> ptsDepths;

    for(int iy = 0; iy < 3; ++iy)
    {
        for(int ix = 0; ix < 3; ++ix)
        {
            const int x = (ix + 1) * xOffset;
            const int y = (iy + 1) * yOffset;

            std::vector<double>& pDepths = ptsDepths.at(iy * 3 + ix);

            for(int stepZ = 0; stepZ < (volDimZ / volStepZ); ++stepZ)
            {
                pDepths.push_back(volumePtr[stepZ * volDimX * volDimY + y * volDimX + x]);
            }
        }
    }

    std::stringstream ss;
    {
      ss << name << "\n";
      int ptId = 1;
      for(const std::vector<double>& pDepths : ptsDepths)
      {
        ss << "p" << ptId << ";";
        for(const double& depth : pDepths)
          ss << depth << ";";
        ss << "\n";
        ++ptId;
      }
    }

    std::ofstream file;
    file.open(filepath, std::ios_base::app);
    if(file.is_open())
      file << ss.str();
}

/**
 * @param[in] volStepXY step in the image space
 */
void SemiGlobalMatchingVolume::SGMoptimizeVolumeStepZ(int rc, int volStepXY, int volLUX, int volLUY, int scale)
{
    long tall = clock();

    sp->cps.SGMoptimizeSimVolume(rc, _volumeStepZ, volDimX, volDimY, volDimZ / volStepZ, volStepXY, volLUX, volLUY,
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

void SemiGlobalMatchingVolume::copyVolume(const StaticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
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

void SemiGlobalMatchingVolume::copyVolume(const StaticVector<int>* volume)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const int* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = 0; z < volDimZ; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] =
                    (unsigned char)volumePtr[z * volDimY * volDimX + y * volDimX + x];
            }
        }
    }
}

void SemiGlobalMatchingVolume::addVolumeMin(const StaticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                int offs = z * volDimY * volDimX + y * volDimX + x;
                unsigned char va = _volumePtr[offs];
                unsigned char vn = volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
                _volumePtr[offs] = std::min(va, vn);
            }
        }
    }
}

void SemiGlobalMatchingVolume::addVolumeSecondMin(const StaticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    unsigned char* _volumeSecondBestPtr = _volumeSecondBest->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
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

void SemiGlobalMatchingVolume::addVolumeAvg(int n, const StaticVector<unsigned char>* volume, int zFrom, int nZSteps)
{
    unsigned char* _volumePtr = _volume->getDataWritable().data();
    const unsigned char* volumePtr = volume->getData().data();
#pragma omp parallel for
    for(int z = zFrom; z < zFrom + nZSteps; z++)
    {
        for(int y = 0; y < volDimY; y++)
        {
            for(int x = 0; x < volDimX; x++)
            {
                unsigned char va = _volumePtr[z * volDimY * volDimX + y * volDimX + x];
                unsigned char vn = volumePtr[(z - zFrom) * volDimY * volDimX + y * volDimX + x];
                float vv = ((float)va * (float)(n - 1) + (float)vn) / (float)n;
                assert(vv >= 0.0);
                assert(vv < 255.0);
                _volumePtr[z * volDimY * volDimX + y * volDimX + x] = (unsigned char)vv;
            }
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
