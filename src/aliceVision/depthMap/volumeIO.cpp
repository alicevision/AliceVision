// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "volumeIO.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Matrix3x3.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/OrientedPoint.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap/cuda/device/device_utils.h>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aliceVision {
namespace depthMap {

void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                                const StaticVector<float>& depths, 
                                int camIndex, 
                                const std::string& name, 
                                const std::string& filepath)
{
    const auto volDim = volumeSim.getSize();
    const size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    const size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    const int sampleSize = 3;

    const int xOffset = std::floor(volDim[0] / (sampleSize + 1.0f));
    const int yOffset = std::floor(volDim[1] / (sampleSize + 1.0f));

    std::vector<std::vector<float>> ptsDepths(sampleSize*sampleSize);

    for (int iy = 0; iy < sampleSize; ++iy)
    {
        for (int ix = 0; ix < sampleSize; ++ix)
        {
            const int x = (ix + 1) * xOffset;
            const int y = (iy + 1) * yOffset;

            std::vector<float>& pDepths = ptsDepths.at(iy * sampleSize + ix);

            for (int iz = 0; iz < volDim[2]; ++iz)
            {
                float simValue = *get3DBufferAt_h<TSim>(volumeSim.getBuffer(), spitch, pitch, x, y, iz);
                pDepths.push_back(simValue);
            }
        }
    }

    std::stringstream ss;
    {
        ss << name << "\n";
        int ptId = 1;
        for (const std::vector<float>& pDepths : ptsDepths)
        {
            ss << "p" << ptId << ";";
            for (const float depth : pDepths)
                ss << depth << ";";
            ss << "\n";
            ++ptId;
        }
    }

    std::ofstream file;
    file.open(filepath, std::ios_base::app);
    if (file.is_open())
        file << ss.str();
}

void exportSimilaritySamplesCSV(const CudaHostMemoryHeap<TSimRefine, 3>& volumeSim,
                                int camIndex,  
                                const std::string& name, 
                                const std::string& filepath)
{
    const auto volDim = volumeSim.getSize();
    const size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    const size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    const int sampleSize = 9;

    const int xOffset = std::floor(volDim.x() / (sampleSize + 1.0f));
    const int yOffset = std::floor(volDim.y() / (sampleSize + 1.0f));

    std::vector<std::vector<float>> simPerDepthsPerPts(sampleSize * sampleSize);

    for(int iy = 0; iy < 1; ++iy)
    {
        for(int ix = 0; ix < sampleSize; ++ix)
        {
            const int x = (ix + 1) * xOffset;
            const int y = (iy + 1) * yOffset;

            std::vector<float>& simPerDepths = simPerDepthsPerPts.at(iy * sampleSize + ix);
            simPerDepths.reserve(volDim.z());

            for(int iz = 0; iz < volDim.z(); ++iz)
            {
                float sim = float(*get3DBufferAt_h<TSimRefine>(volumeSim.getBuffer(), spitch, pitch, x, y, iz));
                simPerDepths.push_back(sim);
            }
        }
    }

    std::stringstream ss;
    {
        ss << name << "\n";
        int ptId = 1;
        for(const std::vector<float>& simPerDepths : simPerDepthsPerPts)
        {
            ss << "p" << ptId << ";";
            for(const float sim : simPerDepths)
                ss << sim << ";";
            ss << "\n";
            ++ptId;
        }
    }

    std::ofstream file;
    file.open(filepath, std::ios_base::app);
    if(file.is_open())
        file << ss.str();
}
void exportSimilarityVolume(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                            const StaticVector<float>& depths, 
                            const mvsUtils::MultiViewParams& mp, 
                            int camIndex, 
                            const SgmParams& sgmParams, 
                            const std::string& filepath)
{
    sfmData::SfMData pointCloud;
    const int xyStep = 10;

    IndexT landmarkId;

    const auto volDim = volumeSim.getSize();
    const size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    const size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    ALICEVISION_LOG_DEBUG("DepthMap exportSimilarityVolume: " << volDim[0] << " x " << volDim[1] << " x " << volDim[2] << ", xyStep=" << xyStep << ".");

    for (int z = 0; z < volDim[2]; ++z)
    {
        for (int y = 0; y < volDim[1]; y += xyStep)
        {
            for (int x = 0; x < volDim[0]; x += xyStep)
            {
                const double planeDepth = depths[z];
                const Point3d planen = (mp.iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp.CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp.iCamArr[camIndex] * Point2d(x * sgmParams.scale * sgmParams.stepXY, y * sgmParams.scale * sgmParams.stepXY)).normalize();
                const Point3d p = linePlaneIntersect(mp.CArr[camIndex], v, planep, planen);

                const float maxValue = 80.f;
                float simValue = *get3DBufferAt_h<TSim>(volumeSim.getBuffer(), spitch, pitch, x, y, z);
                if (simValue > maxValue)
                    continue;
                const rgb c = getRGBFromJetColorMap(simValue / maxValue);
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSim, 3>& volumeSim, 
                                 const StaticVector<float>& depths,
                                 const mvsUtils::MultiViewParams& mp, 
                                 int camIndex, 
                                 const SgmParams& sgmParams,
                                 const std::string& filepath)
{
    sfmData::SfMData pointCloud;

    IndexT landmarkId;

    const auto volDim = volumeSim.getSize();
    const size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    const size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    ALICEVISION_LOG_DEBUG("Export similarity volume cross: " << volDim[0] << " x " << volDim[1] << " x " << volDim[2] << ".");

    for(int z = 0; z < volDim[2]; ++z)
    {
        for(int y = 0; y < volDim[1]; ++y)
        {
            const bool yCenter = (y >= volDim[1]/2) && ((y-1)< volDim[1]/2);
            const int xIdxStart = (yCenter ? 0 : (volDim[0] / 2));
            const int xIdxStop = (yCenter ? volDim[0] : (xIdxStart + 1));

            for(int x = xIdxStart; x < xIdxStop; ++x)
            {
                const double planeDepth = depths[z];
                const Point3d planen = (mp.iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp.CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp.iCamArr[camIndex] * Point2d(x * sgmParams.scale * sgmParams.stepXY, y * sgmParams.scale * sgmParams.stepXY)).normalize();
                const Point3d p = linePlaneIntersect(mp.CArr[camIndex], v, planep, planen);

                const float maxValue = 80.f;
                float simValue = *get3DBufferAt_h<TSim>(volumeSim.getBuffer(), spitch, pitch, x, y, z);

                if(simValue > maxValue)
                    continue;

                const rgb c = getRGBFromJetColorMap(simValue / maxValue);
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

void exportSimilarityVolumeCross(const CudaHostMemoryHeap<TSimRefine, 3>& volumeSim, 
                                 const DepthSimMap& depthSimMapSgmUpscale,
                                 const mvsUtils::MultiViewParams& mp, 
                                 int camIndex, 
                                 const RefineParams& refineParams,
                                 const std::string& filepath)
{
    sfmData::SfMData pointCloud;

    const auto volDim = volumeSim.getSize();
    const size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    const size_t pitch = volumeSim.getBytesPaddedUpToDim(0);
    const int width = depthSimMapSgmUpscale.getWidth();

    ALICEVISION_LOG_DEBUG("Export similarity volume cross: " << volDim[0] << " x " << volDim[1] << " x " << volDim[2] << ".");

    IndexT landmarkId = 0;

    for(int y = 0; y < volDim[1]; ++y)
    {
        const bool yCenter = ((y*2) == volDim[1]);
        const int xIdxStart = (yCenter ? 0 : (volDim[0] / 2));
        const int xIdxStop = (yCenter ? volDim[0] : (xIdxStart + 1));

        for(int x = xIdxStart; x < xIdxStop; ++x)
        {
            const float downscaleX = float(x * refineParams.scale * refineParams.stepXY);
            const float downscaleY = float(y * refineParams.scale * refineParams.stepXY);
            const Point2d pix = Point2d(downscaleX, downscaleY);

            const double orignalDepth = double(depthSimMapSgmUpscale._dsm[y * width + x].depth);

            if(orignalDepth < 0.0f) // original depth invalid or masked
                continue;

            const Point3d originalP = mp.CArr[camIndex] + (mp.iCamArr[camIndex] * pix).normalize() * orignalDepth;
            const double pixSize = mp.getCamPixelSize(originalP, camIndex);

            for(int z = 0; z < volDim[2]; ++z)
            {
                const float simValue = float(*get3DBufferAt_h<TSimRefine>(volumeSim.getBuffer(), spitch, pitch, x, y, z));

                const float maxValue = 10.f; // sum of similarity between 0 and 1
                if(simValue > maxValue)
                    continue;

                const int relativeDepthIndexOffset = z - ((refineParams.nDepthsToRefine - 1) / 2);
                const double depth = orignalDepth + (relativeDepthIndexOffset * pixSize); // original depth + z based pixSize offset

               const Point3d p = mp.CArr[camIndex] + (mp.iCamArr[camIndex] * pix).normalize() * depth;

                const rgb c = getRGBFromJetColorMap(simValue / maxValue);
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

inline unsigned char float_to_uchar(float v)
{
    float vv = std::max(0.f, v);
    vv = std::min(255.f, vv);
    unsigned char out = vv;
    return out;
}

inline rgb float4_to_rgb(const float4& v)
{
    return { float_to_uchar(v.x), float_to_uchar(v.y), float_to_uchar(v.z) };
}

void exportColorVolume(const CudaHostMemoryHeap<float4, 3>& volumeSim, 
                       const std::vector<float>& depths, 
                       int startDepth, 
                       int nbDepths, 
                       const mvsUtils::MultiViewParams& mp, 
                       int camIndex, 
                       int scale, 
                       int step, 
                       const std::string& filepath)
{
    sfmData::SfMData pointCloud;
    int xyStep = 10;

    IndexT landmarkId;

    auto volDim = volumeSim.getSize();
    size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    ALICEVISION_LOG_DEBUG("DepthMap exportColorVolume: " << volDim[0] << " x " << volDim[1] << " x " << nbDepths << ", volDim[2]=" << volDim[2] << ", xyStep=" << xyStep << ".");

    for (int z = 0; z < nbDepths; ++z)
    {
        for (int y = 0; y < volDim[1]; y += xyStep)
        {
            for (int x = 0; x < volDim[0]; x += xyStep)
            {
                const double planeDepth = depths[startDepth + z];
                const Point3d planen = (mp.iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp.CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp.iCamArr[camIndex] * Point2d(x * scale * step, y * scale * step)).normalize();
                const Point3d p = linePlaneIntersect(mp.CArr[camIndex], v, planep, planen);

                float4 colorValue = *get3DBufferAt_h<float4>(volumeSim.getBuffer(), spitch, pitch, x, y, z);
                const rgb c = float4_to_rgb(colorValue); // TODO: convert Lab color into sRGB color
                pointCloud.getLandmarks()[landmarkId] = sfmData::Landmark(Vec3(p.x, p.y, p.z), feature::EImageDescriberType::UNKNOWN, sfmData::Observations(), image::RGBColor(c.r, c.g, c.b));

                ++landmarkId;
            }
        }
    }

    sfmDataIO::Save(pointCloud, filepath, sfmDataIO::ESfMData::STRUCTURE);
}

} // namespace depthMap
} // namespace aliceVision
