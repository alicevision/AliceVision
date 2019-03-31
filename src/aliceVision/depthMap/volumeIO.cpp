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
#include <aliceVision/depthMap/cuda/planeSweeping/plane_sweeping_cuda.hpp>
#include <aliceVision/depthMap/cuda/planeSweeping/host_utils.h>
#include <aliceVision/depthMap/cuda/planeSweeping/device_utils.h>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <iostream>
#include <sstream>
#include <stdexcept>

namespace aliceVision {
namespace depthMap {


void exportVolume(const CudaHostMemoryHeap<float, 3>& volumeSim, StaticVector<float>& depths, const mvsUtils::MultiViewParams& mp, int camIndex, int scale, int step, const std::string& filepath)
{
    sfmData::SfMData pointCloud;
    int xyStep = 10;

    IndexT landmarkId;

    auto volDim = volumeSim.getSize();
    size_t spitch = volumeSim.getBytesPaddedUpToDim(1);
    size_t pitch = volumeSim.getBytesPaddedUpToDim(0);

    ALICEVISION_LOG_DEBUG("DepthMap exportVolume: " << volDim[0] << " x " << volDim[1] << " x " << volDim[2] << ", xyStep=" << xyStep << ".");

    for (int z = 0; z < volDim[2]; ++z)
    {
        for (int y = 0; y < volDim[1]; y += xyStep)
        {
            for (int x = 0; x < volDim[0]; x += xyStep)
            {
                const double planeDepth = depths[z];
                const Point3d planen = (mp.iRArr[camIndex] * Point3d(0.0f, 0.0f, 1.0f)).normalize();
                const Point3d planep = mp.CArr[camIndex] + planen * planeDepth;
                const Point3d v = (mp.iCamArr[camIndex] * Point2d(x * scale * step, y * scale * step)).normalize();
                const Point3d p = linePlaneIntersect(mp.CArr[camIndex], v, planep, planen);

                const float maxValue = 80.f;
                float simValue = *get3DBufferAt_h<float>(volumeSim.getBuffer(), spitch, pitch, x, y, z);
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

void export9PCSV(const CudaHostMemoryHeap<float, 3>& volumeSim, StaticVector<float>& depths, int camIndex, int scale, int step, const std::string& name, const std::string& filepath)
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
                float simValue = *get3DBufferAt_h<float>(volumeSim.getBuffer(), spitch, pitch, x, y, iz);
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


} // namespace depthMap
} // namespace aliceVision
