// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthMapUtils.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsUtils/depthSimMapIO.hpp>

namespace aliceVision {
namespace depthMap {

void writeDeviceImage(const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp, const std::string& path) 
{
    const CudaSize<2>& imgSize = in_img_dmp.getSize();
    
    // copy image from device pitched memory to host memory
    CudaHostMemoryHeap<CudaRGBA, 2> img_hmh(imgSize);
    img_hmh.copyFrom(in_img_dmp);

    // copy image from host memory to a vector
    std::vector<ColorRGBf> img(imgSize.x() * imgSize.y(), {0.f,0.f,0.f});

    for(size_t x = 0; x < imgSize.x(); ++x)
    {
        for(size_t y = 0; y < imgSize.y(); ++y)
        {
            const CudaRGBA& rgba_hmh = img_hmh(x, y);
            ColorRGBf& rgb = img.at(y * imgSize.x() + x);
            rgb.r = rgba_hmh.x;
            rgb.g = rgba_hmh.y;
            rgb.b = rgba_hmh.z;
        }
    }

    // write the vector buffer
    using namespace imageIO;
    imageIO::writeImage(path, int(imgSize.x()), int(imgSize.y()), img, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION));
}

void resetDepthSimMap(CudaHostMemoryHeap<float2, 2>& inout_depthSimMap_hmh, float depth, float sim)
{
  const CudaSize<2>& depthSimMapSize = inout_depthSimMap_hmh.getSize();

  for(size_t x = 0; x < depthSimMapSize.x(); ++x)
  {
      for(size_t y = 0; y < depthSimMapSize.y(); ++y)
      {
          float2& depthSim_hmh = inout_depthSimMap_hmh(x, y);
          depthSim_hmh.x = depth;
          depthSim_hmh.y = sim;
      }
  }
}

void copyDepthSimMap(std::vector<float>& out_depthMap, std::vector<float>& out_simMap, const CudaHostMemoryHeap<float2, 2>& in_depthSimMap_hmh, const ROI& roi, int downscale)
{
    const ROI downscaledROI = downscaleROI(roi, downscale);
    const size_t vectorSize = downscaledROI.width() * downscaledROI.height();

    // resize output vectors
    out_depthMap.resize(vectorSize);
    out_simMap.resize(vectorSize);

    // copy image from host memory to output vectors
    for(size_t x = 0; x < downscaledROI.width(); ++x)
    {
        for(size_t y = 0; y < downscaledROI.height(); ++y)
        {
            const size_t index = y * downscaledROI.width() + x;
            const float2& depthSim = in_depthSimMap_hmh(x, y);
            out_depthMap[index] = depthSim.x;
            out_simMap[index] = depthSim.y;
        }
    }
}

void copyDepthSimMap(std::vector<float>& out_depthMap, std::vector<float>& out_simMap, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const ROI& roi, int downscale)
{
    // copy depth/sim maps from device pitched memory to host memory
    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(in_depthSimMap_dmp.getSize());
    depthSimMap_hmh.copyFrom(in_depthSimMap_dmp);

    copyDepthSimMap(out_depthMap, out_simMap, depthSimMap_hmh, roi, downscale);
}

void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi, 
                      const CudaHostMemoryHeap<float2, 2>& in_depthSimMap_hmh,
                      int scale,
                      int step,
                      const std::string& customSuffix)
{
    const int scaleStep = scale * step;

    std::vector<float> depthMap;
    std::vector<float> simMap;

    copyDepthSimMap(depthMap, simMap, in_depthSimMap_hmh, roi, scaleStep);

    mvsUtils::writeDepthSimMap(rc, mp, tileParams, roi, depthMap, simMap, scale, step, customSuffix);
}

void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi, 
                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                      int scale,
                      int step,
                      const std::string& customSuffix)
{
    const int scaleStep = scale * step;

    std::vector<float> depthMap;
    std::vector<float> simMap;

    copyDepthSimMap(depthMap, simMap, in_depthSimMap_dmp, roi, scaleStep);

    mvsUtils::writeDepthSimMap(rc, mp, tileParams, roi, depthMap, simMap, scale, step, customSuffix);
}

void mergeDepthSimMapTiles(int rc,
                           const mvsUtils::MultiViewParams& mp,
                           int scale,
                           int step,
                           const std::string& customSuffix)
{
    std::vector<float> depthMap;
    std::vector<float> simMap;

    mvsUtils::readDepthSimMap(rc, mp, depthMap, simMap, scale, step, customSuffix);  // read and merge tiles
    mvsUtils::writeDepthSimMap(rc, mp, depthMap, simMap, scale, step, customSuffix); // write the merged depth/sim maps
}

} // namespace depthMap
} // namespace aliceVision
