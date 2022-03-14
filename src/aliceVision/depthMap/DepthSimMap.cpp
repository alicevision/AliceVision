// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DepthSimMap.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/depthSimMapIO.hpp>
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/jetColorMap.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>
#include <aliceVision/depthMap/cuda/host/memory.hpp>

#include <iostream>

#define ALICEVISION_DEPTHMAP_UPSCALE_NEAREST_NEIGHBOR

namespace aliceVision {
namespace depthMap {

DepthSimMap::DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step) 
    : _mp(mp)
    , _rc(rc)
    , _scale(scale)
    , _step(step)
    , _roi(ROI(0, mp.getOriginalWidth(rc), 0, mp.getOriginalHeight(rc)))
    , _width(std::ceil(mp.getOriginalWidth(rc) / float(_mp.getProcessDownscale() * scale * step)))
    , _height(std::ceil(mp.getOriginalHeight(rc) / float(_mp.getProcessDownscale() * scale * step)))
{
    _dsm.resize(_width * _height, DepthSim(-1.0f, 1.0f));
}

DepthSimMap::DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, const mvsUtils::TileParams& tileParams, int scale, int step, const ROI& roi) 
    : _mp(mp)
    , _rc(rc)
    , _scale(scale)
    , _step(step)
    , _roi(roi)
    , _tileParams(tileParams)
    , _width(downscaleRange(roi.x, float(_mp.getProcessDownscale() * scale * step)).size())
    , _height(downscaleRange(roi.y, float(_mp.getProcessDownscale() * scale * step)).size())
{
    _dsm.resize(_width * _height, DepthSim(-1.0f, 1.0f));
}

DepthSim getPixelValueInterpolated(const std::vector<DepthSim>& depthSimMap, double x, double y, int width, int height)
{
#ifdef ALICEVISION_DEPTHMAP_UPSCALE_NEAREST_NEIGHBOR
    // Nearest neighbor, no interpolation
    int xp = static_cast<int>(x + 0.5);
    int yp = static_cast<int>(y + 0.5);

    xp = std::min(xp, width - 1);
    yp = std::min(yp, height - 1);

    return depthSimMap[yp * width + xp];
#else
    // Interpolate using the distance to the pixels center
    int xp = static_cast<int>(x);
    int yp = static_cast<int>(y);
    xp = std::min(xp, width - 2);
    yp = std::min(yp, height - 2);
    const DepthSim lu = depthSimMap[yp       * width + xp    ];
    const DepthSim ru = depthSimMap[yp       * width + xp + 1];
    const DepthSim rd = depthSimMap[(yp + 1) * width + xp + 1];
    const DepthSim ld = depthSimMap[(yp + 1) * width + xp    ];

    if(lu.depth <= 0.0f || ru.depth <= 0.0f ||
        rd.depth <= 0.0f || ld.depth <= 0.0f)
    {
        DepthSim acc(0.0f, 0.0f);
        int count = 0;
        if(lu.depth > 0.0f)
        {
            acc = acc + lu;
            ++count;
        }
        if(ru.depth > 0.0f)
        {
            acc = acc + ru;
            ++count;
        }
        if(rd.depth > 0.0f)
        {
            acc = acc + rd;
            ++count;
        }
        if(ld.depth > 0.0f)
        {
            acc = acc + ld;
            ++count;
        }
        if(count != 0)
        {
            return acc / float(count);
        }
        else
        {
            return DepthSim(-1.0f, 1.0f);
        }
    }

    // bilinear interpolation
    const float ui = x - static_cast<float>(xp);
    const float vi = y - static_cast<float>(yp);
    const DepthSim u = lu + (ru - lu) * ui;
    const DepthSim d = ld + (rd - ld) * ui;
    const DepthSim out = u + (d - u) * vi;

    return out;
#endif
}

void DepthSimMap::initFromSmaller(const DepthSimMap& other)
{
    if ((_scale * _step) > (other._scale * other._step))
    {
        throw std::runtime_error("Error DepthSimMap: You cannot init from a larger map.");
    }
    const double ratio = double(_scale * _step) / double(other._scale * other._step);

    ALICEVISION_LOG_TRACE("Initialize depth/sim map from smaller: ratio = " << ratio << ", other scale x step = " << other._scale * other._step << ", scale x step = " << _scale * _step);
    
    for (int y = 0; y < _height; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;
        for (int x = 0; x < _width; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;
            const DepthSim otherDepthSim = getPixelValueInterpolated(other._dsm, ox, oy, other._width, other._height);
            _dsm[y * _width + x] = otherDepthSim;
        }
    }
}

Point2d DepthSimMap::getMaxMinDepth() const
{
    float maxDepth = -1.0f;
    float minDepth = std::numeric_limits<float>::max();

    for(int i = 0; i < _dsm.size(); ++i)
    {
        if(_dsm[i].depth > -1.0f)
        {
            maxDepth = std::max(maxDepth, _dsm[i].depth);
            minDepth = std::min(minDepth, _dsm[i].depth);
        }
    }

    return Point2d(maxDepth, minDepth);
}

Point2d DepthSimMap::getMaxMinSim() const
{
    float maxSim = -1.0f;
    float minSim = std::numeric_limits<float>::max();

    for(int i = 0; i < _dsm.size(); ++i)
    {
        if(_dsm[i].sim > -1.0f)
        {
            maxSim = std::max(maxSim, _dsm[i].sim);
            minSim = std::min(minSim, _dsm[i].sim);
        }
    }

    return Point2d(maxSim, minSim);
}

float DepthSimMap::getPercentileDepth(float percentile) const
{
    const int mapSize = _width * _height;
    const int step = std::max(1, mapSize / 50000);
    const int n = mapSize / std::max(1, (step - 1));

    StaticVector<float> depths;
    depths.reserve(n);

    for(int i = 0; i < mapSize; i += step)
    {
        if(_dsm[i].depth > -1.0f)
        {
            depths.push_back(_dsm[i].depth);
        }
    }

    qsort(&depths[0], depths.size(), sizeof(float), qSortCompareFloatAsc);

    float out = depths[(float)((float)depths.size() * percentile)];

    return out;
}

void DepthSimMap::getDepthMapStep1(std::vector<float>& out_depthMap) const
{
    // dimensions of the output depth map 
    // with only process downscale and internal scale applied
    const int out_width  = downscaleRange(_roi.x, float(_mp.getProcessDownscale() * _scale)).size();
    const int out_height = downscaleRange(_roi.y, float(_mp.getProcessDownscale() * _scale)).size();

    // resize the output depth map 
    out_depthMap.resize(out_width * out_height);

    // compute step ratio
    const double ratio = 1.0 / double(_step);

    ALICEVISION_LOG_TRACE("Compute depth map step1, ratio: " << ratio);

    for(int y = 0; y < out_height; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;

        for(int x = 0; x < out_width; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;

            out_depthMap[y * out_width + x] = getPixelValueInterpolated(_dsm, ox, oy, _width, _height).depth;
        }
    }
}

void DepthSimMap::getSimMapStep1(std::vector<float>& out_simMap) const
{
    // dimensions of the output sim map
    // with only process downscale and internal scale applied
    const int out_width  = downscaleRange(_roi.x, float(_mp.getProcessDownscale() * _scale)).size();
    const int out_height = downscaleRange(_roi.y, float(_mp.getProcessDownscale() * _scale)).size();

    // resize the output sim map 
    out_simMap.resize(out_width * out_height);

    // compute step ratio
    const double ratio = 1.0 / double(_step);

    ALICEVISION_LOG_TRACE("Compute sim map step1, ratio: " << ratio);

    for(int y = 0; y < out_height; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;

        for(int x = 0; x < out_width; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;

            out_simMap[y * out_width + x] = getPixelValueInterpolated(_dsm, ox, oy, _width, _height).sim;
        }
    }
}

void DepthSimMap::getDepthMap(std::vector<float>& out_depthMap) const
{
    out_depthMap.resize(_dsm.size());

    for(int i = 0; i < _dsm.size(); ++i)
    {
        out_depthMap[i] = _dsm[i].depth;
    }
}

void DepthSimMap::getSimMap(std::vector<float>& out_simMap) const
{
    out_simMap.resize(_dsm.size());

    for(int i = 0; i < _dsm.size(); ++i)
    {
        out_simMap[i] = _dsm[i].sim;
    }
}

void DepthSimMap::copyTo(CudaDeviceMemoryPitched<float2, 2>& out_depthSimMap_dmp) const
{
    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(CudaSize<2>(_width, _height));

    for(int y = 0; y < _height; ++y)
    {
        for(int x = 0; x < _width; ++x)
        {
            float2& depthSim_h = depthSimMap_hmh(x, y);
            const DepthSim& depthSim = getDepthSim(x, y);
            depthSim_h.x = depthSim.depth;
            depthSim_h.y = depthSim.sim;
        }
    }

    if(depthSimMap_hmh.getSize() != out_depthSimMap_dmp.getSize())
    {
        ALICEVISION_THROW_ERROR("Cannot copy depth/sim map to a device memory array, non-compatible buffer sizes.");
    }

    out_depthSimMap_dmp.copyFrom(depthSimMap_hmh);
}
 
void DepthSimMap::copyFrom(const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp) 
{
    CudaHostMemoryHeap<float2, 2> depthSimMap_hmh(in_depthSimMap_dmp.getSize());
    depthSimMap_hmh.copyFrom(in_depthSimMap_dmp);

    if(in_depthSimMap_dmp.getSize().x() != _width || 
       in_depthSimMap_dmp.getSize().y() != _height)
    {
        ALICEVISION_THROW_ERROR("Cannot copy depth/sim map from a device memory array, non-compatible buffer sizes.");
    }

    for(int y = 0; y < _height; ++y)
    {
        for(int x = 0; x < _width; ++x)
        {
            const float2& depthSim_h = depthSimMap_hmh(x, y);
            DepthSim& depthSim = getDepthSim(x, y);
            depthSim.depth = depthSim_h.x;
            depthSim.sim = depthSim_h.y;
        }
    }
}

void DepthSimMap::copyFrom(const CudaDeviceMemoryPitched<float, 2>& in_depthMap_dmp, const CudaDeviceMemoryPitched<float, 2>& in_simMap_dmp)
{
    CudaHostMemoryHeap<float, 2> depthMap_hmh(in_depthMap_dmp.getSize());
    depthMap_hmh.copyFrom(in_depthMap_dmp);

    CudaHostMemoryHeap<float, 2> simMap_hmh(in_simMap_dmp.getSize());
    simMap_hmh.copyFrom(in_simMap_dmp);

    if(depthMap_hmh.getSize().x() != simMap_hmh.getSize().x() || 
       depthMap_hmh.getSize().y() != simMap_hmh.getSize().y())
    {
        ALICEVISION_THROW_ERROR("Cannot copy depth/sim map from two device memory arrays, depth and sim buffer size are different.");
    }

    if(depthMap_hmh.getSize().x() != _width || 
       depthMap_hmh.getSize().y() != _height)
    {
        ALICEVISION_THROW_ERROR("Cannot copy depth/sim map from two device memory arrays, non-compatible buffer sizes.");
    }

    for(int y = 0; y < _height; ++y)
    {
        for(int x = 0; x < _width; ++x)
        {
            DepthSim& depthSim = getDepthSim(x, y);
            depthSim.depth = depthMap_hmh(x, y);
            depthSim.sim = simMap_hmh(x, y);
        }
    }
}

void DepthSimMap::saveToImage(const std::string& filename, float simThr) const
{
    const int bufferWidth = 2 * _width;
    std::vector<ColorRGBf> colorBuffer(bufferWidth * _height);

    try
    {
        Point2d maxMinDepth;
        maxMinDepth.x = getPercentileDepth(0.9) * 1.1;
        maxMinDepth.y = getPercentileDepth(0.01) * 0.8;

        Point2d maxMinSim = Point2d(simThr, -1.0f);

        if(simThr < -1.0f)
        {
            Point2d autoMaxMinSim = getMaxMinSim();
            // only use it if the default range is valid
            if (std::abs(autoMaxMinSim.x - autoMaxMinSim.y) > std::numeric_limits<float>::epsilon())
                maxMinSim = autoMaxMinSim;
        }

        for (int y = 0; y < _height; y++)
        {
            for (int x = 0; x < _width; x++)
            {
                const DepthSim& depthSim = _dsm[y * _width + x];
                float depth = (depthSim.depth - maxMinDepth.y) / (maxMinDepth.x - maxMinDepth.y);
                colorBuffer.at(y * bufferWidth + x) = getColorFromJetColorMap(depth);

                float sim = (depthSim.sim - maxMinSim.y) / (maxMinSim.x - maxMinSim.y);
                colorBuffer.at(y * bufferWidth + _width + x) = getColorFromJetColorMap(sim);
            }
        }

        oiio::ParamValueList metadata;
        using namespace imageIO;
        writeImage(filename, bufferWidth, _height, colorBuffer, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);
    }
    catch (...)
    {
        ALICEVISION_LOG_ERROR("Failed to save '" << filename << "' (simThr: " << simThr << ")");
    }
}

void DepthSimMap::save(const std::string& customSuffix, bool useStep1) const
{
    const int step = (useStep1) ? 1 : _step;

    std::vector<float> depthMap;
    std::vector<float> simMap;

    if(useStep1)
    {
        getDepthMapStep1(depthMap);
        getSimMapStep1(simMap);
    }
    else
    {
        getDepthMap(depthMap);
        getSimMap(simMap);
    }

    mvsUtils::writeDepthSimMap(_rc, _mp, _tileParams, _roi, depthMap, simMap, _scale, step, customSuffix);
}

void DepthSimMap::load(const std::string& customSuffix)
{
    std::vector<float> depthMap;
    std::vector<float> simMap;

    mvsUtils::readDepthSimMap(_rc, _mp, depthMap, simMap, _scale, _step, customSuffix);

    for(int i = 0; i < _dsm.size(); ++i)
    {
        DepthSim& depthSim = _dsm[i];
        depthSim.depth = depthMap[i];
        depthSim.sim = simMap[i];
    }
}

} // namespace depthMap
} // namespace aliceVision
