// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "DepthSimMap.hpp"
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
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
    , _width(mp.getOriginalWidth(rc) / float(_mp.getProcessDownscale() * scale * step))
    , _height(mp.getOriginalHeight(rc) / float(_mp.getProcessDownscale() * scale * step))
{
    _dsm.resize_with(_width * _height, DepthSim(-1.0f, 1.0f));
}

DepthSimMap::DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step,const TileParams& tileParams, const ROI& roi) 
    : _mp(mp)
    , _rc(rc)
    , _scale(scale)
    , _step(step)
    , _roi(roi)
    , _tileParams(tileParams)
    , _width(downscaleRange(roi.x, float(_mp.getProcessDownscale() * scale * step)).size())
    , _height(downscaleRange(roi.y, float(_mp.getProcessDownscale() * scale * step)).size())
{
    _dsm.resize_with(_width * _height, DepthSim(-1.0f, 1.0f));
}

DepthSim getPixelValueInterpolated(const StaticVector<DepthSim>& depthSimMap, double x, double y, int width, int height)
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

void DepthSimMap::getDepthMapStep1(StaticVector<float>& out_depthMap) const
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

void DepthSimMap::getSimMapStep1(StaticVector<float>& out_simMap) const
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

void DepthSimMap::getDepthMap(StaticVector<float>& out_depthMap) const
{
    out_depthMap.resize(_dsm.size());

    for(int i = 0; i < _dsm.size(); ++i)
    {
        out_depthMap[i] = _dsm[i].depth;
    }
}

void DepthSimMap::getSimMap(StaticVector<float>& out_simMap) const
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
    StaticVector<float> depthMap;
    StaticVector<float> simMap;

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

    const int step = (useStep1 ? 1 : _step);
    const int scaleStep = _scale * step;
    const int downscale = _mp.getDownscaleFactor(_rc) * scaleStep;
    const int nbDepthValues = std::count_if(depthMap.begin(), depthMap.end(), [](float v) { return v > 0.0f; });

    oiio::ParamValueList metadata = imageIO::getMetadataFromMap(_mp.getMetadata(_rc));

    // tile metadata
    if((_tileParams.width > 0) && (_tileParams.height > 0) && (_tileParams.padding > 0))
    {
        metadata.push_back(oiio::ParamValue("AliceVision:tileWidth", int(_tileParams.width)));
        metadata.push_back(oiio::ParamValue("AliceVision:tileHeight", int(_tileParams.height)));
        metadata.push_back(oiio::ParamValue("AliceVision:tilePadding", int(_tileParams.padding)));    
    }

    // downscale metadata
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", downscale));

    double s = scaleStep;
    Point3d C = _mp.CArr[_rc];
    Matrix3x3 iP = _mp.iCamArr[_rc];
    if (s > 1.0)
    {
        Matrix3x4 P = _mp.camArr[_rc];
        for (int i = 0; i < 8; ++i)
            P.m[i] /= s;
        Matrix3x3 K, iK;
        Matrix3x3 R, iR;

        P.decomposeProjectionMatrix(K, R, C); // replace C
        iK = K.inverse();
        iR = R.inverse();
        iP = iR * iK; // replace iP
    }

    // CArr & iCamArr metadata
    metadata.push_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, C.m));
    metadata.push_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, iP.m));

    // min/max/nb depth metadata
    { 
        const Point2d maxMinDepth = getMaxMinDepth();
        metadata.push_back(oiio::ParamValue("AliceVision:minDepth", static_cast<float>(maxMinDepth.y)));
        metadata.push_back(oiio::ParamValue("AliceVision:maxDepth", static_cast<float>(maxMinDepth.x)));
        metadata.push_back(oiio::ParamValue("AliceVision:nbDepthValues", oiio::TypeDesc::INT32, 1, &nbDepthValues));
    }

    // projection matrix metadata
    {
        std::vector<double> matrixP = _mp.getOriginalP(_rc);
        metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, matrixP.data()));
    }

    // get full image dimensions
    const ROI downscaledROI = downscaleROI(_roi, downscale);
    const int imageWidth  = _mp.getOriginalWidth(_rc)  / downscale;
    const int imageHeight = _mp.getOriginalHeight(_rc) / downscale;

    oiio::ROI imageROI = oiio::ROI::All();
    std::string depthMapPath;
    std::string simMapPath;

    if(downscaledROI.width() != imageWidth || downscaledROI.height() != imageHeight)
    {
        // tiled depth/sim map
        imageROI = oiio::ROI(downscaledROI.x.begin, downscaledROI.x.end, downscaledROI.y.begin, downscaledROI.y.end, 0, 1, 0, 1);
;       depthMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, _scale, customSuffix, _roi.x.begin, _roi.y.begin);
        simMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, _scale, customSuffix, _roi.x.begin, _roi.y.begin);
    }
    else
    {
        // fullsize depth/sim map
        depthMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, _scale, customSuffix);
        simMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, _scale, customSuffix);
    }

    // write depth/sim map
    using namespace imageIO;
    writeImage(depthMapPath, imageWidth, imageHeight, depthMap.getDataWritable(), EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata, imageROI);
    writeImage(simMapPath, imageWidth, imageHeight, simMap.getDataWritable(), imageIO::EImageQuality::OPTIMIZED, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata, imageROI);
}

void DepthSimMap::loadFromTiles(const std::vector<ROI>& tileRoiList, const std::string& customSuffix)
{
    // initialize depth/sim map at 0
    for(int i = 0; i < _dsm.size(); ++i)
    {
        _dsm[i] = {0, 0};
    }

    // find and join each roi to the depth and similarity buffers
    for(const ROI& roi : tileRoiList)
    {
        loadTileWeighted(roi, customSuffix);
    }
}

void weightTileBorder(int a, int b, int c, int d, 
                      int currentTileWidth, 
                      int currentTileHeight,
                      int borderWidth, 
                      int borderHeight,  
                      const Point2d& lu, 
                      std::vector<float>& in_tileDepthMap, 
                      std::vector<float>& in_tileSimMap)
{
    const Point2d rd = lu + Point2d(borderWidth, borderHeight);

    const int endX = std::min(int(rd.x), currentTileWidth);
    const int endY = std::min(int(rd.y), currentTileHeight);

    for(int x = lu.x; x < endX; ++x)
    {
        for(int y = lu.y; y < endY; ++y)
        {
            // bilinear interpolation
            const float ui = (rd.x - x) / borderWidth;
            const float vi = (x - lu.x) / borderWidth;
            const float uj = (rd.y - y) / borderHeight;
            const float vj = (y - lu.y) / borderHeight;

            const float weight = uj * (ui * a + vi * b) + vj * (ui * d + vi * c);

            // apply weight to tile depth/sim map
            in_tileDepthMap[y * currentTileWidth + x] *= weight;
            in_tileSimMap[y * currentTileWidth + x] *= weight;
        }
    }
}

void DepthSimMap::loadTileWeighted(const ROI& tileRoi, const std::string& customSuffix)
{
    // get depth/sim map downscale factor (could be 1)
    const int downscale = _mp.getProcessDownscale() * _scale * _step;

    // get downscaled ROI
    const ROI downscaledRoi = downscaleROI(tileRoi, downscale);
    const int currentTileWidth = downscaledRoi.width();
    const int currentTileHeight = downscaledRoi.height();

    // get tile depth/sim map filenames
    const std::string depthMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, _scale, customSuffix, tileRoi.x.begin, tileRoi.y.begin);
    const std::string simMapPath = getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, _scale, customSuffix, tileRoi.x.begin, tileRoi.y.begin);

    // create tile depth/sim map buffers
    std::vector<float> tileDepthMap;
    std::vector<float> tileSimMap;

    // read tile depth/sim map
    {
        using namespace imageIO;
        int w, h;
        readImage(depthMapPath, w, h, tileDepthMap, EImageColorSpace::NO_CONVERSION);
        readImage(simMapPath, w, h, tileSimMap, EImageColorSpace::NO_CONVERSION);
    }

    // get tile dimensions from metadata
    TileParams tileParams;
    {
        // read tile metadata
        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(depthMapPath, metadata);

        const auto tileWidthIt = metadata.find("AliceVision:tileWidth");
        const auto tileHeightIt = metadata.find("AliceVision:tileHeight");
        const auto tilePaddingIt = metadata.find("AliceVision:tilePadding");

        if(tileWidthIt != metadata.end() && tileWidthIt->type() == oiio::TypeDesc::INT)
            tileParams.width = tileWidthIt->get_int();

        if(tileHeightIt != metadata.end() && tileHeightIt->type() == oiio::TypeDesc::INT)
            tileParams.height = tileHeightIt->get_int();

        if(tilePaddingIt != metadata.end() && tilePaddingIt->type() == oiio::TypeDesc::INT)
            tileParams.padding = tilePaddingIt->get_int();
    }

    // invalid or no tile metadata
    if((tileParams.width <= 0) || (tileParams.height <= 0) || (tileParams.padding <= 0))
    {
        ALICEVISION_THROW_ERROR("Cannot find tile information in file: " << depthMapPath);
    }

    // get tile border size
    const int tileWidth = tileParams.width / downscale;
    const int tileHeight = tileParams.height / downscale;
    const int tilePadding = tileParams.padding / downscale;

    // get tile position information
    const bool firstColumn = (tileRoi.x.begin == 0);
    const bool lastColumn = (tileRoi.x.end == _mp.getOriginalWidth(_rc));
    const bool firstRow = (tileRoi.y.begin == 0);
    const bool lastRow = (tileRoi.y.end == _mp.getOriginalHeight(_rc));

    // weight the top left corner
    if(!firstColumn || !firstRow)
    {
        const Point2d lu(0, 0);
        const int b = (firstRow) ? 1 : 0;
        const int d = (firstColumn) ? 1 : 0;
        weightTileBorder(0, b, 1, d, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the bottom left corner
    if(!firstColumn || !lastRow)
    {
        const Point2d lu(0, tileHeight - tilePadding);
        const int a = (firstColumn) ? 1 : 0;
        const int c = (lastRow) ? 1 : 0;
        weightTileBorder(a, 1, c, 0, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the top right corner
    if(!lastColumn || !firstRow)
    {
        const Point2d lu(tileWidth - tilePadding, 0);
        const int a = (firstRow) ? 1 : 0;
        const int c = (lastColumn) ? 1 : 0;
        weightTileBorder(a, 0, c, 1, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the bottom right corner
    if(!lastColumn || !lastRow)
    {
        const Point2d lu(tileWidth - tilePadding, tileHeight - tilePadding);
        const int b = (lastColumn) ? 1 : 0;
        const int d = (lastRow) ? 1 : 0;
        weightTileBorder(1, b, 0, d, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the top border
    if(!firstRow)
    {
        const Point2d lu(tilePadding, 0);
        weightTileBorder(0, 0, 1, 1, currentTileWidth, currentTileHeight, tileWidth - 2 * tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the bottom border
    if(!lastRow)
    {
        const Point2d lu(tilePadding, tileHeight - tilePadding);
        weightTileBorder(1, 1, 0, 0, currentTileWidth, currentTileHeight, tileWidth - 2 * tilePadding, tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the left border
    if(!firstColumn)
    {
        const Point2d lu(0, tilePadding);
        weightTileBorder(0, 1, 1, 0, currentTileWidth, currentTileHeight, tilePadding, tileHeight - 2 * tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // weight the right border
    if(!lastColumn)
    {
        const Point2d lu(tileWidth - tilePadding, tilePadding);
        weightTileBorder(1, 0, 0, 1, currentTileWidth, currentTileHeight, tilePadding, tileHeight - 2 * tilePadding, lu, tileDepthMap, tileSimMap);
    }

    // add weighted tile to the depth/sim map
    for(int x = downscaledRoi.x.begin; x < downscaledRoi.x.end; ++x)
    {
        for(int y = downscaledRoi.y.begin; y < downscaledRoi.y.end; ++y)
        {
            const int tx = x - downscaledRoi.x.begin;
            const int ty = y - downscaledRoi.y.begin;

            DepthSim& depthSim = _dsm[y * _width + x];
            depthSim.depth += tileDepthMap[ty * currentTileWidth + tx];
            depthSim.sim += tileSimMap[ty * currentTileWidth + tx];
        }
    }
}

} // namespace depthMap
} // namespace aliceVision
