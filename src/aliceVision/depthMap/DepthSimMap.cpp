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
#include <aliceVision/imageIO/image.hpp>

#include <iostream>

namespace aliceVision {
namespace depthMap {

DepthSimMap::DepthSimMap(int rc, mvsUtils::MultiViewParams& mp, int scale, int step)
    : _scale(scale)
    , _step(step)
    , _mp(mp)
    , _rc(rc)
{
    _w = _mp.getWidth(_rc) / (_scale * _step);
    _h = _mp.getHeight(_rc) / (_scale * _step);
    _dsm.resize_with(_w * _h, DepthSim(-1.0f, 1.0f));
}

DepthSimMap::~DepthSimMap()
{
}

void DepthSimMap::initFromSmaller(const DepthSimMap& other)
{
    if ((_scale * _step) > (other._scale * other._step))
    {
        throw std::runtime_error("Error DepthSimMap: You cannot init from a larger map.");
    }
    double ratio = double(_scale * _step) / double(other._scale * other._step);

    ALICEVISION_LOG_DEBUG("DepthSimMap::initFromSmaller: ratio=" << ratio << ", otherScaleStep=" << other._scale * other._step << ", scaleStep=" << _scale * _step);
    for (int y = 0; y < _h; ++y)
    {
        const int oy = y * ratio;
        for (int x = 0; x < _w; ++x)
        {
            const int ox = x * ratio;
            const DepthSim& otherDepthSim = other._dsm[oy * other._w + ox];
            _dsm[y * _w + x] = otherDepthSim;
        }
    }
}

void DepthSimMap::add11(const DepthSimMap& other)
{
    if ((_scale != 1) || (_step != 1))
    {
        throw std::runtime_error("Error DepthSimMap: You can only add to scale1-step1 map.");
    }

    int k = (other._step * other._scale) / 2;
    int k1 = k;
    if ((other._step * other._scale) % 2 == 0)
        k -= 1;

    for (int i = 0; i < other._dsm.size(); i++)
    {
        int x = (i % other._w) * other._step * other._scale;
        int y = (i / other._w) * other._step * other._scale;
        DepthSim depthSim = other._dsm[i];

        if (depthSim.depth > -1.0f)
        {

            bool isBest = true;
            for (int yp = y - k; yp <= y + k1; yp++)
            {
                for (int xp = x - k; xp <= x + k1; xp++)
                {
                    if ((xp >= 0) && (xp < _w) && (yp >= 0) && (yp < _h) && // check image borders
                        (depthSim.sim > _dsm[yp * _w + xp].sim))
                    {
                        isBest = false;
                    }
                }
            }

            if (isBest)
            {
                for (int yp = y - k; yp <= y + k1; yp++)
                {
                    for (int xp = x - k; xp <= x + k1; xp++)
                    {
                        if ((xp >= 0) && (xp < _w) && (yp >= 0) && (yp < _h))
                        {
                            _dsm[yp * _w + xp] = depthSim;
                        }
                    }
                }
            }
        }
    }
}

void DepthSimMap::add(const DepthSimMap& other)
{
    if ((_scale != other._scale) || (_step != other._step))
    {
        throw std::runtime_error("Error DepthSimMap: You can only add to the same _scale and step map.");
    }

    for (int i = 0; i < _dsm.size(); i++)
    {
        const DepthSim& depthSim1 = _dsm[i];
        const DepthSim& depthSim2 = other._dsm[i];

        if ((depthSim2.depth > -1.0f) && (depthSim2.sim < depthSim1.sim))
        {
            _dsm[i] = depthSim2;
        }
    }
}

Point2d DepthSimMap::getMaxMinDepth() const
{
    float maxDepth = -1.0f;
    float minDepth = std::numeric_limits<float>::max();
    for (int j = 0; j < _w * _h; j++)
    {
        if (_dsm[j].depth > -1.0f)
        {
            maxDepth = std::max(maxDepth, _dsm[j].depth);
            minDepth = std::min(minDepth, _dsm[j].depth);
        }
    }
    return Point2d(maxDepth, minDepth);
}

Point2d DepthSimMap::getMaxMinSim() const
{
    float maxSim = -1.0f;
    float minSim = std::numeric_limits<float>::max();
    for (int j = 0; j < _w * _h; j++)
    {
        if (_dsm[j].sim > -1.0f)
        {
            maxSim = std::max(maxSim, _dsm[j].sim);
            minSim = std::min(minSim, _dsm[j].sim);
        }
    }
    return Point2d(maxSim, minSim);
}

float DepthSimMap::getPercentileDepth(float perc) const
{
    int step = std::max(1, (_w * _h) / 50000);
    int n = (_w * _h) / std::max(1, (step - 1));
    StaticVector<float> depths;
    depths.reserve(n);

    for (int j = 0; j < _w * _h; j += step)
    {
        if (_dsm[j].depth > -1.0f)
        {
            depths.push_back(_dsm[j].depth);
        }
    }

    qsort(&depths[0], depths.size(), sizeof(float), qSortCompareFloatAsc);

    float out = depths[(float)((float)depths.size() * perc)];

    return out;
}

void DepthSimMap::getDepthMap(StaticVector<float>& out_depthMap) const
{
    // Create a depth map at the size of our input image
    out_depthMap.resize(_w * _h);

    for (int y = 0; y < _h; ++y)
    {
        for (int x = 0; x < _w; ++x)
        {
            int xyoffset = y * _w + x;
            out_depthMap[xyoffset] = _dsm[xyoffset].depth;
        }
    }
}

void DepthSimMap::getSimMap(StaticVector<float>& out_simMap) const
{
    // Create a depth map at the size of our input image
    out_simMap.resize(_w * _h);

    for (int y = 0; y < _h; ++y)
    {
        for (int x = 0; x < _w; ++x)
        {
            int xyoffset = y * _w + x;
            out_simMap[xyoffset] = _dsm[xyoffset].sim;
        }
    }
}

void DepthSimMap::getDepthMapXPart(StaticVector<float>& out_depthMap, int xFrom, int partW)
{
    out_depthMap.resize_with(partW * _h, -1.0f);
    for (int yp = 0; yp < _h; yp++)
    {
        for (int xp = xFrom; xp < xFrom + partW; xp++)
        {
            float depth = _dsm[yp * _w + xp].depth;
            out_depthMap[yp * partW + (xp - xFrom)] = depth;
        }
    }
}

void DepthSimMap::getSimMapXPart(StaticVector<float>& out_simMap, int xFrom, int partW)
{
    out_simMap.resize_with(partW * _h, -1.0f);
    for (int yp = 0; yp < _h; yp++)
    {
        for (int xp = xFrom; xp < xFrom + partW; xp++)
        {
            float sim = _dsm[yp * _w + xp].sim;
            out_simMap[yp * partW + (xp - xFrom)] = sim;
        }
    }
}

void DepthSimMap::initJustFromDepthMap(const StaticVector<float>& depthMap, float defaultSim)
{
    int wdm = _mp.getWidth(_rc) / _scale;

    for (int i = 0; i < _dsm.size(); i++)
    {
        int x = (i % _w) * _step;
        int y = (i / _w) * _step;
        if ((x < _w) && (y < _h))
        {
            _dsm[i].depth = depthMap[y * wdm + x];
            _dsm[i].sim = defaultSim;
        }
    }
}

void DepthSimMap::initJustFromDepthMap(const DepthSimMap& depthSimMap, float defaultSim)
{
    if (depthSimMap._w != _w || depthSimMap._h != _h)
        throw std::runtime_error("DepthSimMap:initJustFromDepthMap: Error input depth map is not at the same size.");

    for (int y = 0; y < _h; ++y)
    {
        for (int x = 0; x < _w; ++x)
        {
            DepthSim& ds = _dsm[y * _w + x];
            ds.depth = depthSimMap._dsm[y * depthSimMap._w + x].depth;
            ds.sim = defaultSim;
        }
    }
}

void DepthSimMap::initFromDepthMapAndSimMap(StaticVector<float>* depthMapT, StaticVector<float>* simMapT,
    int depthSimMapsScale)
{
    int wdm = _mp.getWidth(_rc) / depthSimMapsScale;
    int hdm = _mp.getHeight(_rc) / depthSimMapsScale;

    for (int i = 0; i < _dsm.size(); i++)
    {
        int x = (((i % _w) * _step) * _scale) / depthSimMapsScale;
        int y = (((i / _w) * _step) * _scale) / depthSimMapsScale;
        if ((x < wdm) && (y < hdm))
        {
            int index = y * wdm + x;
            _dsm[i].depth = (*depthMapT)[index];
            _dsm[i].sim = (*simMapT)[index];
        }
    }
}

void DepthSimMap::saveToImage(const std::string& filename, float simThr) const
{
    const int bufferWidth = 2 * _w;
    std::vector<Color> colorBuffer(bufferWidth * _h);

    try
    {
        Point2d maxMinDepth;
        maxMinDepth.x = getPercentileDepth(0.9) * 1.1;
        maxMinDepth.y = getPercentileDepth(0.01) * 0.8;

        Point2d maxMinSim = Point2d(simThr, -1.0f);
        if (simThr < -1.0f)
        {
            Point2d autoMaxMinSim = getMaxMinSim();
            // only use it if the default range is valid
            if (std::abs(autoMaxMinSim.x - autoMaxMinSim.y) > std::numeric_limits<float>::epsilon())
                maxMinSim = autoMaxMinSim;

            if (_mp.verbose)
                ALICEVISION_LOG_DEBUG("saveToImage: max : " << maxMinSim.x << ", min: " << maxMinSim.y);
        }

        for (int y = 0; y < _h; y++)
        {
            for (int x = 0; x < _w; x++)
            {
                const DepthSim& depthSim = _dsm[y * _w + x];
                float depth = (depthSim.depth - maxMinDepth.y) / (maxMinDepth.x - maxMinDepth.y);
                colorBuffer.at(y * bufferWidth + x) = getColorFromJetColorMap(depth);

                float sim = (depthSim.sim - maxMinSim.y) / (maxMinSim.x - maxMinSim.y);
                colorBuffer.at(y * bufferWidth + _w + x) = getColorFromJetColorMap(sim);
            }
        }

        imageIO::writeImage(filename, bufferWidth, _h, colorBuffer, imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::AUTO);
    }
    catch (...)
    {
        ALICEVISION_LOG_ERROR("Failed to save '" << filename << "' (simThr: " << simThr << ")");
    }
}

void DepthSimMap::save(const std::string& customSuffix) const
{
    StaticVector<float> depthMap;
    getDepthMap(depthMap);
    StaticVector<float> simMap;
    getSimMap(simMap);

    double s = _scale * _step;
    oiio::ParamValueList metadata = imageIO::getMetadataFromMap(_mp.getMetadata(_rc));
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", _mp.getDownscaleFactor(_rc) * _scale * _step));

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

    metadata.push_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, C.m));
    metadata.push_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, iP.m));

    {
        const Point2d maxMinDepth = getMaxMinDepth();
        metadata.push_back(oiio::ParamValue("AliceVision:minDepth", static_cast<float>(maxMinDepth.y)));
        metadata.push_back(oiio::ParamValue("AliceVision:maxDepth", static_cast<float>(maxMinDepth.x)));
    }

    {
        std::vector<double> matrixP = _mp.getOriginalP(_rc);
        metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, matrixP.data()));
    }

    imageIO::writeImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, _scale, customSuffix), _w, _h, depthMap.getDataWritable(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::NO_CONVERSION, metadata);
    imageIO::writeImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, _scale, customSuffix), _w, _h, simMap.getDataWritable(), imageIO::EImageQuality::OPTIMIZED, imageIO::EImageColorSpace::NO_CONVERSION, metadata);
}

void DepthSimMap::load(int fromScale)
{
    int width, height;

    StaticVector<float> depthMap;
    StaticVector<float> simMap;

    imageIO::readImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, fromScale), width, height, depthMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);
    imageIO::readImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, fromScale), width, height, simMap.getDataWritable(), imageIO::EImageColorSpace::NO_CONVERSION);

    initFromDepthMapAndSimMap(&depthMap, &simMap, fromScale);
}

} // namespace depthMap
} // namespace aliceVision
