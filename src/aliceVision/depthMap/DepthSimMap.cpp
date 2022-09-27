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

#include <iostream>

#define ALICEVISION_DEPTHMAP_UPSCALE_NEAREST_NEIGHBOR


namespace aliceVision {
namespace depthMap {

DepthSimMap::DepthSimMap(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step)
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

    ALICEVISION_LOG_DEBUG("DepthSimMap::initFromSmaller: ratio=" << ratio << ", otherScaleStep=" << other._scale * other._step << ", scaleStep=" << _scale * _step);
    for (int y = 0; y < _h; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;
        for (int x = 0; x < _w; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;
            const DepthSim otherDepthSim = getPixelValueInterpolated(other._dsm, ox, oy, other._w, other._h);
            _dsm[y * _w + x] = otherDepthSim;
        }
    }
}

void DepthSimMap::init(const DepthSimMap& other)
{
    if ((_scale != other._scale) || (_step != other._step))
    {
        throw std::runtime_error("Error DepthSimMap: You can only add to the same _scale and step map.");
    }

    for (int i = 0; i < _dsm.size(); i++)
    {
        _dsm[i] = other._dsm[i];
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

/**
* @brief Get depth map at the size of our input image (with _scale applied)
*        from an internal buffer only computed for a subpart (based on the step).
*/
void DepthSimMap::getDepthMapStep1(StaticVector<float>& out_depthMap) const
{
    // Size of our input image (with _scale applied)
    const int wdm = _mp.getWidth(_rc) / _scale;
    const int hdm = _mp.getHeight(_rc) / _scale;

    // Create a depth map at the size of our input image
    out_depthMap.resize(wdm * hdm);

    const double ratio = 1.0 / double(_step);

    ALICEVISION_LOG_DEBUG("DepthSimMap::getDepthMapStep1: ratio=" << ratio);
    for (int y = 0; y < hdm; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;
        for (int x = 0; x < wdm; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;
            const float depth = getPixelValueInterpolated(_dsm, ox, oy, _w, _h).depth;
            out_depthMap[y * wdm + x] = depth;
        }
    }
}

void DepthSimMap::getSimMapStep1(StaticVector<float>& out_simMap) const
{
    // Size of our input image (with _scale applied)
    const int wdm = _mp.getWidth(_rc) / _scale;
    const int hdm = _mp.getHeight(_rc) / _scale;

    // Create a depth map at the size of our input image
    out_simMap.resize(wdm * hdm);

    const double ratio = 1.0 / double(_step);

    ALICEVISION_LOG_DEBUG("DepthSimMap::getDepthMapStep1: ratio=" << ratio);
    for (int y = 0; y < hdm; ++y)
    {
        const double oy = (double(y) - 0.5) * ratio;
        for (int x = 0; x < wdm; ++x)
        {
            const double ox = (double(x) - 0.5) * ratio;
            const float sim = getPixelValueInterpolated(_dsm, ox, oy, _w, _h).sim;
            out_simMap[y * wdm + x] = sim;
        }
    }
}

void DepthSimMap::getDepthMapStep1XPart(StaticVector<float>& out_depthMap, int xFrom, int partW)
{
    int wdm = _mp.getWidth(_rc) / _scale;
    int hdm = _mp.getHeight(_rc) / _scale;

    out_depthMap.resize_with(wdm * hdm, -1.0f);
    for (int yp = 0; yp < hdm; yp++)
    {
        for (int xp = xFrom; xp < xFrom + partW; xp++)
        {
            int x = xp / _step;
            int y = yp / _step;
            if ((x < _w) && (y < _h))
            {
                float depth = _dsm[y * _w + x].depth;
                out_depthMap[yp * partW + (xp - xFrom)] = depth;
            }
        }
    }
}

void DepthSimMap::getSimMapStep1XPart(StaticVector<float>& out_simMap, int xFrom, int partW)
{
    int wdm = _mp.getWidth(_rc) / _scale;
    int hdm = _mp.getHeight(_rc) / _scale;

    out_simMap.resize_with(wdm * hdm, -1.0f);
    for (int yp = 0; yp < hdm; yp++)
    {
        for (int xp = xFrom; xp < xFrom + partW; xp++)
        {
            int x = xp / _step;
            int y = yp / _step;
            if ((x < _w) && (y < _h))
            {
                float sim = _dsm[y * _w + x].sim;
                out_simMap[yp * partW + (xp - xFrom)] = sim;
            }
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

void DepthSimMap::getDepthMap(StaticVector<float>& out_depthMap) const
{
    out_depthMap.resize(_dsm.size());
    for (int i = 0; i < _dsm.size(); i++)
    {
        out_depthMap[i] = _dsm[i].depth;
    }
}

void DepthSimMap::getSimMap(StaticVector<float>& out_simMap) const
{
    out_simMap.resize(_dsm.size());
    for (int i = 0; i < _dsm.size(); i++)
    {
        out_simMap[i] = _dsm[i].sim;
    }
}

void DepthSimMap::saveToImage(const std::string& filename, float simThr) const
{
    const int bufferWidth = 2 * _w;
    std::vector<ColorRGBf> colorBuffer(bufferWidth * _h);

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

        oiio::ParamValueList metadata;
        using namespace imageIO;
        writeImage(filename, bufferWidth, _h, colorBuffer, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);
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
    if (useStep1)
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

    const int width = _mp.getWidth(_rc) / scaleStep;
    const int height = _mp.getHeight(_rc) / scaleStep;

    oiio::ParamValueList metadata = imageIO::getMetadataFromMap(_mp.getMetadata(_rc));
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", _mp.getDownscaleFactor(_rc) * scaleStep));

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

    const int nbDepthValues = std::count_if(depthMap.begin(), depthMap.end(), [](float v) { return v > 0.0f; });
    metadata.push_back(oiio::ParamValue("AliceVision:nbDepthValues", oiio::TypeDesc::INT32, 1, &nbDepthValues));

    using namespace imageIO;
    writeImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, _scale, customSuffix), width, height, depthMap.getDataWritable(), EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);
    writeImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, _scale, customSuffix), width, height, simMap.getDataWritable(), imageIO::EImageQuality::OPTIMIZED, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata);
}

void DepthSimMap::load(int fromScale)
{
    int width, height;

    StaticVector<float> depthMap;
    StaticVector<float> simMap;

    using namespace imageIO;
    readImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::depthMap, fromScale), width, height, depthMap.getDataWritable(), EImageColorSpace::NO_CONVERSION);
    readImage(getFileNameFromIndex(_mp, _rc, mvsUtils::EFileType::simMap, fromScale), width, height, simMap.getDataWritable(), EImageColorSpace::NO_CONVERSION);

    initFromDepthMapAndSimMap(&depthMap, &simMap, fromScale);
}

} // namespace depthMap
} // namespace aliceVision
