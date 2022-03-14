// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthSimMapIO.hpp"

#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/TileParams.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace mvsUtils {

void getTileParamsFromMetadata(const std::string& mapFirstTilePath, TileParams& tileParams) 
{
    oiio::ParamValueList metadata;
    imageIO::readImageMetadata(mapFirstTilePath, metadata);

    const auto tileWidthIt = metadata.find("AliceVision:tileWidth");
    const auto tileHeightIt = metadata.find("AliceVision:tileHeight");
    const auto tilePaddingIt = metadata.find("AliceVision:tilePadding");

    if(tileWidthIt != metadata.end() && tileWidthIt->type() == oiio::TypeDesc::INT)
        tileParams.width = tileWidthIt->get_int();

    if(tileHeightIt != metadata.end() && tileHeightIt->type() == oiio::TypeDesc::INT)
        tileParams.height = tileHeightIt->get_int();

    if(tilePaddingIt != metadata.end() && tilePaddingIt->type() == oiio::TypeDesc::INT)
        tileParams.padding = tilePaddingIt->get_int();
    
    // invalid or no tile metadata
    if((tileParams.width <= 0) || (tileParams.height <= 0) || (tileParams.padding < 0))
    {
        ALICEVISION_THROW_ERROR("Cannot find tile information in file: " << mapFirstTilePath);
    }
}

void weightTileBorder(int a, int b, int c, int d, 
                      int currentTileWidth, 
                      int currentTileHeight,
                      int borderWidth, 
                      int borderHeight,  
                      const Point2d& lu, 
                      std::vector<float>& in_tileMap)
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
            in_tileMap[y * currentTileWidth + x] *= weight;
        }
    }
}

void readTileMapWeighted(int rc,
                         const MultiViewParams& mp, 
                         const TileParams& tileParams,
                         const ROI& roi, 
                         const std::string& tilePath,
                         int downscale,
                         std::vector<float>& inout_map)
{
    // get downscaled ROI
    const ROI downscaledRoi = downscaleROI(roi, downscale);
    const int currentTileWidth = downscaledRoi.width();
    const int currentTileHeight = downscaledRoi.height();

    std::vector<float> tileMap(currentTileWidth * currentTileHeight);

    // read tile depth/sim map
    {
        using namespace imageIO;
        int w, h;
        readImage(tilePath, w, h, tileMap, EImageColorSpace::NO_CONVERSION);
    }

    // get tile border size
    const int tileWidth = tileParams.width / downscale;
    const int tileHeight = tileParams.height / downscale;
    const int tilePadding = tileParams.padding / downscale;

    // get tile position information
    const bool firstColumn = (roi.x.begin == 0);
    const bool lastColumn = (roi.x.end == mp.getOriginalWidth(rc));
    const bool firstRow = (roi.y.begin == 0);
    const bool lastRow = (roi.y.end == mp.getOriginalHeight(rc));

    // weight the top left corner
    if(!firstColumn || !firstRow)
    {
        const Point2d lu(0, 0);
        const int b = (firstRow) ? 1 : 0;
        const int d = (firstColumn) ? 1 : 0;
        weightTileBorder(0, b, 1, d, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileMap);
    }

    // weight the bottom left corner
    if(!firstColumn || !lastRow)
    {
        const Point2d lu(0, tileHeight - tilePadding);
        const int a = (firstColumn) ? 1 : 0;
        const int c = (lastRow) ? 1 : 0;
        weightTileBorder(a, 1, c, 0, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileMap);
    }

    // weight the top right corner
    if(!lastColumn || !firstRow)
    {
        const Point2d lu(tileWidth - tilePadding, 0);
        const int a = (firstRow) ? 1 : 0;
        const int c = (lastColumn) ? 1 : 0;
        weightTileBorder(a, 0, c, 1, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileMap);
    }

    // weight the bottom right corner
    if(!lastColumn || !lastRow)
    {
        const Point2d lu(tileWidth - tilePadding, tileHeight - tilePadding);
        const int b = (lastColumn) ? 1 : 0;
        const int d = (lastRow) ? 1 : 0;
        weightTileBorder(1, b, 0, d, currentTileWidth, currentTileHeight, tilePadding, tilePadding, lu, tileMap);
    }

    // weight the top border
    if(!firstRow)
    {
        const Point2d lu(tilePadding, 0);
        weightTileBorder(0, 0, 1, 1, currentTileWidth, currentTileHeight, tileWidth - 2 * tilePadding, tilePadding, lu, tileMap);
    }

    // weight the bottom border
    if(!lastRow)
    {
        const Point2d lu(tilePadding, tileHeight - tilePadding);
        weightTileBorder(1, 1, 0, 0, currentTileWidth, currentTileHeight, tileWidth - 2 * tilePadding, tilePadding, lu, tileMap);
    }

    // weight the left border
    if(!firstColumn)
    {
        const Point2d lu(0, tilePadding);
        weightTileBorder(0, 1, 1, 0, currentTileWidth, currentTileHeight, tilePadding, tileHeight - 2 * tilePadding, lu, tileMap);
    }

    // weight the right border
    if(!lastColumn)
    {
        const Point2d lu(tileWidth - tilePadding, tilePadding);
        weightTileBorder(1, 0, 0, 1, currentTileWidth, currentTileHeight, tilePadding, tileHeight - 2 * tilePadding, lu, tileMap);
    }

    // add weighted tile to the depth/sim map
    const int mapWidth = std::ceil(mp.getOriginalWidth(rc) / float(downscale));

    for(int x = downscaledRoi.x.begin; x < downscaledRoi.x.end; ++x)
    {
        for(int y = downscaledRoi.y.begin; y < downscaledRoi.y.end; ++y)
        {
            const int tx = x - downscaledRoi.x.begin;
            const int ty = y - downscaledRoi.y.begin;

            inout_map[y * mapWidth + x] += tileMap[ty * currentTileWidth + tx];
        }
    }
}

void readMapFromTiles(int rc, 
                      const MultiViewParams& mp, 
                      EFileType fileType,
                      std::vector<float>& out_map, 
                      int scale,
                      int step, 
                      const std::string& customSuffix)
{
    const int downscale = mp.getProcessDownscale() * std::max(scale, 1) * step; // avoid 0 special case (reserved for depth map filtering)
    const int width = std::ceil(mp.getOriginalWidth(rc) / float(downscale));
    const int height = std::ceil(mp.getOriginalHeight(rc) / float(downscale));

    out_map.resize(width * height, 0.f);

    const std::string mapFirstTilePath = getFileNameFromIndex(mp, rc, fileType, scale, customSuffix, 0, 0);

    if (!fs::exists(mapFirstTilePath))
        ALICEVISION_THROW_ERROR("Cannot find depth/sim map first tile (rc: " << rc << ")");

    // get tile dimensions from metadata
    TileParams tileParams;
    getTileParamsFromMetadata(mapFirstTilePath, tileParams);

    std::vector<ROI> tileList;
    getTileList(tileParams, mp.getOriginalWidth(rc), mp.getOriginalHeight(rc), tileList);

    for(const ROI& roi : tileList)
    {
        const std::string mapTilePath = getFileNameFromIndex(mp, rc, fileType, scale, customSuffix, roi.x.begin, roi.y.begin);
        readTileMapWeighted(rc, mp, tileParams, roi, mapTilePath, downscale, out_map);
    }
}

void writeDepthSimMap(int rc, 
                      const MultiViewParams& mp, 
                      const TileParams& tileParams, 
                      const ROI& roi,
                      const std::vector<float>& depthMap, 
                      const std::vector<float>& simMap, 
                      int scale,
                      int step,
                      const std::string& customSuffix)
{
    const int scaleStep = std::max(scale, 1) * step; // avoid 0 special case (reserved for depth map filtering)
    const int downscale = mp.getDownscaleFactor(rc) * scaleStep;
    const int nbDepthValues = std::count_if(depthMap.begin(), depthMap.end(), [](float v) { return v > 0.0f; });

    oiio::ParamValueList metadata = imageIO::getMetadataFromMap(mp.getMetadata(rc));

    // tile metadata
    if((tileParams.width > 0) && (tileParams.height > 0) && (tileParams.padding >= 0))
    {
        metadata.push_back(oiio::ParamValue("AliceVision:tileWidth", int(tileParams.width)));
        metadata.push_back(oiio::ParamValue("AliceVision:tileHeight", int(tileParams.height)));
        metadata.push_back(oiio::ParamValue("AliceVision:tilePadding", int(tileParams.padding)));    
    }

    // downscale metadata
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", downscale));

    double s = scaleStep;
    Point3d C = mp.CArr[rc];
    Matrix3x3 iP = mp.iCamArr[rc];
    if (s > 1.0)
    {
        Matrix3x4 P = mp.camArr[rc];
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
        float maxDepth = -1.0f;
        float minDepth = std::numeric_limits<float>::max();

        for(const float depth : depthMap)
        {
            if(depth <= -1.0f)
                continue;

            maxDepth = std::max(maxDepth, depth);
            minDepth = std::min(minDepth, depth);
        }
;
        metadata.push_back(oiio::ParamValue("AliceVision:minDepth", oiio::TypeDesc::FLOAT, 1, &minDepth));
        metadata.push_back(oiio::ParamValue("AliceVision:maxDepth", oiio::TypeDesc::FLOAT, 1, &maxDepth));
        metadata.push_back(oiio::ParamValue("AliceVision:nbDepthValues", oiio::TypeDesc::INT32, 1, &nbDepthValues));
    }

    // projection matrix metadata
    {
        std::vector<double> matrixP = mp.getOriginalP(rc);
        metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, matrixP.data()));
    }

    // get full image dimensions
    const ROI downscaledROI = downscaleROI(roi, downscale);
    const int imageWidth = std::ceil(mp.getOriginalWidth(rc) / float(downscale));
    const int imageHeight = std::ceil(mp.getOriginalHeight(rc) / float(downscale));

    oiio::ROI imageROI = oiio::ROI::All();
    std::string depthMapPath;
    std::string simMapPath;

    if(downscaledROI.width() != imageWidth || downscaledROI.height() != imageHeight)
    {
        // tiled depth/sim map
        imageROI = oiio::ROI(downscaledROI.x.begin, downscaledROI.x.end, downscaledROI.y.begin, downscaledROI.y.end, 0, 1, 0, 1);
;       depthMapPath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix, roi.x.begin, roi.y.begin);
        simMapPath = getFileNameFromIndex(mp, rc, EFileType::simMap, scale, customSuffix, roi.x.begin, roi.y.begin);
    }
    else
    {
        // fullsize depth/sim map
        depthMapPath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix);
        simMapPath = getFileNameFromIndex(mp, rc, EFileType::simMap, scale, customSuffix);
    }

    // write depth/sim map
    using namespace imageIO;
    if(!depthMap.empty())
      writeImage(depthMapPath, imageWidth, imageHeight, depthMap, EImageQuality::LOSSLESS, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata, imageROI);
    if(!simMap.empty())
      writeImage(simMapPath, imageWidth, imageHeight, simMap, imageIO::EImageQuality::OPTIMIZED, OutputFileColorSpace(EImageColorSpace::NO_CONVERSION), metadata, imageROI);
}

void writeDepthSimMap(int rc, 
                      const MultiViewParams& mp,
                      const std::vector<float>& depthMap, 
                      const std::vector<float>& simMap, 
                      int scale,
                      int step,
                      const std::string& customSuffix)
{
    const TileParams tileParams; // default tile parameters, no tiles
    const ROI roi = ROI(0, mp.getOriginalWidth(rc), 0, mp.getOriginalHeight(rc)); // full roi
    writeDepthSimMap(rc, mp, tileParams, roi, depthMap, simMap, scale, step, customSuffix);
}


void writeDepthMap(int rc, 
                   const MultiViewParams& mp,
                   const std::vector<float>& depthMap, 
                   int scale,
                   int step,
                   const std::string& customSuffix)
{
    const TileParams tileParams;  // default tile parameters, no tiles
    const ROI roi = ROI(0, mp.getOriginalWidth(rc), 0, mp.getOriginalHeight(rc)); // full roi
    std::vector<float> simMap(0); // empty simMap, write only depth map
    writeDepthSimMap(rc, mp, tileParams, roi, depthMap, simMap, scale, step, customSuffix);
}

void readDepthSimMap(int rc, 
                     const MultiViewParams& mp,
                     std::vector<float>& out_depthMap, 
                     std::vector<float>& out_simMap, 
                     int scale,
                     int step, 
                     const std::string& customSuffix)
{
    const std::string depthMapPath = getFileNameFromIndex(mp, rc,EFileType::depthMap, scale, customSuffix);
    const std::string simMapPath = getFileNameFromIndex(mp, rc, EFileType::simMap, scale, customSuffix);

    if (fs::exists(depthMapPath) && fs::exists(simMapPath))
    {
        using namespace imageIO;
        int w, h;
        readImage(depthMapPath, w, h, out_depthMap, EImageColorSpace::NO_CONVERSION);
        readImage(simMapPath, w, h, out_simMap, EImageColorSpace::NO_CONVERSION);
    }
    else
    {
        readMapFromTiles(rc, mp, EFileType::depthMap, out_depthMap, scale, step, customSuffix);
        readMapFromTiles(rc, mp, EFileType::simMap, out_simMap, scale, step, customSuffix);
    }
}

void readDepthMap(int rc, 
                  const MultiViewParams& mp,
                  std::vector<float>& out_depthMap, 
                  int scale,
                  int step,
                  const std::string& customSuffix)
{
    const std::string depthMapPath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix);
        
    if (fs::exists(depthMapPath))
    {
        using namespace imageIO;
        int w, h;
        readImage(depthMapPath, w, h, out_depthMap, EImageColorSpace::NO_CONVERSION);
    }
    else
    {
        readMapFromTiles(rc, mp, EFileType::depthMap, out_depthMap, scale, step, customSuffix);
    }
}

void readSimMap(int rc, const MultiViewParams& mp, std::vector<float>& out_simMap, int scale, int step, const std::string& customSuffix)
{
    const std::string simMapPath = getFileNameFromIndex(mp, rc, EFileType::simMap, scale, customSuffix);

    if (fs::exists(simMapPath))
    {
        using namespace imageIO;
        int w, h;
        readImage(simMapPath, w, h, out_simMap, EImageColorSpace::NO_CONVERSION);
    }
    else
    {
        readMapFromTiles(rc, mp, EFileType::simMap, out_simMap, scale, step, customSuffix);
    }
}

unsigned long getNbDepthValuesFromDepthMap(int rc, 
                                           const MultiViewParams& mp,
                                           int scale,
                                           int step,
                                           const std::string& customSuffix)
{
    const std::string depthMapPath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix);
    int nbDepthValues = -1;

    // get nbDepthValues from metadata
    if (fs::exists(depthMapPath)) // untilled
    {
        oiio::ParamValueList metadata;
        imageIO::readImageMetadata(depthMapPath, metadata);
        nbDepthValues = metadata.get_int("AliceVision:nbDepthValues", -1);
    }
    else // tilled
    {
        const std::string depthMapFirstTilePath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix, 0, 0);

        if (!fs::exists(depthMapFirstTilePath))
            ALICEVISION_THROW_ERROR("Cannot find depth map first tile (rc: " << rc << ")");

        // get tile dimensions from metadata
        TileParams tileParams;
        getTileParamsFromMetadata(depthMapFirstTilePath, tileParams);

        std::vector<ROI> tileList;
        getTileList(tileParams, mp.getOriginalWidth(rc), mp.getOriginalHeight(rc), tileList);

        for(const ROI& roi : tileList)
        {
            oiio::ParamValueList metadata;
            const std::string mapTilePath = getFileNameFromIndex(mp, rc, EFileType::depthMap, scale, customSuffix, roi.x.begin, roi.y.begin);
            imageIO::readImageMetadata(depthMapPath, metadata);
            const int nbTileDepthValues = metadata.get_int("AliceVision:nbDepthValues", -1);

            if(nbTileDepthValues < 0)
                ALICEVISION_THROW_ERROR("Cannot find or incorrect 'AliceVision:nbDepthValues' metadata in depth map tile (rc: " << rc << ")");

            nbDepthValues += nbTileDepthValues;
        }
    }

    // no metadata compute number of depth values
    if(nbDepthValues < 0)
    {
        std::vector<float> depthMap;

        ALICEVISION_LOG_WARNING("Can't find or invalid 'nbDepthValues' metadata in depth map (rc: " << rc << "). Recompute the number of valid values.");

        readDepthMap(rc, mp, depthMap, scale, step, customSuffix);

        for(int i = 0; i < depthMap.size(); ++i)
            nbDepthValues += static_cast<unsigned long>(depthMap[i] > 0.0f);
    }

    return nbDepthValues;
}

} // namespace mvsUtils
} // namespace aliceVision
