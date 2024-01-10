// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "mapIO.hpp"

#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/regex.hpp>

#include <filesystem>

namespace fs = std::filesystem;

namespace aliceVision {
namespace mvsUtils {

/**
 * @brief Get the map name from the given fileType enum
 * @param[in] fileType the map fileType enum
 * @return map type string
 */
std::string getMapNameFromFileType(EFileType fileType)
{
    switch (fileType)
    {
        case EFileType::depthMap:
            return "depth map";
            break;
        case EFileType::depthMapFiltered:
            return "filtered depth map";
            break;
        case EFileType::simMap:
            return "similarity map";
            break;
        case EFileType::simMapFiltered:
            return "filtered similarity map";
            break;
        case EFileType::normalMap:
            return "normal map";
            break;
        case EFileType::normalMapFiltered:
            return "filtered normal map";
            break;
        case EFileType::thicknessMap:
            return "thickness map";
            break;
        case EFileType::pixSizeMap:
            return "pixSize map";
            break;
        default:
            break;
    }
    return "unknown map";
}

/**
 * @brief Get tile map ROI from file metadata.
 * @param[in] mapTilePath the tile map file path
 * @param[in,out] out_roi the corresponding region-of-interest read from file metadata
 */
void getRoiFromMetadata(const std::string& mapTilePath, ROI& out_roi)
{
    const oiio::ParamValueList metadata = image::readImageMetadata(mapTilePath);

    const auto roiBeginXIt = metadata.find("AliceVision:roiBeginX");
    const auto roiBeginYIt = metadata.find("AliceVision:roiBeginY");
    const auto roiEndXIt = metadata.find("AliceVision:roiEndX");
    const auto roiEndYIt = metadata.find("AliceVision:roiEndY");

    if (roiBeginXIt != metadata.end() && roiBeginXIt->type() == oiio::TypeDesc::INT)
        out_roi.x.begin = roiBeginXIt->get_int();

    if (roiBeginYIt != metadata.end() && roiBeginYIt->type() == oiio::TypeDesc::INT)
        out_roi.y.begin = roiBeginYIt->get_int();

    if (roiEndXIt != metadata.end() && roiEndXIt->type() == oiio::TypeDesc::INT)
        out_roi.x.end = roiEndXIt->get_int();

    if (roiEndYIt != metadata.end() && roiEndYIt->type() == oiio::TypeDesc::INT)
        out_roi.y.end = roiEndYIt->get_int();

    // invalid or no roi metadata
    if ((out_roi.x.begin < 0) || (out_roi.y.begin < 0) || (out_roi.x.end <= 0) || (out_roi.y.end <= 0))
    {
        ALICEVISION_THROW_ERROR("Cannot find ROI information in file: " << fs::path(mapTilePath).filename().string());
    }
}

/**
 * @brief Get tile map TileParams from file metadata.
 * @param[in] mapTilePath the tile map file path
 * @param[in,out] out_tileParams the corresponding TileParams read from file metadata
 */
void getTileParamsFromMetadata(const std::string& mapTilePath, TileParams& out_tileParams)
{
    const oiio::ParamValueList metadata = image::readImageMetadata(mapTilePath);

    const auto tileWidthIt = metadata.find("AliceVision:tileBufferWidth");
    const auto tileHeightIt = metadata.find("AliceVision:tileBufferHeight");
    const auto tilePaddingIt = metadata.find("AliceVision:tilePadding");

    if (tileWidthIt != metadata.end() && tileWidthIt->type() == oiio::TypeDesc::INT)
        out_tileParams.bufferWidth = tileWidthIt->get_int();

    if (tileHeightIt != metadata.end() && tileHeightIt->type() == oiio::TypeDesc::INT)
        out_tileParams.bufferHeight = tileHeightIt->get_int();

    if (tilePaddingIt != metadata.end() && tilePaddingIt->type() == oiio::TypeDesc::INT)
        out_tileParams.padding = tilePaddingIt->get_int();

    // invalid or no tile metadata
    if ((out_tileParams.bufferWidth <= 0) || (out_tileParams.bufferHeight <= 0) || (out_tileParams.padding < 0))
    {
        ALICEVISION_THROW_ERROR("Cannot find tile parameters in file: " << fs::path(mapTilePath).filename().string());
    }
}

/**
 * @brief Get the tile map path list for a R camera et a given scale / stepXY.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] fileType the map fileType enum
 * @param[in] customSuffix the map filename custom suffix
 * @param[in,out] out_mapTilePathList the tile map path list
 */
void getTilePathList(int rc,
                     const MultiViewParams& mp,
                     const EFileType fileType,
                     const std::string& customSuffix,
                     std::vector<std::string>& out_mapTilePathList)
{
    const fs::path mapPath(getFileNameFromIndex(mp, rc, fileType, customSuffix));
    const fs::path mapDirectory(mapPath.parent_path());

    if (!is_directory(mapDirectory))
        ALICEVISION_THROW_ERROR("Cannot find " << getMapNameFromFileType(fileType) << " directory (rc: " << rc << ").");

    const boost::regex mapPattern(mapPath.stem().string() + "_\\d+_\\d+" + mapPath.extension().string());

    for (auto& entry : fs::directory_iterator{mapDirectory})
    {
        if (boost::regex_match(entry.path().filename().string(), mapPattern))
            out_mapTilePathList.push_back(entry.path().string());
    }
}

/**
 * @brief Weight one of the corners/edges of a tile according to the size of the padding.
 *
 * When merging tiles, there are 8 intersection areas:
 *  * 4 corners (intersection of 4 tiles or 2 tiles when the tile is on one image edge)
 *  * 4 edges (intersection of 2 tiles)
 *
 * @param a alpha for top-left
 * @param b alpha for top-right
 * @param c alpha for bottom-right
 * @param d alpha for bottom-left
 * @param borderWidth tiles intersection area width (could be the intersection between 2 or 4 tiles)
 * @param borderHeight tiles intersection area height
 * @param lu left-up corner of the intersection area in the tile coordinate system
 * @param in_tileMap image of the tile
 */
template<typename T>
void weightTileBorder(int a, int b, int c, int d, int borderWidth, int borderHeight, const Point2d& lu, image::Image<T>& in_tileMap)
{
    const Point2d rd = lu + Point2d(borderWidth, borderHeight);

    const int endX = std::min(int(rd.x), in_tileMap.width());
    const int endY = std::min(int(rd.y), in_tileMap.height());

    // Add small margin where alpha is 0 for corners (lu and rd)
    static const double margin = 2.0;
    const Point2d lu_m(lu.x + margin, lu.y + margin);
    const Point2d rd_m(rd.x - margin, rd.y - margin);
    const double borderWidth_m = borderWidth - 2.0 * margin;
    const double borderHeight_m = borderHeight - 2.0 * margin;

    for (int x = lu.x; x < endX; ++x)
    {
        for (int y = lu.y; y < endY; ++y)
        {
            // bilinear interpolation
            const float r_x = clamp((rd_m.x - x) / borderWidth_m, 0.0, 1.0);
            const float r_y = clamp((rd_m.y - y) / borderHeight_m, 0.0, 1.0);
            const float l_x = clamp((x - lu_m.x) / borderWidth_m, 0.0, 1.0);
            const float l_y = clamp((y - lu_m.y) / borderHeight_m, 0.0, 1.0);

            const float weight = r_y * (r_x * a + l_x * b) + l_y * (r_x * d + l_x * c);

            // apply weight to tile depth/sim map
            in_tileMap(y, x) *= weight;
        }
    }
}

/**
 * @brief Add a single tile to a full map with weighting.
 * @param[in] rc the related R camera index
 * @param[in] mp the multi-view parameters
 * @param[in] tileParams tile workflow parameters
 * @param[in] roi the 2d region of interest without any downscale apply
 * @param[in] downscale the map downscale factor
 * @param[in] in_tileMap the tile map to add
 * @param[in,out] inout_map the full output map
 */
template<typename T>
void addSingleTileMapWeighted(int rc,
                              const MultiViewParams& mp,
                              const TileParams& tileParams,
                              const ROI& roi,
                              int downscale,
                              image::Image<T>& in_tileMap,
                              image::Image<T>& inout_map)
{
    // get downscaled ROI
    const ROI downscaledRoi = downscaleROI(roi, downscale);

    // get tile border size
    const int tileWidth = downscaledRoi.width();
    const int tileHeight = downscaledRoi.height();
    const int tilePadding = tileParams.padding / downscale;

    // get tile position information
    const bool firstColumn = (roi.x.begin == 0);
    const bool lastColumn = (roi.x.end == mp.getWidth(rc));
    const bool firstRow = (roi.y.begin == 0);
    const bool lastRow = (roi.y.end == mp.getHeight(rc));

    // weight the top left corner
    if (!firstColumn || !firstRow)
    {
        const Point2d lu(0, 0);
        const int b = (firstRow) ? 1 : 0;
        const int d = (firstColumn) ? 1 : 0;
        weightTileBorder(0, b, 1, d, tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the bottom left corner
    if (!firstColumn || !lastRow)
    {
        const Point2d lu(0, tileHeight - tilePadding);
        const int a = (firstColumn) ? 1 : 0;
        const int c = (lastRow) ? 1 : 0;
        weightTileBorder(a, 1, c, 0, tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the top right corner
    if (!lastColumn || !firstRow)
    {
        const Point2d lu(tileWidth - tilePadding, 0);
        const int a = (firstRow) ? 1 : 0;
        const int c = (lastColumn) ? 1 : 0;
        weightTileBorder(a, 0, c, 1, tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the bottom right corner
    if (!lastColumn || !lastRow)
    {
        const Point2d lu(tileWidth - tilePadding, tileHeight - tilePadding);
        const int b = (lastColumn) ? 1 : 0;
        const int d = (lastRow) ? 1 : 0;
        weightTileBorder(1, b, 0, d, tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the top border
    if (!firstRow)
    {
        const Point2d lu(tilePadding, 0);
        weightTileBorder(0, 0, 1, 1, tileWidth - 2 * tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the bottom border
    if (!lastRow)
    {
        const Point2d lu(tilePadding, tileHeight - tilePadding);
        weightTileBorder(1, 1, 0, 0, tileWidth - 2 * tilePadding, tilePadding, lu, in_tileMap);
    }

    // weight the left border
    if (!firstColumn)
    {
        const Point2d lu(0, tilePadding);
        weightTileBorder(0, 1, 1, 0, tilePadding, tileHeight - 2 * tilePadding, lu, in_tileMap);
    }

    // weight the right border
    if (!lastColumn)
    {
        const Point2d lu(tileWidth - tilePadding, tilePadding);
        weightTileBorder(1, 0, 0, 1, tilePadding, tileHeight - 2 * tilePadding, lu, in_tileMap);
    }

    // add weighted tile to the depth/sim map
    for (int x = downscaledRoi.x.begin; x < downscaledRoi.x.end; ++x)
    {
        for (int y = downscaledRoi.y.begin; y < downscaledRoi.y.end; ++y)
        {
            const int tx = x - downscaledRoi.x.begin;
            const int ty = y - downscaledRoi.y.begin;

            inout_map(y, x) += in_tileMap(ty, tx);
        }
    }
}

template<typename T>
void readMapFromFileOrTiles(int rc,
                            const MultiViewParams& mp,
                            EFileType fileType,
                            image::Image<T>& out_map,
                            int scale,
                            int step,
                            const std::string& customSuffix)
{
    // assert scale & step
    assert(scale > 0);
    assert(step > 0);

    // single file fullsize map path
    const std::string mapPath = getFileNameFromIndex(mp, rc, fileType, customSuffix);

    // check single file fullsize map exists
    if (fs::exists(mapPath))
    {
        ALICEVISION_LOG_TRACE("Load depth map (full image): " << mapPath << ", scale: " << scale << ", step: " << step);
        // read single file fullsize map
        image::readImage(mapPath, out_map, image::EImageColorSpace::NO_CONVERSION);
        return;
    }
    ALICEVISION_LOG_TRACE("No full image depth map: " << mapPath << ", scale: " << scale << ", step: " << step << ". Looking for tiles.");

    // read map from tiles
    const ROI imageRoi(Range(0, mp.getWidth(rc)), Range(0, mp.getHeight(rc)));

    const int scaleStep = scale * step;
    const int width = divideRoundUp(mp.getWidth(rc), scaleStep);
    const int height = divideRoundUp(mp.getHeight(rc), scaleStep);

    // the output full map
    out_map.resize(width, height, true, T(0.f));  // should be initialized, additive process

    // get tile map path list for the given R camera
    std::vector<std::string> mapTilePathList;
    getTilePathList(rc, mp, fileType, customSuffix, mapTilePathList);

    if (mapTilePathList.empty())
    {
        // map can be empty
        ALICEVISION_LOG_INFO("Cannot find any " << getMapNameFromFileType(fileType) << " tile file (rc: " << rc << ").");
        return;  // nothing to do, already initialized
    }
    else
    {
        ALICEVISION_LOG_TRACE("Load depth map from " << mapTilePathList.size() << " tiles. First tile: " << mapTilePathList[0] << ", scale: " << scale
                                                     << ", step: " << step);
    }

    // get tileParams from first tile file metadata
    TileParams tileParams;
    getTileParamsFromMetadata(mapTilePathList.front(), tileParams);

    // get tile roi list from each file metadata
    std::vector<ROI> tileRoiList;
    tileRoiList.resize(mapTilePathList.size());
    for (size_t i = 0; i < mapTilePathList.size(); ++i)
    {
        getRoiFromMetadata(mapTilePathList.at(i), tileRoiList.at(i));
    }

    // read and add each tile to the full map
    for (size_t i = 0; i < tileRoiList.size(); ++i)
    {
        const ROI roi = intersect(tileRoiList.at(i), imageRoi);
        const std::string mapTilePath = getFileNameFromIndex(mp, rc, fileType, customSuffix, roi.x.begin, roi.y.begin);

        if (roi.isEmpty())
            continue;

        try
        {
            // read tile
            image::Image<T> tileMap;
            image::readImage(mapTilePath, tileMap, image::EImageColorSpace::NO_CONVERSION);

            // add tile to the full map
            addSingleTileMapWeighted(rc, mp, tileParams, roi, scaleStep, tileMap, out_map);
        }
        catch (const std::exception& e)
        {
            ALICEVISION_LOG_WARNING("Cannot find map (rc: " << rc << "): " << fs::path(mapTilePath).filename().string());
        }
    }
}

template<typename T>
void writeMapToFileOrTile(int rc,
                          const MultiViewParams& mp,
                          const EFileType fileType,
                          const TileParams& tileParams,
                          const ROI& roi,
                          const image::Image<T>& in_map,
                          int scale,
                          int step,
                          const std::string& customSuffix = "")
{
    // assert scale & step
    assert(scale > 0);
    assert(step > 0);

    const int scaleStep = scale * step;

    // get image dimensions at scale / stepXY
    const int imageWidth = divideRoundUp(mp.getWidth(rc), scaleStep);
    const int imageHeight = divideRoundUp(mp.getHeight(rc), scaleStep);

    // get downscaled ROI
    const ROI downscaledROI = downscaleROI(roi, scaleStep);

    // check input map dimensions
    assert(in_map.width() == downscaledROI.width() && in_map.width() <= imageWidth);
    assert(in_map.height() == downscaledROI.height() && in_map.height() <= imageHeight);

    // set OIIO ROI for map writing
    // displayRoi is the image region of interest for display (image size)
    // pixelRoi is the buffer region of interest within the displayRoi (tile size)
    // no tiling if displayRoi == pixelRoi
    const oiio::ROI displayRoi(0, imageWidth, 0, imageHeight);
    const oiio::ROI pixelRoi(downscaledROI.x.begin, downscaledROI.x.end, downscaledROI.y.begin, downscaledROI.y.end);

    // output map path
    std::string mapPath;

    if (downscaledROI.width() != imageWidth || downscaledROI.height() != imageHeight)  // is a tile
    {
        // tiled map
        mapPath = getFileNameFromIndex(mp, rc, fileType, customSuffix, roi.x.begin, roi.y.begin);
    }
    else
    {
        // fullsize map
        mapPath = getFileNameFromIndex(mp, rc, fileType, customSuffix);
    }

    // original picture file metadata
    oiio::ParamValueList metadata = image::getMetadataFromMap(mp.getMetadata(rc));

    // downscale metadata
    metadata.push_back(oiio::ParamValue("AliceVision:downscale", mp.getDownscaleFactor(rc) * scaleStep));

    // roi metadata
    {
        metadata.push_back(oiio::ParamValue("AliceVision:roiBeginX", int(roi.x.begin)));
        metadata.push_back(oiio::ParamValue("AliceVision:roiBeginY", int(roi.y.begin)));
        metadata.push_back(oiio::ParamValue("AliceVision:roiEndX", int(roi.x.end)));
        metadata.push_back(oiio::ParamValue("AliceVision:roiEndY", int(roi.y.end)));
    }

    // tile params metadata
    {
        metadata.push_back(oiio::ParamValue("AliceVision:tileBufferWidth", tileParams.bufferWidth));
        metadata.push_back(oiio::ParamValue("AliceVision:tileBufferHeight", tileParams.bufferHeight));
        metadata.push_back(oiio::ParamValue("AliceVision:tilePadding", tileParams.padding));
    }

    // projection matrix metadata
    {
        std::vector<double> matrixP = mp.getOriginalP(rc);
        metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, matrixP.data()));
    }

    // CArr & iCamArr metadata
    {
        Point3d C = mp.CArr[rc];
        Matrix3x3 iP = mp.iCamArr[rc];

        if (scaleStep > 1)
        {
            Matrix3x4 P = mp.camArr[rc];
            for (int i = 0; i < 8; ++i)
                P.m[i] /= double(scaleStep);
            Matrix3x3 K, iK;
            Matrix3x3 R, iR;

            P.decomposeProjectionMatrix(K, R, C);  // replace C
            iK = K.inverse();
            iR = R.inverse();
            iP = iR * iK;  // replace iP
        }

        metadata.push_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, C.m));
        metadata.push_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, iP.m));
    }

    // min/max/nb depth metadata (for depth map only)
    if ((fileType == EFileType::depthMap) || (fileType == EFileType::depthMapFiltered))
    {
        const int nbDepthValues = std::count_if(in_map.data(), in_map.data() + in_map.size(), [](float v) { return v > 0.0f; });
        float maxDepth = -1.0f;
        float minDepth = std::numeric_limits<float>::max();

        for (int i = 0; i < in_map.size(); ++i)
        {
            const float depth = in_map(i);

            if (depth <= -1.0f)
                continue;

            maxDepth = std::max(maxDepth, depth);
            minDepth = std::min(minDepth, depth);
        }

        metadata.push_back(oiio::ParamValue("AliceVision:nbDepthValues", nbDepthValues));
        metadata.push_back(oiio::ParamValue("AliceVision:minDepth", minDepth));
        metadata.push_back(oiio::ParamValue("AliceVision:maxDepth", maxDepth));
    }

    // set colorspace
    image::ImageWriteOptions mapWriteOptions;
    mapWriteOptions.toColorSpace(image::EImageColorSpace::NO_CONVERSION);

    // set storage type
    if ((fileType == EFileType::depthMap) || (fileType == EFileType::depthMapFiltered))
    {
        mapWriteOptions.storageDataType(image::EStorageDataType::Float);
    }
    else
    {
        mapWriteOptions.storageDataType(image::EStorageDataType::Half);
    }

    // write map
    image::writeImage(mapPath, in_map, mapWriteOptions, metadata, displayRoi, pixelRoi);
}

void addTileMapWeighted(int rc,
                        const MultiViewParams& mp,
                        const TileParams& tileParams,
                        const ROI& roi,
                        int downscale,
                        image::Image<float>& in_tileMap,
                        image::Image<float>& inout_map)
{
    addSingleTileMapWeighted(rc, mp, tileParams, roi, downscale, in_tileMap, inout_map);
}

void readMap(int rc,
             const MultiViewParams& mp,
             const EFileType fileType,
             image::Image<float>& out_map,
             int scale,
             int step,
             const std::string& customSuffix)
{
    readMapFromFileOrTiles(rc, mp, fileType, out_map, scale, step, customSuffix);
}

void readMap(int rc,
             const MultiViewParams& mp,
             const EFileType fileType,
             image::Image<image::RGBfColor>& out_map,
             int scale,
             int step,
             const std::string& customSuffix)
{
    readMapFromFileOrTiles(rc, mp, fileType, out_map, scale, step, customSuffix);
}

void writeMap(int rc,
              const MultiViewParams& mp,
              const EFileType fileType,
              const TileParams& tileParams,
              const ROI& roi,
              const image::Image<float>& in_map,
              int scale,
              int step,
              const std::string& customSuffix)
{
    writeMapToFileOrTile(rc, mp, fileType, tileParams, roi, in_map, scale, step, customSuffix);
}

void writeMap(int rc,
              const MultiViewParams& mp,
              const EFileType fileType,
              const TileParams& tileParams,
              const ROI& roi,
              const image::Image<image::RGBfColor>& in_map,
              int scale,
              int step,
              const std::string& customSuffix)
{
    writeMapToFileOrTile(rc, mp, fileType, tileParams, roi, in_map, scale, step, customSuffix);
}

unsigned long getNbDepthValuesFromDepthMap(int rc, const MultiViewParams& mp, int scale, int step, const std::string& customSuffix)
{
    const std::string depthMapPath = getFileNameFromIndex(mp, rc, EFileType::depthMapFiltered, customSuffix);
    int nbDepthValues = -1;
    bool fileExists = false;
    bool fromTiles = false;

    // get nbDepthValues from metadata
    if (fs::exists(depthMapPath))  // untilled
    {
        fileExists = true;
        const oiio::ParamValueList metadata = image::readImageMetadata(depthMapPath);
        nbDepthValues = metadata.get_int("AliceVision:nbDepthValues", -1);
    }
    else  // tilled
    {
        std::vector<std::string> mapTilePathList;
        getTilePathList(rc, mp, EFileType::depthMapFiltered, customSuffix, mapTilePathList);

        if (mapTilePathList.empty())  // depth map can be empty
        {
            ALICEVISION_LOG_INFO("Cannot find any depth map tile file (rc: " << rc << ").");
        }
        else
        {
            fileExists = true;
            fromTiles = true;
        }

        for (const std::string& mapTilePath : mapTilePathList)
        {
            const oiio::ParamValueList metadata = image::readImageMetadata(mapTilePath);

            const int nbTileDepthValues = metadata.get_int("AliceVision:nbDepthValues", -1);

            if (nbTileDepthValues < 0)
                ALICEVISION_THROW_ERROR("Cannot find or incorrect 'AliceVision:nbDepthValues' metadata in depth map tile (rc: " << rc << ")");

            nbDepthValues += nbTileDepthValues;
        }
    }

    ALICEVISION_LOG_TRACE("DepthMap: " << depthMapPath << ", fileExists: " << fileExists << ", fromTiles: " << fromTiles
                                       << ", nbDepthValues (from metadata): " << nbDepthValues);

    // no metadata compute number of depth values
    if (fileExists && nbDepthValues < 0)
    {
        image::Image<float> depthMap;

        ALICEVISION_LOG_WARNING("Can't find or invalid 'nbDepthValues' metadata in depth map (rc: " << rc
                                                                                                    << "). Recompute the number of valid values.");

        readMap(rc, mp, EFileType::depthMapFiltered, depthMap, scale, step, customSuffix);

        nbDepthValues = std::count_if(depthMap.data(), depthMap.data() + depthMap.size(), [](float v) { return v > 0.0f; });
        ALICEVISION_LOG_WARNING("nbDepthValues (recomputed from pixels): " << nbDepthValues);
    }

    return nbDepthValues;
}

void deleteMapTiles(int rc, const MultiViewParams& mp, const EFileType fileType, const std::string& customSuffix)
{
    std::vector<std::string> mapTilePathList;

    getTilePathList(rc, mp, fileType, customSuffix, mapTilePathList);

    if (mapTilePathList.empty())  // depth map can be empty
        ALICEVISION_LOG_INFO("Cannot find any " << getMapNameFromFileType(fileType) << " tile file to delete (rc: " << rc << ").");

    // delete map tile files
    for (const std::string& mapTilePath : mapTilePathList)
    {
        try
        {
            fs::remove(mapTilePath);
        }
        catch (const std::exception& e)
        {
            ALICEVISION_LOG_WARNING("Cannot delete map tile file (rc: " << rc << "): " << fs::path(mapTilePath).filename().string());
        }
    }
}

}  // namespace mvsUtils
}  // namespace aliceVision
