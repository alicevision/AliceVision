// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "depthMapUtils.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/mvsUtils/mapIO.hpp>

#include <assimp/Importer.hpp>
#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

namespace aliceVision {
namespace depthMap {

void copyFloat2Map(image::Image<float>& out_mapX,
                   image::Image<float>& out_mapY,
                   const CudaHostMemoryHeap<float2, 2>& in_map_hmh,
                   const ROI& roi,
                   int downscale)
{
    const ROI downscaledROI = downscaleROI(roi, downscale);
    const int width = int(downscaledROI.width());
    const int height = int(downscaledROI.height());

    // resize output images
    out_mapX.resize(width, height);
    out_mapY.resize(width, height);

    // copy image from host memory to output images
    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            const float2& value = in_map_hmh(size_t(x), size_t(y));
            out_mapX(y, x) = value.x;
            out_mapY(y, x) = value.y;
        }
    }
}

void copyFloat2Map(image::Image<float>& out_mapX,
                   image::Image<float>& out_mapY,
                   const CudaDeviceMemoryPitched<float2, 2>& in_map_dmp,
                   const ROI& roi,
                   int downscale)
{
    // copy float2 map from device pitched memory to host memory
    CudaHostMemoryHeap<float2, 2> map_hmh(in_map_dmp.getSize());
    map_hmh.copyFrom(in_map_dmp);

    copyFloat2Map(out_mapX, out_mapY, map_hmh, roi, downscale);
}

void writeFloat2Map(int rc,
                    const mvsUtils::MultiViewParams& mp,
                    const mvsUtils::TileParams& tileParams,
                    const ROI& roi,
                    const CudaHostMemoryHeap<float2, 2>& in_map_hmh,
                    const mvsUtils::EFileType fileTypeX,
                    const mvsUtils::EFileType fileTypeY,
                    int scale,
                    int step,
                    const std::string& name)
{
    const std::string customSuffix = (name.empty()) ? "" : "_" + name;
    const int scaleStep = scale * step;

    image::Image<float> mapX;
    image::Image<float> mapY;

    copyFloat2Map(mapX, mapY, in_map_hmh, roi, scaleStep);

    mvsUtils::writeMap(rc, mp, fileTypeX, tileParams, roi, mapX, scale, step, customSuffix);
    mvsUtils::writeMap(rc, mp, fileTypeY, tileParams, roi, mapY, scale, step, customSuffix);
}

void writeFloat2Map(int rc,
                    const mvsUtils::MultiViewParams& mp,
                    const mvsUtils::TileParams& tileParams,
                    const ROI& roi,
                    const CudaDeviceMemoryPitched<float2, 2>& in_map_dmp,
                    const mvsUtils::EFileType fileTypeX,
                    const mvsUtils::EFileType fileTypeY,
                    int scale,
                    int step,
                    const std::string& name)
{
    const std::string customSuffix = (name.empty()) ? "" : "_" + name;
    const int scaleStep = scale * step;

    image::Image<float> mapX;
    image::Image<float> mapY;

    copyFloat2Map(mapX, mapY, in_map_dmp, roi, scaleStep);

    mvsUtils::writeMap(rc, mp, fileTypeX, tileParams, roi, mapX, scale, step, customSuffix);
    mvsUtils::writeMap(rc, mp, fileTypeY, tileParams, roi, mapY, scale, step, customSuffix);
}

void writeFloat3Map(int rc,
                    const mvsUtils::MultiViewParams& mp,
                    const mvsUtils::TileParams& tileParams,
                    const ROI& roi,
                    const CudaDeviceMemoryPitched<float3, 2>& in_map_dmp,
                    const mvsUtils::EFileType fileType,
                    int scale,
                    int step,
                    const std::string& name)
{
    const ROI downscaledROI = downscaleROI(roi, scale * step);
    const int width = int(downscaledROI.width());
    const int height = int(downscaledROI.height());

    // copy map from device pitched memory to host memory
    CudaHostMemoryHeap<float3, 2> map_hmh(in_map_dmp.getSize());
    map_hmh.copyFrom(in_map_dmp);

    // copy map from host memory to an Image
    image::Image<image::RGBfColor> map(width, height, true, {0.f, 0.f, 0.f});

    for (size_t x = 0; x < size_t(width); ++x)
    {
        for (size_t y = 0; y < size_t(height); ++y)
        {
            const float3& rgba_hmh = map_hmh(x, y);
            image::RGBfColor& rgb = map(int(y), int(x));
            rgb.r() = rgba_hmh.x;
            rgb.g() = rgba_hmh.y;
            rgb.b() = rgba_hmh.z;
        }
    }

    // write map from the image buffer
    mvsUtils::writeMap(rc, mp, fileType, tileParams, roi, map, scale, step, (name.empty()) ? "" : "_" + name);
}

void writeDeviceImage(const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp, const std::string& path)
{
    const CudaSize<2>& imgSize = in_img_dmp.getSize();

    // copy image from device pitched memory to host memory
    CudaHostMemoryHeap<CudaRGBA, 2> img_hmh(imgSize);
    img_hmh.copyFrom(in_img_dmp);

    // copy image from host memory to an Image
    image::Image<image::RGBfColor> img(imgSize.x(), imgSize.y(), true, {0.f, 0.f, 0.f});

    for (size_t x = 0; x < imgSize.x(); ++x)
    {
        for (size_t y = 0; y < imgSize.y(); ++y)
        {
            const CudaRGBA& rgba_hmh = img_hmh(x, y);
            image::RGBfColor& rgb = img(int(y), int(x));
            rgb.r() = rgba_hmh.x;
            rgb.g() = rgba_hmh.y;
            rgb.b() = rgba_hmh.z;
        }
    }

    // write the image buffer
    image::writeImage(
      path, img, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

void writeNormalMap(int rc,
                    const mvsUtils::MultiViewParams& mp,
                    const mvsUtils::TileParams& tileParams,
                    const ROI& roi,
                    const CudaDeviceMemoryPitched<float3, 2>& in_normalMap_dmp,
                    int scale,
                    int step,
                    const std::string& name)
{
    writeFloat3Map(rc, mp, tileParams, roi, in_normalMap_dmp, mvsUtils::EFileType::normalMap, scale, step, name);
}

void writeNormalMapFiltered(int rc,
                            const mvsUtils::MultiViewParams& mp,
                            const mvsUtils::TileParams& tileParams,
                            const ROI& roi,
                            const CudaDeviceMemoryPitched<float3, 2>& in_normalMap_dmp,
                            int scale,
                            int step,
                            const std::string& name)
{
    writeFloat3Map(rc, mp, tileParams, roi, in_normalMap_dmp, mvsUtils::EFileType::normalMapFiltered, scale, step, name);
}

void writeDepthThicknessMap(int rc,
                            const mvsUtils::MultiViewParams& mp,
                            const mvsUtils::TileParams& tileParams,
                            const ROI& roi,
                            const CudaDeviceMemoryPitched<float2, 2>& in_depthThicknessMap_dmp,
                            int scale,
                            int step,
                            const std::string& name)
{
    const mvsUtils::EFileType fileTypeX = mvsUtils::EFileType::depthMap;
    const mvsUtils::EFileType fileTypeY = mvsUtils::EFileType::thicknessMap;

    writeFloat2Map(rc, mp, tileParams, roi, in_depthThicknessMap_dmp, fileTypeX, fileTypeY, scale, step, name);
}

void writeDepthPixSizeMap(int rc,
                          const mvsUtils::MultiViewParams& mp,
                          const mvsUtils::TileParams& tileParams,
                          const ROI& roi,
                          const CudaDeviceMemoryPitched<float2, 2>& in_depthPixSize_dmp,
                          int scale,
                          int step,
                          const std::string& name)
{
    const mvsUtils::EFileType fileTypeX = mvsUtils::EFileType::depthMap;
    const mvsUtils::EFileType fileTypeY = mvsUtils::EFileType::pixSizeMap;

    writeFloat2Map(rc, mp, tileParams, roi, in_depthPixSize_dmp, fileTypeX, fileTypeY, scale, step, name);
}

void writeDepthSimMap(int rc,
                      const mvsUtils::MultiViewParams& mp,
                      const mvsUtils::TileParams& tileParams,
                      const ROI& roi,
                      const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp,
                      int scale,
                      int step,
                      const std::string& name)
{
    const mvsUtils::EFileType fileTypeX = mvsUtils::EFileType::depthMap;
    const mvsUtils::EFileType fileTypeY = mvsUtils::EFileType::simMap;

    writeFloat2Map(rc, mp, tileParams, roi, in_depthSimMap_dmp, fileTypeX, fileTypeY, scale, step, name);
}

void writeDepthSimMapFromTileList(int rc,
                                  const mvsUtils::MultiViewParams& mp,
                                  const mvsUtils::TileParams& tileParams,
                                  const std::vector<ROI>& tileRoiList,
                                  const std::vector<CudaHostMemoryHeap<float2, 2>>& in_depthSimMapTiles_hmh,
                                  int scale,
                                  int step,
                                  const std::string& name)
{
    ALICEVISION_LOG_TRACE("Merge and write depth/similarity map tiles (rc: " << rc << ", view id: " << mp.getViewId(rc) << ").");

    const std::string customSuffix = (name.empty()) ? "" : "_" + name;

    const ROI imageRoi(Range(0, mp.getWidth(rc)), Range(0, mp.getHeight(rc)));

    const int scaleStep = scale * step;
    const int width = divideRoundUp(mp.getWidth(rc), scaleStep);
    const int height = divideRoundUp(mp.getHeight(rc), scaleStep);

    image::Image<float> depthMap(width, height, true, 0.0f);  // map should be initialize, additive process
    image::Image<float> simMap(width, height, true, 0.0f);    // map should be initialize, additive process

    for (size_t i = 0; i < tileRoiList.size(); ++i)
    {
        const ROI roi = intersect(tileRoiList.at(i), imageRoi);

        if (roi.isEmpty())
            continue;

        image::Image<float> tileDepthMap;
        image::Image<float> tileSimMap;

        // copy tile depth/sim map from host memory
        copyFloat2Map(tileDepthMap, tileSimMap, in_depthSimMapTiles_hmh.at(i), roi, scaleStep);

        // add tile maps to the full-size maps with weighting
        mvsUtils::addTileMapWeighted(rc, mp, tileParams, roi, scaleStep, tileDepthMap, depthMap);
        mvsUtils::addTileMapWeighted(rc, mp, tileParams, roi, scaleStep, tileSimMap, simMap);
    }

    // write fullsize maps on disk
    mvsUtils::writeMap(rc, mp, mvsUtils::EFileType::depthMap, depthMap, scale, step, customSuffix);  // write the merged depth map
    mvsUtils::writeMap(rc, mp, mvsUtils::EFileType::simMap, simMap, scale, step, customSuffix);      // write the merged similarity map
}

void resetDepthSimMap(CudaHostMemoryHeap<float2, 2>& inout_depthSimMap_hmh, float depth, float sim)
{
    const CudaSize<2>& depthSimMapSize = inout_depthSimMap_hmh.getSize();

    for (size_t x = 0; x < depthSimMapSize.x(); ++x)
    {
        for (size_t y = 0; y < depthSimMapSize.y(); ++y)
        {
            float2& depthSim_hmh = inout_depthSimMap_hmh(x, y);
            depthSim_hmh.x = depth;
            depthSim_hmh.y = sim;
        }
    }
}

void mergeNormalMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name)
{
    const std::string customSuffix = (name.empty()) ? "" : "_" + name;

    image::Image<image::RGBfColor> normalMap;

    mvsUtils::readMap(rc, mp, mvsUtils::EFileType::normalMap, normalMap, scale, step, customSuffix);   // read and merge normal map tiles
    mvsUtils::writeMap(rc, mp, mvsUtils::EFileType::normalMap, normalMap, scale, step, customSuffix);  // write the merged normal map
    mvsUtils::deleteMapTiles(rc, mp, mvsUtils::EFileType::normalMap, customSuffix);                    // delete normal map tile files
}

void mergeFloatMapTiles(int rc, const mvsUtils::MultiViewParams& mp, const mvsUtils::EFileType fileType, int scale, int step, const std::string& name)
{
    const std::string customSuffix = (name.empty()) ? "" : "_" + name;

    image::Image<float> map;

    mvsUtils::readMap(rc, mp, fileType, map, scale, step, customSuffix);   // read and merge depth map tiles
    mvsUtils::writeMap(rc, mp, fileType, map, scale, step, customSuffix);  // write the merged depth map
    mvsUtils::deleteMapTiles(rc, mp, fileType, customSuffix);              // delete depth map tile files
}

void mergeDepthThicknessMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name)
{
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::depthMap, scale, step, name);
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::thicknessMap, scale, step, name);
}

void mergeDepthPixSizeMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name)
{
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::depthMap, scale, step, name);
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::pixSizeMap, scale, step, name);
}

void mergeDepthSimMapTiles(int rc, const mvsUtils::MultiViewParams& mp, int scale, int step, const std::string& name)
{
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::depthMap, scale, step, name);
    mergeFloatMapTiles(rc, mp, mvsUtils::EFileType::simMap, scale, step, name);
}

void exportDepthSimMapTilePatternObj(int rc,
                                     const mvsUtils::MultiViewParams& mp,
                                     const std::vector<ROI>& tileRoiList,
                                     const std::vector<std::pair<float, float>>& tileMinMaxDepthsList)
{
    const std::string filepath = mvsUtils::getFileNameFromIndex(mp, rc, mvsUtils::EFileType::tilePattern);

    const int nbRoiCornerVertices = 6;                  // 6 vertices per ROI corner
    const int nbRoiCornerFaces = 4;                     // 4 faces per ROI corner
    const int nbRoiVertices = nbRoiCornerVertices * 4;  // 24 vertices per ROI
    const int nbRoiFaces = nbRoiCornerFaces * 4 + 2;    // 18 faces per ROI (16 for corners + 2 for first/last depth)

    std::vector<Point3d> vertices(nbRoiVertices * tileRoiList.size());
    std::vector<std::tuple<int, int, int>> faces(nbRoiFaces * tileRoiList.size());

    const double cornerPixSize = tileRoiList.front().x.size() / 5;  // corner bevel size in image pixel

    // 2 points offset from corner (to draw a bevel)
    const std::vector<std::pair<Point2d, Point2d>> roiCornerOffsets = {
      {{cornerPixSize, 0.0}, {0.0, cornerPixSize}},   // corner (roi.x.begin, roi.y.begin)
      {{cornerPixSize, 0.0}, {0.0, -cornerPixSize}},  // corner (roi.x.begin, roi.y.end  )
      {{-cornerPixSize, 0.0}, {0.0, cornerPixSize}},  // corner (roi.x.end,   roi.y.begin)
      {{-cornerPixSize, 0.0}, {0.0, -cornerPixSize}}  // corner (roi.x.end,   roi.y.end  )
    };

    // vertex color sets
    const std::vector<aiColor4D> roiColors = {
      {1, 0, 0, 0},
      {0, 1, 0, 0},
      {0, 0, 1, 0},
      {1, 1, 0, 0},
      {0, 1, 1, 0},
      {1, 0, 1, 0},
    };

    // build vertices and faces for each ROI
    for (std::size_t ri = 0; ri < tileRoiList.size(); ++ri)
    {
        const ROI& roi = tileRoiList.at(ri);

        const auto& minMaxDepth = tileMinMaxDepthsList.at(ri);
        const Point3d planeN = (mp.iRArr[rc] * Point3d(0.0f, 0.0f, 1.0f)).normalize();  // plane normal
        const Point3d firstPlaneP = mp.CArr[rc] + planeN * minMaxDepth.first;           // first depth plane point
        const Point3d lastPlaneP = mp.CArr[rc] + planeN * minMaxDepth.second;           // last depth plane point

        const std::vector<Point2d> roiCorners = {{double(roi.x.begin), double(roi.y.begin)},
                                                 {double(roi.x.begin), double(roi.y.end)},
                                                 {double(roi.x.end), double(roi.y.begin)},
                                                 {double(roi.x.end), double(roi.y.end)}};

        // build vertices and faces for each ROI corner
        for (std::size_t ci = 0; ci < roiCorners.size(); ++ci)
        {
            const std::size_t vStartIdx = ri * nbRoiVertices + ci * nbRoiCornerVertices;
            const std::size_t fStartIdx = ri * nbRoiFaces + ci * nbRoiCornerFaces;

            const auto& corner = roiCorners.at(ci);  // corner 2d point
            const auto& cornerOffsets = roiCornerOffsets.at(ci);

            const Point2d cornerX = corner + cornerOffsets.first;   // corner 2d point X offsetted
            const Point2d cornerY = corner + cornerOffsets.second;  // corner 2d point Y offsetted

            vertices[vStartIdx] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * corner).normalize(), firstPlaneP, planeN);
            vertices[vStartIdx + 1] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * corner).normalize(), lastPlaneP, planeN);
            vertices[vStartIdx + 2] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerX).normalize(), firstPlaneP, planeN);
            vertices[vStartIdx + 3] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerX).normalize(), lastPlaneP, planeN);
            vertices[vStartIdx + 4] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerY).normalize(), firstPlaneP, planeN);
            vertices[vStartIdx + 5] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerY).normalize(), lastPlaneP, planeN);

            faces[fStartIdx] = {vStartIdx, vStartIdx + 1, vStartIdx + 2};
            faces[fStartIdx + 1] = {vStartIdx + 1, vStartIdx + 2, vStartIdx + 3};
            faces[fStartIdx + 2] = {vStartIdx, vStartIdx + 1, vStartIdx + 4};
            faces[fStartIdx + 3] = {vStartIdx + 1, vStartIdx + 4, vStartIdx + 5};
        }

        // build first/last depth faces
        {
            const std::size_t vStartIdx = ri * nbRoiVertices;
            const std::size_t fStartIdx = ri * nbRoiFaces + roiCorners.size() * nbRoiCornerFaces;

            // first depth
            faces[fStartIdx] = {vStartIdx, vStartIdx + 1 * nbRoiCornerVertices, vStartIdx + 2 * nbRoiCornerVertices};

            // last depth
            faces[fStartIdx + 1] = {
              vStartIdx + 1 * nbRoiCornerVertices + 1, vStartIdx + 2 * nbRoiCornerVertices + 1, vStartIdx + 3 * nbRoiCornerVertices + 1};
        }
    }

    aiScene scene;

    scene.mRootNode = new aiNode;

    scene.mMeshes = new aiMesh*[1];
    scene.mNumMeshes = 1;
    scene.mRootNode->mMeshes = new unsigned int[1];
    scene.mRootNode->mNumMeshes = 1;

    scene.mMaterials = new aiMaterial*[1];
    scene.mNumMaterials = 1;
    scene.mMaterials[0] = new aiMaterial;

    scene.mRootNode->mMeshes[0] = 0;
    scene.mMeshes[0] = new aiMesh;
    aiMesh* aimesh = scene.mMeshes[0];
    aimesh->mMaterialIndex = 0;

    aimesh->mNumVertices = vertices.size();
    aimesh->mVertices = new aiVector3D[vertices.size()];

    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        const auto& vertex = vertices[i];
        aimesh->mVertices[i].x = vertex.x;
        aimesh->mVertices[i].y = -vertex.y;  // openGL display
        aimesh->mVertices[i].z = -vertex.z;  // openGL display
    }

    aimesh->mColors[0] = new aiColor4D[vertices.size()];

    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
        aimesh->mColors[0][i] = roiColors[(i / nbRoiVertices) % roiColors.size()];
    }

    aimesh->mNumFaces = faces.size();
    aimesh->mFaces = new aiFace[faces.size()];

    for (std::size_t i = 0; i < faces.size(); ++i)
    {
        const auto& face = faces[i];
        aimesh->mFaces[i].mNumIndices = 3;
        aimesh->mFaces[i].mIndices = new unsigned int[3];
        aimesh->mFaces[i].mIndices[0] = std::get<0>(face);
        aimesh->mFaces[i].mIndices[1] = std::get<1>(face);
        aimesh->mFaces[i].mIndices[2] = std::get<2>(face);
    }

    const std::string formatId = "objnomtl";
    const unsigned int pPreprocessing = 0u;

    Assimp::Exporter exporter;
    exporter.Export(&scene, formatId, filepath, pPreprocessing);

    ALICEVISION_LOG_INFO("Save debug tiles pattern obj (rc: " << rc << ", view id: " << mp.getViewId(rc) << ") done.");
}

}  // namespace depthMap
}  // namespace aliceVision
