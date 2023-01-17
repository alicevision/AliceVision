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

void writeDeviceImage(const CudaDeviceMemoryPitched<CudaRGBA, 2>& in_img_dmp, const std::string& path) 
{
    const CudaSize<2>& imgSize = in_img_dmp.getSize();
    
    // copy image from device pitched memory to host memory
    CudaHostMemoryHeap<CudaRGBA, 2> img_hmh(imgSize);
    img_hmh.copyFrom(in_img_dmp);

    // copy image from host memory to an Image
    image::Image<image::RGBfColor> img(imgSize.x(), imgSize.y(), true, {0.f,0.f,0.f});

    for(size_t x = 0; x < imgSize.x(); ++x)
    {
        for(size_t y = 0; y < imgSize.y(); ++y)
        {
            const CudaRGBA& rgba_hmh = img_hmh(x, y);
            image::RGBfColor& rgb = img(int(y), int(x));
            rgb.r() = rgba_hmh.x;
            rgb.g() = rgba_hmh.y;
            rgb.b() = rgba_hmh.z;
        }
    }

    // write the vector buffer
    image::writeImage(path, img, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

void writeDeviceImage(const CudaDeviceMemoryPitched<float3, 2>& in_img_dmp, const std::string& path)
{
    const CudaSize<2>& imgSize = in_img_dmp.getSize();

    // copy image from device pitched memory to host memory
    CudaHostMemoryHeap<float3, 2> img_hmh(imgSize);
    img_hmh.copyFrom(in_img_dmp);

    // copy image from host memory to an Image
    image::Image<image::RGBfColor> img(imgSize.x(), imgSize.y(), true, {0.f, 0.f, 0.f});

    for(size_t x = 0; x < imgSize.x(); ++x)
    {
        for(size_t y = 0; y < imgSize.y(); ++y)
        {
            const float3& rgba_hmh = img_hmh(x, y);
            image::RGBfColor& rgb = img(int(y), int(x));
            rgb.r() = rgba_hmh.x;
            rgb.g() = rgba_hmh.y;
            rgb.b() = rgba_hmh.z;
        }
    }

    // write the vector buffer
    image::writeImage(path, img, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
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

void copyDepthSimMap(image::Image<float>& out_depthMap, image::Image<float>& out_simMap, const CudaHostMemoryHeap<float2, 2>& in_depthSimMap_hmh, const ROI& roi, int downscale)
{
    const ROI downscaledROI = downscaleROI(roi, downscale);
    const int width  = int(downscaledROI.width());
    const int height = int(downscaledROI.height());

    // resize output vectors
    out_depthMap.resize(width, height);
    out_simMap.resize(width, height);

    // copy image from host memory to output vectors
    for(int x = 0; x < width; ++x)
    {
        for(int y = 0; y < height; ++y)
        {
            const float2& depthSim = in_depthSimMap_hmh(x, y);
            out_depthMap(y, x) = depthSim.x;
            out_simMap(y, x) = depthSim.y;
        }
    }
}

void copyDepthSimMap(image::Image<float>& out_depthMap, image::Image<float>& out_simMap, const CudaDeviceMemoryPitched<float2, 2>& in_depthSimMap_dmp, const ROI& roi, int downscale)
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

    image::Image<float> depthMap;
    image::Image<float> simMap;

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

    image::Image<float> depthMap;
    image::Image<float> simMap;

    copyDepthSimMap(depthMap, simMap, in_depthSimMap_dmp, roi, scaleStep);

    mvsUtils::writeDepthSimMap(rc, mp, tileParams, roi, depthMap, simMap, scale, step, customSuffix);
}

void writeDepthSimMapFromTileList(int rc,
                                  const mvsUtils::MultiViewParams& mp,
                                  const mvsUtils::TileParams& tileParams,
                                  const std::vector<ROI>& tileRoiList,
                                  const std::vector<CudaHostMemoryHeap<float2, 2>>& in_depthSimMapTiles_hmh,
                                  int scale,
                                  int step,
                                  const std::string& customSuffix)
{
  ALICEVISION_LOG_TRACE("Merge and write depth/similarity map tiles (rc: " << rc << ", view id: " << mp.getViewId(rc) << ").");
  
  const ROI imageRoi(Range(0, mp.getWidth(rc)), Range(0, mp.getHeight(rc)));
  
  const int scaleStep = scale * step;
  const int width  = divideRoundUp(mp.getWidth(rc),  scaleStep);
  const int height = divideRoundUp(mp.getHeight(rc), scaleStep);

  image::Image<float> depthMap(width, height, true, 0.0f); // map should be initialize, additive process
  image::Image<float> simMap(width, height, true, 0.0f);   // map should be initialize, additive process

  for(size_t i = 0; i < tileRoiList.size(); ++i)
  {
    const ROI roi = intersect(tileRoiList.at(i), imageRoi);

    if(roi.isEmpty())
        continue;

    image::Image<float> tileDepthMap;
    image::Image<float> tileSimMap;

    // copy tile depth/sim map from host memory
    copyDepthSimMap(tileDepthMap, tileSimMap, in_depthSimMapTiles_hmh.at(i), roi, scaleStep);

    // add tile maps to the full-size maps with weighting
    addTileMapWeighted(rc, mp, tileParams, roi, scaleStep, tileDepthMap, depthMap);
    addTileMapWeighted(rc, mp, tileParams, roi, scaleStep, tileSimMap,   simMap);
  }

  // write full-size maps on disk
  mvsUtils::writeDepthSimMap(rc, mp, depthMap, simMap, scale, step, customSuffix);
}

void mergeDepthSimMapTiles(int rc,
                           const mvsUtils::MultiViewParams& mp,
                           int scale,
                           int step,
                           const std::string& customSuffix)
{
    image::Image<float> depthMap;
    image::Image<float> simMap;

    mvsUtils::readDepthSimMap(rc, mp, depthMap, simMap, scale, step, customSuffix);  // read and merge tiles
    mvsUtils::writeDepthSimMap(rc, mp, depthMap, simMap, scale, step, customSuffix); // write the merged depth/sim maps
    mvsUtils::deleteDepthSimMapTiles(rc, mp, scale, step, customSuffix);             // delete tile files
}

void exportDepthSimMapTilePatternObj(int rc,
                                     const mvsUtils::MultiViewParams& mp,
                                     const std::vector<ROI>& tileRoiList,
                                     const std::vector<std::pair<float, float>>& tileMinMaxDepthsList)
{
  const std::string filepath = mvsUtils::getFileNameFromIndex(mp, rc, mvsUtils::EFileType::tilePattern, 1);

  const int nbRoiCornerVertices = 6;                 // 6 vertices per ROI corner
  const int nbRoiCornerFaces = 4;                    // 4 faces per ROI corner
  const int nbRoiVertices = nbRoiCornerVertices * 4; // 24 vertices per ROI
  const int nbRoiFaces = nbRoiCornerFaces * 4 + 2;   // 18 faces per ROI (16 for corners + 2 for first/last depth)

  std::vector<Point3d> vertices(nbRoiVertices * tileRoiList.size());
  std::vector<std::tuple<int,int,int>> faces(nbRoiFaces * tileRoiList.size());

  const double cornerPixSize = tileRoiList.front().x.size() / 5;  // corner bevel size in image pixel

  // 2 points offset from corner (to draw a bevel)
  const std::vector<std::pair<Point2d, Point2d>> roiCornerOffsets = {
    {{ cornerPixSize, 0.0},{0.0,  cornerPixSize}},  // corner (roi.x.begin, roi.y.begin)
    {{ cornerPixSize, 0.0},{0.0, -cornerPixSize}},  // corner (roi.x.begin, roi.y.end  )
    {{-cornerPixSize, 0.0},{0.0,  cornerPixSize}},  // corner (roi.x.end,   roi.y.begin)
    {{-cornerPixSize, 0.0},{0.0, -cornerPixSize}}   // corner (roi.x.end,   roi.y.end  )
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
  for(std::size_t ri = 0; ri < tileRoiList.size(); ++ri)
  {
      const ROI& roi = tileRoiList.at(ri);

      const auto& minMaxDepth = tileMinMaxDepthsList.at(ri);
      const Point3d planeN = (mp.iRArr[rc] * Point3d(0.0f, 0.0f, 1.0f)).normalize(); // plane normal
      const Point3d firstPlaneP = mp.CArr[rc] + planeN * minMaxDepth.first;          // first depth plane point
      const Point3d lastPlaneP  = mp.CArr[rc] + planeN * minMaxDepth.second;         // last depth plane point

      const std::vector<Point2d> roiCorners = {
        {double(roi.x.begin), double(roi.y.begin)},
        {double(roi.x.begin), double(roi.y.end)  },
        {double(roi.x.end),   double(roi.y.begin)},
        {double(roi.x.end),   double(roi.y.end)  }
      };

      // build vertices and faces for each ROI corner
      for(std::size_t ci = 0; ci < roiCorners.size(); ++ci)
      {
        const std::size_t vStartIdx = ri * nbRoiVertices + ci * nbRoiCornerVertices;
        const std::size_t fStartIdx = ri * nbRoiFaces + ci * nbRoiCornerFaces;

        const auto& corner = roiCorners.at(ci); // corner 2d point
        const auto& cornerOffsets = roiCornerOffsets.at(ci);

        const Point2d cornerX = corner + cornerOffsets.first;  // corner 2d point X offsetted
        const Point2d cornerY = corner + cornerOffsets.second; // corner 2d point Y offsetted

        vertices[vStartIdx    ] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * corner ).normalize(), firstPlaneP, planeN);
        vertices[vStartIdx + 1] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * corner ).normalize(), lastPlaneP , planeN);
        vertices[vStartIdx + 2] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerX).normalize(), firstPlaneP, planeN);
        vertices[vStartIdx + 3] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerX).normalize(), lastPlaneP , planeN);
        vertices[vStartIdx + 4] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerY).normalize(), firstPlaneP, planeN);
        vertices[vStartIdx + 5] = linePlaneIntersect(mp.CArr[rc], (mp.iCamArr[rc] * cornerY).normalize(), lastPlaneP , planeN);

        faces[fStartIdx    ] = {vStartIdx    , vStartIdx + 1, vStartIdx + 2};
        faces[fStartIdx + 1] = {vStartIdx + 1, vStartIdx + 2, vStartIdx + 3};
        faces[fStartIdx + 2] = {vStartIdx    , vStartIdx + 1, vStartIdx + 4};
        faces[fStartIdx + 3] = {vStartIdx + 1, vStartIdx + 4, vStartIdx + 5};
      }

      // build first/last depth faces
      {
          const std::size_t vStartIdx = ri * nbRoiVertices;
          const std::size_t fStartIdx = ri * nbRoiFaces + roiCorners.size() * nbRoiCornerFaces;

          // first depth
          faces[fStartIdx    ] = {vStartIdx, 
                                  vStartIdx + 1 * nbRoiCornerVertices, 
                                  vStartIdx + 2 * nbRoiCornerVertices}; 

          // last depth
          faces[fStartIdx + 1] = {vStartIdx + 1 * nbRoiCornerVertices + 1, 
                                  vStartIdx + 2 * nbRoiCornerVertices + 1,
                                  vStartIdx + 3 * nbRoiCornerVertices + 1};
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

  for(std::size_t i = 0; i < vertices.size(); ++i)
  {
      const auto& vertex = vertices[i];
      aimesh->mVertices[i].x = vertex.x;
      aimesh->mVertices[i].y = -vertex.y; // openGL display
      aimesh->mVertices[i].z = -vertex.z; // openGL display
  }

  aimesh->mColors[0] = new aiColor4D[vertices.size()];

  for(std::size_t i = 0; i < vertices.size(); ++i)
  {
      aimesh->mColors[0][i] = roiColors[(i/nbRoiVertices) % roiColors.size()];
  }

  aimesh->mNumFaces = faces.size();
  aimesh->mFaces = new aiFace[faces.size()];

  for(std::size_t i = 0; i < faces.size(); ++i)
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

} // namespace depthMap
} // namespace aliceVision
