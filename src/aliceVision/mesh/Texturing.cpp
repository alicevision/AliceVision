// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Texturing.hpp"
#include "geoMesh.hpp"
#include "UVAtlas.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/mvsData/Image.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

#include <geogram/basic/common.h>
#include <geogram/basic/geometry_nd.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/parameterization/mesh_atlas_maker.h>

#include <geogram/basic/permutation.h>
#include <geogram/basic/attributes.h>
#include <geogram/basic/geometry_nd.h>
#include <geogram/points/kd_tree.h>
#include <geogram/mesh/mesh_AABB.h>
#include <geogram/mesh/mesh_reorder.h>
#include <geogram/mesh/mesh_geometry.h>

#include <boost/algorithm/string/case_conv.hpp> 

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <map>
#include <set>

// Debug mode: save atlases decomposition in frequency bands and
// the number of contribution in each band (if useScore is set to false)
#define TEXTURING_MBB_DEBUG 0

namespace aliceVision {
namespace mesh {

EUnwrapMethod EUnwrapMethod_stringToEnum(const std::string& method)
{
    std::string m = method;
    boost::to_lower(m);

    if(m == "basic")
        return EUnwrapMethod::Basic;
    if(m == "abf")
        return EUnwrapMethod::ABF;
    if(m == "lscm")
        return EUnwrapMethod::LSCM;
    throw std::out_of_range("Invalid unwrap method " + method);
}

std::string EUnwrapMethod_enumToString(EUnwrapMethod method)
{
    switch(method)
    {
    case EUnwrapMethod::Basic:
        return "Basic";
    case EUnwrapMethod::ABF:
        return "ABF";
    case EUnwrapMethod::LSCM:
        return "LSCM";
    }
    throw std::out_of_range("Unrecognized EUnwrapMethod");
}

EVisibilityRemappingMethod EVisibilityRemappingMethod_stringToEnum(const std::string& method)
{
    if (method == "Pull")
        return EVisibilityRemappingMethod::Pull;
    if (method == "Push")
        return EVisibilityRemappingMethod::Push;
    if (method == "PullPush")
        return EVisibilityRemappingMethod::PullPush;
    if(method == "MeshItself")
        return EVisibilityRemappingMethod::MeshItself;
    throw std::out_of_range("Invalid visibilities remapping method " + method);
}

std::string EVisibilityRemappingMethod_enumToString(EVisibilityRemappingMethod method)
{
    switch (method)
    {
    case EVisibilityRemappingMethod::Pull:
        return "Push";
    case EVisibilityRemappingMethod::Push:
        return "Pull";
    case EVisibilityRemappingMethod::PullPush:
        return "PullPush";
    case EVisibilityRemappingMethod::MeshItself:
        return "MeshItself";
    }
    throw std::out_of_range("Unrecognized EVisibilityRemappingMethod");
}

EBumpMappingType EBumpMappingType_stringToEnum(const std::string& type) 
{
    if(type == "Height")
        return EBumpMappingType::Height;
    if(type == "Normal")
        return EBumpMappingType::Normal;
    throw std::out_of_range("Invalid bump mapping type " + type);
}
std::string EBumpMappingType_enumToString(EBumpMappingType type) 
{
    switch(type)
    {
        case EBumpMappingType::Height:
            return "Height";
        case EBumpMappingType::Normal:
            return "Normal";
    }
    throw std::out_of_range("Invalid bump mapping type enum");
}
std::ostream& operator<<(std::ostream& os, EBumpMappingType bumpMappingType)
{
    return os << EBumpMappingType_enumToString(bumpMappingType);
}
std::istream& operator>>(std::istream& in, EBumpMappingType& bumpMappingType)
{
    std::string token;
    in >> token;
    bumpMappingType = EBumpMappingType_stringToEnum(token);
    return in;
}

/**
 * @brief Return whether a pixel is contained in or intersected by a 2D triangle.
 * @param[in] triangle the triangle as an array of 3 point2Ds
 * @param[in] pixel the pixel to test
 * @param[out] barycentricCoords the barycentric
 *  coordinates of this pixel relative to \p triangle
 * @return
 */
bool isPixelInTriangle(const Point2d* triangle, const Pixel& pixel, Point2d& barycentricCoords)
{
    // get pixel center
    GEO::vec2 p(pixel.x + 0.5, pixel.y + 0.5);
    GEO::vec2 V0(triangle[0].x, triangle[0].y);
    GEO::vec2 V1(triangle[1].x, triangle[1].y);
    GEO::vec2 V2(triangle[2].x, triangle[2].y);
    GEO::vec2 closestPoint;
    double l1, l2, l3;
    double dist = GEO::Geom::point_triangle_squared_distance<GEO::vec2>(p, V0, V1, V2, closestPoint, l1, l2, l3);
    // fill barycentric coordinates as expected by other internal methods
    barycentricCoords.x = l3;
    barycentricCoords.y = l2;
    // tolerance threshold of 1/2 pixel for pixels on the edges of the triangle
    return dist < 0.5 + std::numeric_limits<double>::epsilon();
}

Point2d barycentricToCartesian(const Point2d* triangle, const Point2d& coords)
{
    return triangle[0] + (triangle[2] - triangle[0]) * coords.x + (triangle[1] - triangle[0]) * coords.y;
}

Point3d barycentricToCartesian(const Point3d* triangle, const Point2d& coords)
{
    return triangle[0] + (triangle[2] - triangle[0]) * coords.x + (triangle[1] - triangle[0]) * coords.y;
}

void Texturing::generateUVsBasicMethod(mvsUtils::MultiViewParams& mp)
{
    if(!mesh)
        throw std::runtime_error("Can't generate UVs without a mesh");
    if(mesh->pointsVisibilities.empty())
        throw std::runtime_error("Points visibilities are required for basic unwrap method.");

    // automatic uv atlasing
    ALICEVISION_LOG_INFO("Generating UVs (textureSide: " << texParams.textureSide << "; padding: " << texParams.padding << ").");
    UVAtlas mua(*mesh, mp, texParams.textureSide, texParams.padding);

    // create a new mesh to store data
    mesh->trisUvIds.reserve(mesh->tris.size());
    mesh->uvCoords.reserve(mesh->pts.size()); // not sufficient
    _atlases.clear();
    _atlases.resize(mua.atlases().size());
    mesh->nmtls = mua.atlases().size();

    // std::map<int, int> vertexCache;
    // PointsVisibility* updatedPointsCams = new PointsVisibility;
    // updatedPointsCams->reserve(mesh->pointsVisibilities.size());

    int atlasId = 0;

    for(auto& charts : mua.atlases())
    {
        for(auto& chart : charts)
        {
            if(chart.refCameraID == -1)
                continue;

            std::map<int, int> uvCache;

            Point2d sourceLU(chart.sourceLU.x, chart.sourceLU.y);
            Point2d targetLU(chart.targetLU.x, chart.targetLU.y);

            // for each triangle in this chart
            for(size_t i = 0; i < chart.triangleIDs.size(); ++i)
            {
                int triangleID = chart.triangleIDs[i];
                // register triangle in corresponding atlas
                mesh->trisMtlIds()[triangleID] = atlasId;
                _atlases[atlasId].push_back(triangleID);

                Voxel& triUvIds = mesh->trisUvIds[triangleID];
                // for each point
                for(int k = 0; k < 3; ++k)
                {
                    int pointId = mesh->tris[triangleID].v[k];

                    int uvIdx;
                    auto uvCacheIt = uvCache.find(pointId);
                    // if uv coords for this point had not been computed in this chart yet
                    if(uvCacheIt == uvCache.end())
                    {
                        const Point3d& p = mesh->pts[pointId];
                        Point2d uvPix;

                        Point2d pix;
                        mp.getPixelFor3DPoint(&pix, p, chart.refCameraID);
                        if(mp.isPixelInImage(pix, chart.refCameraID))
                        {
                            // compute the final pixel coordinates
                            // get pixel offset in reference camera space with applied downscale
                            Point2d dp = (pix - sourceLU) * chart.downscale;
                            // add this offset to targetLU to get final pixel coordinates + normalize
                            uvPix = (targetLU + dp) / (float)mua.textureSide();
                            uvPix.y = 1.0 - uvPix.y;

                            // sanity check: discard invalid UVs
                            if(   uvPix.x < 0 || uvPix.x > 1.0
                                  || uvPix.y < 0 || uvPix.x > 1.0 )
                            {
                                ALICEVISION_LOG_WARNING("Discarding invalid UV: " + std::to_string(uvPix.x) + ", " + std::to_string(uvPix.y));
                                uvPix = Point2d();
                            }

                            if(texParams.useUDIM)
                            {
                                uvPix.x += atlasId % 10;
                                uvPix.y += atlasId / 10;
                            }
                        }
                        mesh->uvCoords.push_back(uvPix);
                        uvIdx = mesh->uvCoords.size() - 1;
                        uvCache[pointId] = uvIdx;
                    }
                    else {
                        uvIdx = uvCacheIt->second;
                    }
                    triUvIds.m[k] = uvIdx;
                }
            }

        }
        ++atlasId;
    }
}

void Texturing::updateAtlases()
{
    ALICEVISION_LOG_INFO("updateAtlases");
    // Fill atlases (1 atlas per material) with triangles issued from mesh subdivision
    _atlases.clear();
    _atlases.resize(std::max(1, mesh->nmtls));
    for(int triangleID = 0; triangleID < mesh->trisMtlIds().size(); ++triangleID)
    {
        unsigned int atlasID = mesh->nmtls ? mesh->trisMtlIds()[triangleID] : 0;
        if(mesh->trisMtlIds()[triangleID] != -1)
            _atlases[atlasID].push_back(triangleID);
    }
}

void Texturing::generateTextures(const mvsUtils::MultiViewParams& mp,
                                 const boost::filesystem::path& outPath, imageIO::EImageFileType textureFileType)
{
    // Ensure that contribution levels do not contain 0 and are sorted (as each frequency band contributes to lower bands).
    auto& m = texParams.multiBandNbContrib;
    m.erase(std::remove(std::begin(m), std::end(m), 0), std::end(m));
    texParams.nbBand = m.size();

    if(!std::is_sorted(std::begin(m), std::end(m)))
    {
        ALICEVISION_LOG_INFO("Sorting contributions per band (necessary).");
        std::sort(std::begin(m), std::end(m));
    }
    ALICEVISION_LOG_INFO("Texturing: Use multiband blending with the following contributions per band:");
    for(int c: texParams.multiBandNbContrib)
    {
        ALICEVISION_LOG_INFO("  - " << c);
    }
    std::partial_sum(m.begin(), m.end(), m.begin());

    ALICEVISION_LOG_INFO("Texturing in " + imageIO::EImageColorSpace_enumToString(texParams.processColorspace) + " colorspace.");
    mvsUtils::ImagesCache<ImageRGBf> imageCache(mp, texParams.processColorspace, texParams.correctEV);
    imageCache.setCacheSize(2);
    ALICEVISION_LOG_INFO("Images loaded from cache with: " + ECorrectEV_enumToString(texParams.correctEV));

    //calculate the maximum number of atlases in memory in MB
    system::MemoryInfo memInfo = system::getMemoryInfo();
    const std::size_t imageMaxMemSize =  mp.getMaxImageWidth() * mp.getMaxImageHeight() * sizeof(ColorRGBf) / std::pow(2,20); //MB
    const std::size_t imagePyramidMaxMemSize = texParams.nbBand * imageMaxMemSize;
    const std::size_t atlasContribMemSize = texParams.textureSide * texParams.textureSide * (sizeof(ColorRGBf)+sizeof(float)) / std::pow(2,20); //MB
    const std::size_t atlasPyramidMaxMemSize = texParams.nbBand * atlasContribMemSize;

    const int availableRam = int(memInfo.availableRam / std::pow(2,20));
    const int availableMem = availableRam - 2 * (imagePyramidMaxMemSize + imageMaxMemSize); // keep some memory for the 2 input images in cache and one laplacian pyramid

    const int nbAtlas = _atlases.size();
    // Memory needed to process each attlas = input + input pyramid + output atlas pyramid
    const int memoryPerAtlas = (imageMaxMemSize + imagePyramidMaxMemSize) + atlasPyramidMaxMemSize;
    int nbAtlasMax = std::floor(availableMem / double(memoryPerAtlas)); //maximum number of textures laplacian pyramid in RAM
    nbAtlasMax = std::min(nbAtlas, nbAtlasMax); //if enough memory, do it with all atlases

    ALICEVISION_LOG_INFO("nbAtlas: " << nbAtlas);
    ALICEVISION_LOG_INFO("availableRam: " << availableRam);
    ALICEVISION_LOG_INFO("availableMem: " << availableMem);
    ALICEVISION_LOG_INFO("memoryPerAtlas: " << memoryPerAtlas);

    ALICEVISION_LOG_DEBUG("nbAtlasMax: " << nbAtlasMax);

    // Add rounding to have a uniform repartition between chunks (avoid a small chunk at the end)
    const int nChunks = std::ceil(nbAtlas / double(nbAtlasMax));
    nbAtlasMax = std::ceil(nbAtlas / double(nChunks));
    ALICEVISION_LOG_DEBUG("nChunks: " << nChunks);
    ALICEVISION_LOG_INFO("nbAtlasMax (after rounding): " << nbAtlasMax);

    if (availableMem - nbAtlasMax*atlasPyramidMaxMemSize < 1000) //keep 1 GB margin in memory
        nbAtlasMax -= 1;
    nbAtlasMax = std::max(1, nbAtlasMax); //if not enough memory, do it one by one

    ALICEVISION_LOG_INFO("Total amount of available RAM: " << availableRam << " MB.");
    ALICEVISION_LOG_INFO("Total amount of memory remaining for the computation: " << availableMem << " MB.");
    ALICEVISION_LOG_INFO("Total amount of an image in memory: " << imageMaxMemSize << " MB.");
    ALICEVISION_LOG_INFO("Total amount of an atlas pyramid in memory: " << atlasPyramidMaxMemSize << " MB.");
    ALICEVISION_LOG_INFO("Processing " << nbAtlas << " atlases by chunks of " << nbAtlasMax);

    //generateTexture for the maximum number of atlases, and iterate
    const std::div_t divresult = div(nbAtlas, nbAtlasMax);
    std::vector<size_t> atlasIDs;
    atlasIDs.reserve(nbAtlasMax);
    for(int n = 0; n <= divresult.quot; ++n)
    {
        atlasIDs.clear();
        int imax = (n < divresult.quot ? nbAtlasMax : divresult.rem);
        if(!imax)
            continue;
        for(int i = 0; i < imax; ++i)
        {
            size_t atlasID = size_t(n*nbAtlasMax + i);
            atlasIDs.push_back(atlasID);
        }
        ALICEVISION_LOG_INFO("Generating texture for atlases " << n*nbAtlasMax + 1 << " to " << n*nbAtlasMax+imax );
        generateTexturesSubSet(mp, atlasIDs, imageCache, outPath, textureFileType);
    }
}

void Texturing::generateTexturesSubSet(const mvsUtils::MultiViewParams& mp,
                                const std::vector<size_t>& atlasIDs, mvsUtils::ImagesCache<ImageRGBf>& imageCache, const bfs::path& outPath, imageIO::EImageFileType textureFileType)
{
    if(atlasIDs.size() > _atlases.size())
        throw std::runtime_error("Invalid atlas IDs ");

    unsigned int textureSize = texParams.textureSide * texParams.textureSide;

    // We select the best cameras for each triangle and store it per camera for each output texture files.
    // Triangles contributions are stored per frequency bands for multi-band blending.
    using AtlasIndex = size_t;
    using ScorePerTriangle = std::vector<std::pair<unsigned int, float>>; //list of <triangleId, score>
    std::vector<std::map<AtlasIndex, std::vector<ScorePerTriangle>>> contributionsPerCamera(mp.ncams);

    //for each atlasID, calculate contributionPerCamera
    for(const size_t atlasID : atlasIDs)
    {
        ALICEVISION_LOG_INFO("Generating texture for atlas " << atlasID + 1 << "/" << _atlases.size()
                  << " (" << _atlases[atlasID].size() << " triangles).");

        // iterate over atlas' triangles
        for(size_t i = 0; i < _atlases[atlasID].size(); ++i)
        {
            int triangleID = _atlases[atlasID][i];

            // Fuse visibilities of the 3 vertices
            std::vector<int> allTriCams;
            for (int k = 0; k < 3; ++k)
            {
                const int pointIndex = mesh->tris[triangleID].v[k];
                const StaticVector<int> pointVisibilities = mesh->pointsVisibilities[pointIndex];
                if (!pointVisibilities.empty())
                {
                    std::copy(pointVisibilities.begin(), pointVisibilities.end(), std::inserter(allTriCams, allTriCams.end()));
                }
            }
            if (allTriCams.empty())
            {
                // triangle without visibility
                ALICEVISION_LOG_TRACE("No visibility for triangle " << triangleID << " in texture atlas " << atlasID << ".");
                continue;
            }
            std::sort(allTriCams.begin(), allTriCams.end());

            std::vector<std::pair<int, int>> selectedTriCams; // <camId, nbVertices>
            selectedTriCams.emplace_back(allTriCams.front(), 1);
            for (int j = 1; j < allTriCams.size(); ++j)
            {
                const unsigned int camId = allTriCams[j];
                if(selectedTriCams.back().first == camId)
                {
                    ++selectedTriCams.back().second;
                }
                else
                {
                    selectedTriCams.emplace_back(camId, 1);
                }
            }

            assert(!selectedTriCams.empty());

            // Select the N best views for texturing
            Point3d triangleNormal;
            Point3d triangleCenter;
            if (texParams.angleHardThreshold != 0.0)
            {
                triangleNormal = mesh->computeTriangleNormal(triangleID);
                triangleCenter = mesh->computeTriangleCenterOfGravity(triangleID);
            }
            using ScoreCamId = std::tuple<int, double, int>; // <nbVertex, score, camId>
            std::vector<ScoreCamId> scorePerCamId;
            for (const auto& itCamVis: selectedTriCams)
            {
                const int camId = itCamVis.first;
                const int verticesSupport = itCamVis.second;
                if(texParams.forceVisibleByAllVertices && verticesSupport < 3)
                    continue;

                if (texParams.angleHardThreshold != 0.0)
                {
                    const Point3d vecPointToCam = (mp.CArr[camId] - triangleCenter).normalize();
                    const double angle = angleBetwV1andV2(triangleNormal, vecPointToCam);
                    if(angle > texParams.angleHardThreshold)
                        continue;
                }

                const int w = mp.getWidth(camId);
                const int h = mp.getHeight(camId);

                const Mesh::triangle_proj tProj = mesh->getTriangleProjection(triangleID, mp, camId, w, h);
                const int nbVertex = mesh->getTriangleNbVertexInImage(mp, tProj, camId, 20);
                if(nbVertex == 0)
                    // No triangle vertex in the image
                    continue;

                const double area = mesh->computeTriangleProjectionArea(tProj);
                const double score = area * double(verticesSupport);
                scorePerCamId.emplace_back(nbVertex, score, camId);
            }
            if (scorePerCamId.empty())
            {
                // triangle without visibility
                ALICEVISION_LOG_TRACE("No visibility for triangle " << triangleID << " in texture atlas " << atlasID << " after scoring!!");
                continue;
            }

            std::sort(scorePerCamId.begin(), scorePerCamId.end(), std::greater<ScoreCamId>());
            const double minScore = texParams.bestScoreThreshold * std::get<1>(scorePerCamId.front()); // bestScoreThreshold * bestScore
            const bool bestIsPartial = (std::get<0>(scorePerCamId.front()) < 3);

            int nbContribMax = std::min(texParams.multiBandNbContrib.back(), static_cast<int>(scorePerCamId.size()));
            int nbCumulatedVertices = 0;
            int band = 0;
            for(int contrib = 0; nbCumulatedVertices < 3 * nbContribMax && contrib < nbContribMax; ++contrib)
            {
                nbCumulatedVertices += std::get<0>(scorePerCamId[contrib]);
                if (!bestIsPartial && contrib != 0)
                {
                    if(std::get<1>(scorePerCamId[contrib]) < minScore)
                    {
                        // The best image fully see the triangle and has a much better score, so only rely on the first ones
                        break;
                    }
                }

                //for the camera camId : add triangle score to the corresponding texture, at the right frequency band
                const int camId = std::get<2>(scorePerCamId[contrib]);
                const int triangleScore = std::get<1>(scorePerCamId[contrib]);
                auto& camContribution = contributionsPerCamera[camId];
                if(camContribution.find(atlasID) == camContribution.end())
                    camContribution[atlasID].resize(texParams.nbBand);
                camContribution.at(atlasID)[band].emplace_back(triangleID, triangleScore);

                if(contrib + 1 == texParams.multiBandNbContrib[band])
                {
                    ++band;
                }
            }
        }
    }

    ALICEVISION_LOG_INFO("Reading pixel color.");

    //pyramid of atlases frequency bands
    std::map<AtlasIndex, AccuPyramid> accuPyramids;
    for(std::size_t atlasID: atlasIDs)
        accuPyramids[atlasID].init(texParams.nbBand, texParams.textureSide, texParams.textureSide);

    //for each camera, for each texture, iterate over triangles and fill the accuPyramids map
    for(int camId = 0; camId < contributionsPerCamera.size(); ++camId)
    {
        const std::map<AtlasIndex, std::vector<ScorePerTriangle>>& cameraContributions = contributionsPerCamera[camId];

        if(cameraContributions.empty())
        {
            ALICEVISION_LOG_INFO("- camera " << mp.getViewId(camId) << " (" << camId + 1 << "/" << mp.ncams << ") unused.");
            continue;
        }
        ALICEVISION_LOG_INFO("- camera " << mp.getViewId(camId) << " (" << camId + 1 << "/" << mp.ncams << ") with contributions to " << cameraContributions.size() << " texture files:");

        // Load camera image from cache
        mvsUtils::ImagesCache<ImageRGBf>::ImgSharedPtr imgPtr = imageCache.getImg_sync(camId);
        const ImageRGBf& camImg = *imgPtr;

        // Calculate laplacianPyramid
        std::vector<ImageRGBf> pyramidL; //laplacian pyramid
        laplacianPyramid(pyramidL, camImg, texParams.nbBand, texParams.multiBandDownscale);

        // for each output texture file
        for(const auto& c : cameraContributions)
        {
            AtlasIndex atlasID = c.first;
            ALICEVISION_LOG_INFO("  - Texture file: " << atlasID + 1);
            //for each frequency band
            for(int band = 0; band < c.second.size(); ++band)
            {
                const ScorePerTriangle& trianglesId = c.second[band];
                ALICEVISION_LOG_INFO("      - band " << band + 1 << ": " << trianglesId.size() << " triangles.");

                // for each triangle
                #pragma omp parallel for
                for(int ti = 0; ti < trianglesId.size(); ++ti)
                {
                    const unsigned int triangleId = std::get<0>(trianglesId[ti]);
                    const float triangleScore = texParams.useScore ? std::get<1>(trianglesId[ti]) : 1.0f;
                    // retrieve triangle 3D and UV coordinates
                    Point2d triPixs[3];
                    Point3d triPts[3];
                    auto& triangleUvIds = mesh->trisUvIds[triangleId];
                    // compute the Bottom-Left minima of the current UDIM for [0,1] range remapping
                    Point2d udimBL;
                    StaticVector<Point2d>& uvCoords = mesh->uvCoords;
                    udimBL.x = std::floor(std::min(std::min(uvCoords[triangleUvIds[0]].x, uvCoords[triangleUvIds[1]].x), uvCoords[triangleUvIds[2]].x));
                    udimBL.y = std::floor(std::min(std::min(uvCoords[triangleUvIds[0]].y, uvCoords[triangleUvIds[1]].y), uvCoords[triangleUvIds[2]].y));

                    for(int k = 0; k < 3; ++k)
                    {
                       const int pointIndex = mesh->tris[triangleId].v[k];
                       triPts[k] = mesh->pts[pointIndex];                               // 3D coordinates
                       const int uvPointIndex = triangleUvIds.m[k];
                       Point2d uv = uvCoords[uvPointIndex];
                       // UDIM: remap coordinates between [0,1]
                       uv = uv - udimBL;

                       triPixs[k] = uv * texParams.textureSide;   // UV coordinates
                    }

                    // compute triangle bounding box in pixel indexes
                    // min values: floor(value)
                    // max values: ceil(value)
                    Pixel LU, RD;
                    LU.x = static_cast<int>(std::floor(std::min(std::min(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
                    LU.y = static_cast<int>(std::floor(std::min(std::min(triPixs[0].y, triPixs[1].y), triPixs[2].y)));
                    RD.x = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
                    RD.y = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].y, triPixs[1].y), triPixs[2].y)));

                    // sanity check: clamp values to [0; textureSide]
                    int texSide = static_cast<int>(texParams.textureSide);
                    LU.x = clamp(LU.x, 0, texSide);
                    LU.y = clamp(LU.y, 0, texSide);
                    RD.x = clamp(RD.x, 0, texSide);
                    RD.y = clamp(RD.y, 0, texSide);

                    // iterate over pixels of the triangle's bounding box
                    for(int y = LU.y; y < RD.y; ++y)
                    {
                       for(int x = LU.x; x < RD.x; ++x)
                       {
                           Pixel pix(x, y); // top-left corner of the pixel
                           Point2d barycCoords;

                           // test if the pixel is inside triangle
                           // and retrieve its barycentric coordinates
                           if(!isPixelInTriangle(triPixs, pix, barycCoords))
                           {
                               continue;
                           }

                           // remap 'y' to image coordinates system (inverted Y axis)
                           const unsigned int y_ = (texParams.textureSide - 1) - y;
                           // 1D pixel index
                           unsigned int xyoffset = y_ * texParams.textureSide + x;
                           // get 3D coordinates
                           Point3d pt3d = barycentricToCartesian(triPts, barycCoords);
                           // get 2D coordinates in source image
                           Point2d pixRC;
                           mp.getPixelFor3DPoint(&pixRC, pt3d, camId);
                           // exclude out of bounds pixels
                           if(!mp.isPixelInImage(pixRC, camId))
                               continue;

                           // If the color is pure zero (ie. no contributions), we consider it as an invalid pixel.
                           if(camImg.getInterpolateColor(pixRC) == ColorRGBf(0.f, 0.f, 0.f))
                               continue;

                           // Fill the accumulated pyramid for this pixel
                           // each frequency band also contributes to lower frequencies (higher band indexes)
                           AccuPyramid& accuPyramid = accuPyramids.at(atlasID);
                           for(std::size_t bandContrib = band; bandContrib < pyramidL.size(); ++bandContrib)
                           {
                               int downscaleCoef = std::pow(texParams.multiBandDownscale, bandContrib);
                               AccuImage& accuImage = accuPyramid.pyramid[bandContrib];

                               // fill the accumulated color map for this pixel
                               accuImage.img[xyoffset] += pyramidL[bandContrib].getInterpolateColor(pixRC/downscaleCoef) * triangleScore;
                               accuImage.imgCount[xyoffset] += triangleScore;
                           }
                       }
                    }
                }
            }
        }
    }

    //calculate atlas texture in the first level of the pyramid (avoid creating a new buffer)
    //debug mode : write all the frequencies levels for each texture
    for(std::size_t atlasID : atlasIDs)
    {
        AccuPyramid& accuPyramid = accuPyramids.at(atlasID);
        AccuImage& atlasTexture = accuPyramid.pyramid[0];
        ALICEVISION_LOG_INFO("Create texture " << atlasID + 1);

#if TEXTURING_MBB_DEBUG
        {
            // write the number of contribution per atlas frequency bands
            if(!texParams.useScore)
            {
                for(std::size_t level = 0; level < accuPyramid.pyramid.size(); ++level)
                {
                    AccuImage& atlasLevelTexture =  accuPyramid.pyramid[level];

                    //write the number of contributions for each texture
                    std::vector<float> imgContrib(textureSize);

                    for(unsigned int yp = 0; yp < texParams.textureSide; ++yp)
                    {
                        unsigned int yoffset = yp * texParams.textureSide;
                        for(unsigned int xp = 0; xp < texParams.textureSide; ++xp)
                        {
                            unsigned int xyoffset = yoffset + xp;
                            imgContrib[xyoffset] = atlasLevelTexture.imgCount[xyoffset];
                        }
                    }

                    const std::string textureName = "contrib_" + std::to_string(1001 + atlasID) + std::string("_") + std::to_string(level) + std::string(".") + EImageFileType_enumToString(textureFileType); // starts at '1001' for UDIM compatibility
                    bfs::path texturePath = outPath / textureName;

                    using namespace imageIO;
                    OutputFileColorSpace colorspace(EImageColorSpace::SRGB, EImageColorSpace::AUTO);
                    if(texParams.convertLAB)
                        colorspace.from = EImageColorSpace::LAB;
                    writeImage(texturePath.string(), texParams.textureSide, texParams.textureSide, imgContrib, EImageQuality::OPTIMIZED, colorspace);
                }
            }
        }
#endif

        ALICEVISION_LOG_INFO("  - Computing final (average) color.");
        for(unsigned int yp = 0; yp < texParams.textureSide; ++yp)
        {
            unsigned int yoffset = yp * texParams.textureSide;
            for(unsigned int xp = 0; xp < texParams.textureSide; ++xp)
            {
                unsigned int xyoffset = yoffset + xp;

                // If the imgCount is valid on the first band, it will be valid on all the other bands
                if(atlasTexture.imgCount[xyoffset] == 0)
                    continue;

                atlasTexture.img[xyoffset] /= atlasTexture.imgCount[xyoffset];
                atlasTexture.imgCount[xyoffset] = 1;

                for(std::size_t level = 1; level < accuPyramid.pyramid.size(); ++level)
                {
                    AccuImage& atlasLevelTexture =  accuPyramid.pyramid[level];
                    atlasLevelTexture.img[xyoffset] /= atlasLevelTexture.imgCount[xyoffset];
                }
            }
        }

#if TEXTURING_MBB_DEBUG
        {
            //write each frequency band, for each texture
            for(std::size_t level = 0; level < accuPyramid.pyramid.size(); ++level)
            {
                AccuImage& atlasLevelTexture =  accuPyramid.pyramid[level];
                writeTexture(atlasLevelTexture, atlasID, outPath, textureFileType, level);
            }

        }
#endif

        // Fuse frequency bands into the first buffer, calculate final texture
        for(unsigned int yp = 0; yp < texParams.textureSide; ++yp)
        {
            unsigned int yoffset = yp * texParams.textureSide;
            for(unsigned int xp = 0; xp < texParams.textureSide; ++xp)
            {
                unsigned int xyoffset = yoffset + xp;
                for(std::size_t level = 1; level < accuPyramid.pyramid.size(); ++level)
                {
                    AccuImage& atlasLevelTexture =  accuPyramid.pyramid[level];
                    atlasTexture.img[xyoffset] += atlasLevelTexture.img[xyoffset];
                }
            }
        }
        writeTexture(atlasTexture, atlasID, outPath, textureFileType, -1);
    }
}

void Texturing::generateNormalAndHeightMaps(const mvsUtils::MultiViewParams& mp, const Mesh& denseMesh,
                                            const bfs::path& outPath, const mesh::BumpMappingParams& bumpMappingParams)
{
    GEO::Mesh geoDenseMesh;
    toGeoMesh(denseMesh, geoDenseMesh);
    GEO::compute_normals(geoDenseMesh);
    GEO::MeshFacetsAABB denseMeshAABB(geoDenseMesh); // warning: mesh_reorder called inside

    GEO::Mesh geoSparseMesh;
    toGeoMesh(*mesh, geoSparseMesh);
    GEO::compute_normals(geoSparseMesh);

    mvsUtils::ImagesCache<ImageRGBf> imageCache(mp, imageIO::EImageColorSpace::NO_CONVERSION);
    
    for(size_t atlasID = 0; atlasID < _atlases.size(); ++atlasID)
        _generateNormalAndHeightMaps(mp, denseMeshAABB, geoSparseMesh, atlasID, imageCache, outPath, bumpMappingParams);
}



void Texturing::writeTexture(AccuImage& atlasTexture, const std::size_t atlasID, const boost::filesystem::path &outPath,
                             imageIO::EImageFileType textureFileType, const int level)
{
    unsigned int outTextureSide = texParams.textureSide;
    // WARNING: we modify the "imgCount" to apply the padding (to avoid the creation of a new buffer)
    // edge padding (dilate gutter)
    if(!texParams.fillHoles && texParams.padding > 0 && level < 0)
    {
        const unsigned int padding = texParams.padding * 3;
        ALICEVISION_LOG_INFO("  - Edge padding (" << padding << " pixels).");

        // Init valid values to 1
        for(unsigned int y = 0; y < outTextureSide; ++y)
        {
            unsigned int yoffset = y * outTextureSide;
            for(unsigned int x = 0; x < outTextureSide; ++x)
            {
                unsigned int xyoffset = yoffset + x;
                if(atlasTexture.imgCount[xyoffset] > 0)
                    atlasTexture.imgCount[xyoffset] = 1;
            }
        }

        //up-left to bottom-right
        for(unsigned int y = 1; y < outTextureSide-1; ++y)
        {
            unsigned int yoffset = y * outTextureSide;
            for(unsigned int x = 1; x < outTextureSide-1; ++x)
            {
                unsigned int xyoffset = yoffset + x;
                if(atlasTexture.imgCount[xyoffset] > 0)
                    continue;

                const int upCount = atlasTexture.imgCount[xyoffset - outTextureSide];
                const int leftCount = atlasTexture.imgCount[xyoffset - 1];
                //if pixel on the edge of a chart
                if(leftCount > 0)
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset - 1];
                    atlasTexture.imgCount[xyoffset] = - 1;
                }
                else if(upCount > 0)
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset - outTextureSide];
                    atlasTexture.imgCount[xyoffset] = - 1;
                }
                //
                else if (leftCount < 0 && - leftCount < padding && (upCount == 0 || leftCount > upCount))
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset - 1];
                    atlasTexture.imgCount[xyoffset] = leftCount - 1;
                }
                else if (upCount < 0 && - upCount < padding)
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset - outTextureSide];
                    atlasTexture.imgCount[xyoffset] = upCount - 1;
                }
            }
        }

        //bottom-right to up-left
        for(unsigned int y = 1; y < outTextureSide-1; ++y)
        {
            unsigned int yoffset = (outTextureSide - 1 - y) * outTextureSide;
            for(unsigned int x = 1; x < outTextureSide-1; ++x)
            {
                unsigned int xyoffset = yoffset + (outTextureSide - 1 - x);
                if(atlasTexture.imgCount[xyoffset] > 0)
                    continue;

                const int upCount = atlasTexture.imgCount[xyoffset - outTextureSide];
                const int downCount = atlasTexture.imgCount[xyoffset + outTextureSide];
                const int rightCount = atlasTexture.imgCount[xyoffset + 1];
                const int leftCount = atlasTexture.imgCount[xyoffset - 1];
                if(rightCount > 0)
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset + 1];
                    atlasTexture.imgCount[xyoffset] = - 1;
                }
                else if(downCount > 0)
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset + outTextureSide];
                    atlasTexture.imgCount[xyoffset] = - 1;
                }
                else if ((rightCount < 0 && - rightCount < padding) &&
                         (leftCount == 0 || rightCount > leftCount) &&
                         (downCount == 0 || rightCount >= downCount)
                         )
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset + 1];
                    atlasTexture.imgCount[xyoffset] = rightCount - 1;
                }
                else if ((downCount < 0 && - downCount < padding) &&
                         (upCount == 0 || downCount > upCount)
                         )
                {
                    atlasTexture.img[xyoffset] = atlasTexture.img[xyoffset + outTextureSide];
                    atlasTexture.imgCount[xyoffset] = downCount - 1;
                }
            }
        }
    }

    // texture holes filling
    if(texParams.fillHoles)
    {
        ALICEVISION_LOG_INFO("  - Filling texture holes.");
        std::vector<float> alphaBuffer(atlasTexture.img.size());
        for(unsigned int yp = 0; yp < texParams.textureSide; ++yp)
        {
            unsigned int yoffset = yp * texParams.textureSide;
            for(unsigned int xp = 0; xp < texParams.textureSide; ++xp)
            {
                unsigned int xyoffset = yoffset + xp;
                alphaBuffer[xyoffset] = atlasTexture.imgCount[xyoffset] ? 1 : 0;
            }
        }
        imageAlgo::fillHoles(atlasTexture.img, alphaBuffer);
        alphaBuffer.clear();
    }

    // downscale texture if required
    if(texParams.downscale > 1)
    {
        ImageRGBf resizedColorBuffer;

        ALICEVISION_LOG_INFO("  - Downscaling texture (" << texParams.downscale << "x).");
        imageAlgo::resizeImage(texParams.downscale, atlasTexture.img, resizedColorBuffer);
        std::swap(resizedColorBuffer, atlasTexture.img);
    }

    const std::string textureName = "texture_" + std::to_string(1001 + atlasID) + (level < 0 ? "" : "_" + std::to_string(level)) + "." + imageIO::EImageFileType_enumToString(textureFileType); // starts at '1001' for UDIM compatibility
    bfs::path texturePath = outPath / textureName;
    ALICEVISION_LOG_INFO("  - Writing texture file: " << texturePath.string());

    using namespace imageIO;
    OutputFileColorSpace colorspace(texParams.processColorspace, EImageColorSpace::AUTO);
    writeImage(texturePath.string(), atlasTexture.img, EImageQuality::OPTIMIZED, colorspace);
}


void Texturing::clear()
{
    delete mesh;
    mesh = nullptr;
}

void Texturing::loadWithAtlas(const std::string& filepath, bool flipNormals)
{
    // Clear internal data
    clear();
    mesh = new Mesh();
    // Load .obj
    mesh->load(filepath);

    // Handle normals flipping
    if(flipNormals)
        mesh->invertTriangleOrientations();

    // Fill atlases (1 atlas per material) with corresponding rectangles
    // if no material, create only one atlas with all triangles
    _atlases.resize(std::max(1, mesh->nmtls));
    for(int triangleID = 0; triangleID < mesh->trisMtlIds().size(); ++triangleID)
    {
        unsigned int atlasID = mesh->nmtls ? mesh->trisMtlIds()[triangleID] : 0;
        _atlases[atlasID].push_back(triangleID);
    }
}

void Texturing::remapVisibilities(EVisibilityRemappingMethod remappingMethod,
                                  const mvsUtils::MultiViewParams& mp, const Mesh& refMesh)
{
    if(refMesh.pointsVisibilities.empty() &&
       (remappingMethod & mesh::EVisibilityRemappingMethod::Pull ||
        remappingMethod & mesh::EVisibilityRemappingMethod::Push))
    {
        throw std::runtime_error("Texturing: Cannot remap visibilities as there is no reference points.");
    }

    // remap visibilities from the reference onto the mesh
    if(remappingMethod & mesh::EVisibilityRemappingMethod::Pull)
    {
        remapMeshVisibilities_pullVerticesVisibility(refMesh, *mesh);
    }
    if(remappingMethod & mesh::EVisibilityRemappingMethod::Push)
    {
        remapMeshVisibilities_pushVerticesVisibilityToTriangles(refMesh, *mesh);
    }
    if(remappingMethod & EVisibilityRemappingMethod::MeshItself)
    {
        remapMeshVisibilities_meshItself(mp, *mesh);
    }
    if(mesh->pointsVisibilities.empty())
    {
        throw std::runtime_error("No visibility after visibility remapping.");
    }
}

void Texturing::replaceMesh(const std::string& otherMeshPath, bool flipNormals)
{
    // keep previous mesh/visibilities as reference
    Mesh* refMesh = mesh;
    // set pointers to null to avoid deallocation by 'loadFromObj'
    mesh->pointsVisibilities.resize(0);
    mesh = nullptr;
    
    // load input obj file
    loadWithAtlas(otherMeshPath, flipNormals);
    // allocate pointsVisibilities for new internal mesh
    mesh->pointsVisibilities = PointsVisibility();
    // remap visibilities from reconstruction onto input mesh
    if(texParams.visibilityRemappingMethod & EVisibilityRemappingMethod::Pull)
        remapMeshVisibilities_pullVerticesVisibility(*refMesh, *mesh);
    if (texParams.visibilityRemappingMethod & EVisibilityRemappingMethod::Push)
        remapMeshVisibilities_pushVerticesVisibilityToTriangles(*refMesh, *mesh);
    if(mesh->pointsVisibilities.empty())
        throw std::runtime_error("No visibility after visibility remapping.");

    // delete ref mesh and visibilities
    delete refMesh;
}

void Texturing::unwrap(mvsUtils::MultiViewParams& mp, EUnwrapMethod method)
{
    if(method == mesh::EUnwrapMethod::Basic)
    {
        // generate UV coordinates based on automatic uv atlas
        generateUVsBasicMethod(mp);
    }
    else
    {
        GEO::initialize();
        GEO::Mesh geoMesh;
        toGeoMesh(*mesh, geoMesh);

        // perform parametrization with Geogram
        const GEO::ChartParameterizer param = (method == mesh::EUnwrapMethod::ABF) ? GEO::PARAM_ABF : GEO::PARAM_SPECTRAL_LSCM;

        ALICEVISION_LOG_INFO("Start mesh atlasing (using Geogram " << EUnwrapMethod_enumToString(method) << ").");
        GEO::mesh_make_atlas(geoMesh, 45.0, param);
        ALICEVISION_LOG_INFO("Mesh atlasing done.");

        // TODO: retrieve computed UV coordinates and find a way to update internal data
        // GEO::Attribute<double> uvs(in.facet_corners.attributes(), "tex_coord");
        // uvCoords.reserve(in.vertices.nb());
        // TODO: fill trisUvsIds
        // trisUvIds.reserve(me->tris->size());

        // Meanwhile,
        // use a temporary obj file to save result - Geogram merges common UV coordinates per facet corner -
        // and reload it
        const std::string tmpObjPath = (bfs::temp_directory_path() / bfs::unique_path()).string() + ".obj";
        // save temp mesh with UVs
        GEO::mesh_save(geoMesh, tmpObjPath);
        // replace initial mesh
        replaceMesh(tmpObjPath);
        // remove temp mesh
        bfs::remove(tmpObjPath);
    }
}

void Texturing::saveAs(const bfs::path& dir, const std::string& basename, 
    EFileType meshFileType, 
    imageIO::EImageFileType textureFileType,
    const BumpMappingParams& bumpMappingParams)
{
    const std::string meshFileTypeStr = EFileType_enumToString(meshFileType);
    const std::string filepath = (dir / (basename + "." + meshFileTypeStr)).string();

    ALICEVISION_LOG_INFO("Save " << filepath << " mesh file");

    if (_atlases.empty())
    {
        return;
    }

    aiScene scene;

    scene.mRootNode = new aiNode;

    scene.mMeshes = new aiMesh*[_atlases.size()];
    scene.mNumMeshes = _atlases.size();
    scene.mRootNode->mMeshes = new unsigned int[_atlases.size()];
    scene.mRootNode->mNumMeshes = _atlases.size();
    scene.mMaterials = new aiMaterial*[_atlases.size()];
    scene.mNumMaterials = _atlases.size();

    // write faces per texture atlas
    for(std::size_t atlasId = 0; atlasId < _atlases.size(); ++atlasId)
    {      
        // starts at '1001' for UDIM compatibility
        const std::size_t textureId = 1001 + atlasId;
        const std::string texturePath = "texture_" + std::to_string(textureId) + "." + imageIO::EImageFileType_enumToString(textureFileType);

        //Set material for this atlas
        const aiVector3D valcolor(0.6, 0.6, 0.6);
        const aiVector3D valspecular(0.0, 0.0, 0.0);
        const double shininess = 0.0;
        const aiString texFile(texturePath);
        const aiString texName(std::to_string(textureId));

        scene.mMaterials[atlasId] = new aiMaterial;
        scene.mMaterials[atlasId]->AddProperty(&valcolor, 1, AI_MATKEY_COLOR_AMBIENT);
        scene.mMaterials[atlasId]->AddProperty(&valcolor, 1, AI_MATKEY_COLOR_DIFFUSE);
        scene.mMaterials[atlasId]->AddProperty(&valspecular, 1, AI_MATKEY_COLOR_SPECULAR);
        scene.mMaterials[atlasId]->AddProperty(&shininess, 1, AI_MATKEY_SHININESS);
        scene.mMaterials[atlasId]->AddProperty(&texFile, AI_MATKEY_TEXTURE_DIFFUSE(0));
        scene.mMaterials[atlasId]->AddProperty(&texName, AI_MATKEY_NAME);

        // Color Mapping
        if(textureFileType != imageIO::EImageFileType::NONE)
        {
            const aiString texFile(texturePath);
            scene.mMaterials[atlasId]->AddProperty(&texFile, AI_MATKEY_TEXTURE_DIFFUSE(0));
        }

        // Displacement Mapping
        if(bumpMappingParams.displacementFileType != imageIO::EImageFileType::NONE)
        {
            const aiString texFileHeightMap("Displacement_" + std::to_string(textureId) + "." +EImageFileType_enumToString(bumpMappingParams.bumpMappingFileType));
            scene.mMaterials[atlasId]->AddProperty(&texFileHeightMap, AI_MATKEY_TEXTURE_DISPLACEMENT(0));
        }
        
        // Bump Mapping
        if(bumpMappingParams.bumpType == EBumpMappingType::Normal && bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::NONE)
        {
            const aiString texFileNormalMap("Normal_" + std::to_string(textureId) + "." + EImageFileType_enumToString(bumpMappingParams.bumpMappingFileType));
            scene.mMaterials[atlasId]->AddProperty(&texFileNormalMap, AI_MATKEY_TEXTURE_NORMALS(0));
        }
        else if(bumpMappingParams.bumpType == EBumpMappingType::Height && bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::NONE)
        {
            const aiString texFileHeightMap("Bump_" + std::to_string(textureId) + "." + EImageFileType_enumToString(bumpMappingParams.displacementFileType));
            scene.mMaterials[atlasId]->AddProperty(&texFileHeightMap, AI_MATKEY_TEXTURE_HEIGHT(0));
        }

        scene.mRootNode->mMeshes[atlasId] = atlasId;
        scene.mMeshes[atlasId] = new aiMesh;
        aiMesh * aimesh = scene.mMeshes[atlasId];
        aimesh->mMaterialIndex = atlasId;
        aimesh->mNumUVComponents[0] = 2;

        //Assimp does not allow vertex indices different from uv indices
        //So we need to group and duplicate
        std::map<std::pair<int, int>, int> unique_pairs;
        for(const auto triangleID : _atlases[atlasId])
        {
            for (int k = 0; k < 3; ++k)
            {
                int vertexId = mesh->tris[triangleID].v[k];
                int uvId = mesh->trisUvIds[triangleID].m[k];

                std::pair<int, int> p = std::make_pair(vertexId, uvId);
                unique_pairs[p] = -1;
            }
        }

        aimesh->mNumVertices = unique_pairs.size();
        aimesh->mVertices = new aiVector3D[unique_pairs.size()];
        aimesh->mTextureCoords[0] = new aiVector3D[unique_pairs.size()];

        int index = 0;
        for (auto & p : unique_pairs)
        {
            int vertexId = p.first.first;
            int uvId = p.first.second;

            aimesh->mVertices[index].x = mesh->pts[vertexId].x;
            aimesh->mVertices[index].y = mesh->pts[vertexId].y;
            aimesh->mVertices[index].z = mesh->pts[vertexId].z;

            aimesh->mTextureCoords[0][index].x = mesh->uvCoords[uvId].x;
            aimesh->mTextureCoords[0][index].y = mesh->uvCoords[uvId].y;
            aimesh->mTextureCoords[0][index].z = 0.0;

            p.second = index;

            ++index;
        }

        aimesh->mNumFaces = _atlases[atlasId].size();
        aimesh->mFaces = new aiFace[aimesh->mNumFaces];

        for(int i = 0; i < _atlases[atlasId].size(); ++i)
        {
            const int triangleId = _atlases[atlasId][i];

            aimesh->mFaces[i].mNumIndices = 3;
            aimesh->mFaces[i].mIndices = new unsigned int[3];

            for (int k = 0; k < 3; ++k)
            {
                const int vertexId = mesh->tris[triangleId].v[k];
                const int uvId = mesh->trisUvIds[triangleId].m[k];

                const std::pair<int, int> p = std::make_pair(vertexId, uvId);
                aimesh->mFaces[i].mIndices[k] = unique_pairs[p];
            }
        }
    }

    std::string formatId = meshFileTypeStr;
    unsigned int pPreprocessing = 0u;
    // If gltf, use gltf 2.0
    if(meshFileType == EFileType::GLTF)
    {
        formatId = "gltf2";
        // Flip UVs when exporting (issue with UV origin for gltf2)
        // https://github.com/around-media/ue4-custom-prompto/commit/044dbad90fc2172f4c5a8b67c779b80ceace5e1e
        pPreprocessing |= aiPostProcessSteps::aiProcess_FlipUVs | aiProcess_GenNormals;
    }

    Assimp::Exporter exporter;
    exporter.Export(&scene, formatId, filepath, pPreprocessing);

    ALICEVISION_LOG_INFO("Save mesh to " << meshFileTypeStr << " done.");
}


inline GEO::vec3 mesh_facet_interpolate_normal_at_point(const GEO::Mesh& mesh, GEO::index_t f, const GEO::vec3& p)
{
    const GEO::index_t v0 = mesh.facets.vertex(f, 0);
    const GEO::index_t v1 = mesh.facets.vertex(f, 1);
    const GEO::index_t v2 = mesh.facets.vertex(f, 2);

    const GEO::vec3 p0 = mesh.vertices.point(v0);
    const GEO::vec3 p1 = mesh.vertices.point(v1);
    const GEO::vec3 p2 = mesh.vertices.point(v2);

    const GEO::vec3 n0 = GEO::normalize(GEO::Geom::mesh_vertex_normal(mesh, v0));
    const GEO::vec3 n1 = GEO::normalize(GEO::Geom::mesh_vertex_normal(mesh, v1));
    const GEO::vec3 n2 = GEO::normalize(GEO::Geom::mesh_vertex_normal(mesh, v2));

    GEO::vec3 barycCoords;
    GEO::vec3 closestPoint;
    GEO::Geom::point_triangle_squared_distance<GEO::vec3>(p, p0, p1, p2, closestPoint, barycCoords.x, barycCoords.y,
                                                          barycCoords.z);

    const GEO::vec3 n = barycCoords.x * n0 + barycCoords.y * n1 + barycCoords.z * n2;

    return GEO::normalize(n);
}

inline GEO::vec3 mesh_facet_interpolate_normal_at_point(const StaticVector<Point3d>& ptsNormals, const Mesh& mesh,
                                                        GEO::index_t f, const GEO::vec3& p)
{
    const GEO::index_t v0 = (mesh.tris)[f].v[0];
    const GEO::index_t v1 = (mesh.tris)[f].v[1];
    const GEO::index_t v2 = (mesh.tris)[f].v[2];

    const GEO::vec3 p0 ((mesh.pts)[v0].x, (mesh.pts)[v0].y, (mesh.pts)[v0].z);
    const GEO::vec3 p1 ((mesh.pts)[v1].x, (mesh.pts)[v1].y, (mesh.pts)[v1].z);
    const GEO::vec3 p2 ((mesh.pts)[v2].x, (mesh.pts)[v2].y, (mesh.pts)[v2].z);

    const GEO::vec3 n0 (ptsNormals[v0].x, ptsNormals[v0].y, ptsNormals[v0].z);
    const GEO::vec3 n1 (ptsNormals[v1].x, ptsNormals[v1].y, ptsNormals[v1].z);
    const GEO::vec3 n2 (ptsNormals[v2].x, ptsNormals[v2].y, ptsNormals[v2].z);

    GEO::vec3 barycCoords;
    GEO::vec3 closestPoint;
    GEO::Geom::point_triangle_squared_distance<GEO::vec3>(p, p0, p1, p2, closestPoint, barycCoords.x, barycCoords.y,
                                                          barycCoords.z);

    const GEO::vec3 n = barycCoords.x * n0 + barycCoords.y * n1 + barycCoords.z * n2;

    return GEO::normalize(n);
}

template <class T, GEO::index_t DIM>
inline Eigen::Matrix<T, DIM, 1> toEigen(const GEO::vecng<DIM, T>& v)
{
    return Eigen::Matrix<T, DIM, 1>(v.data());
}

/**
 * @brief Compute a transformation matrix to convert coordinates in world space coordinates into the triangle space.
 * The triangle space is define by the Z-axis as the normal of the triangle,
 * the X-axis aligned with the horizontal line in the texture file (using texture/UV coordinates).
 *
 * @param[in] mesh: input mesh
 * @param[in] f: facet/triangle index
 * @param[in] triPts: UV Coordinates
 * @return Rotation matrix to convert from world space coordinates in the triangle space
 */
inline Eigen::Matrix3d computeTriangleTransform(const Mesh& mesh, int f, const Point2d* triPts)
{
    const Eigen::Vector3d p0 = toEigen((mesh.pts)[(mesh.tris)[f].v[0]]);
    const Eigen::Vector3d p1 = toEigen((mesh.pts)[(mesh.tris)[f].v[1]]);
    const Eigen::Vector3d p2 = toEigen((mesh.pts)[(mesh.tris)[f].v[2]]);

    const Eigen::Vector3d tX = (p1 - p0).normalized();                       // edge0 => local triangle X-axis
    const Eigen::Vector3d N = tX.cross((p2 - p0).normalized()).normalized(); // cross(edge0, edge1) => Z-axis

    // Correct triangle X-axis to be align with X-axis in the texture
    const GEO::vec2 t0 = GEO::vec2(triPts[0].m);
    const GEO::vec2 t1 = GEO::vec2(triPts[1].m);
    const GEO::vec2 tV = GEO::normalize(t1 - t0);
    const GEO::vec2 origNormal(1.0, 0.0); // X-axis in the texture
    const double tAngle = GEO::Geom::angle(tV, origNormal);
    Eigen::Matrix3d transform(Eigen::AngleAxisd(tAngle, N).toRotationMatrix());
    // Rotate triangle v0v1 axis around Z-axis, to get a X axis aligned with the 2d texture
    Eigen::Vector3d X = (transform * tX).normalized();

    const Eigen::Vector3d Y = N.cross(X).normalized(); // Y-axis

    Eigen::Matrix3d m;
    m.col(0) = X;
    m.col(1) = Y;
    m.col(2) = N;
    // const Eigen::Matrix3d mInv = m.inverse();
    const Eigen::Matrix3d mT = m.transpose();

    return mT;
}

inline void computeNormalHeight(const GEO::Mesh& mesh, double orientation, double t, GEO::index_t f,
                                const Eigen::Matrix3d& m, const GEO::vec3& q, const GEO::vec3& qA, const GEO::vec3& qB,
                                float& out_height, ColorRGBf& out_normal)
{
    GEO::vec3 intersectionPoint = t * qB + (1.0 - t) * qA;
    out_height = q.distance(intersectionPoint) * orientation;

    // Use facet normal
    // GEO::vec3 denseMeshNormal_f = normalize(GEO::Geom::mesh_facet_normal(mesh, f));
    // Use per pixel normal using weighted interpolation of the facet vertex normals
    const GEO::vec3 denseMeshNormal = mesh_facet_interpolate_normal_at_point(mesh, f, intersectionPoint);

    Eigen::Vector3d dNormal = m * toEigen(denseMeshNormal);
    dNormal.normalize();
    out_normal = ColorRGBf(dNormal(0), dNormal(1), dNormal(2));
}

void Texturing::_generateNormalAndHeightMaps(const mvsUtils::MultiViewParams& mp,
                                             const GEO::MeshFacetsAABB& denseMeshAABB, const GEO::Mesh& sparseMesh,
                                             size_t atlasID, mvsUtils::ImagesCache<ImageRGBf>& imageCache,
                                             const bfs::path& outPath, const mesh::BumpMappingParams& bumpMappingParams)
{
    ALICEVISION_LOG_INFO("Generating Height and Normal Maps for atlas " << atlasID + 1 << "/" << _atlases.size() << " ("
                                                                        << _atlases[atlasID].size() << " triangles).");

    std::vector<ColorRGBf> normalMap(texParams.textureSide * texParams.textureSide);
    std::vector<float> heightMap(texParams.textureSide * texParams.textureSide);
    const auto& triangles = _atlases[atlasID];

    // iterate over atlas' triangles
#pragma omp parallel for
    for(int ti = 0; ti < triangles.size(); ++ti)
    {
        const unsigned int triangleId = triangles[ti];
        //  const Point3d __triangleNormal_ = me->computeTriangleNormal(triangleId).normalize();
        //  const GEO::vec3 __triangleNormal(__triangleNormal_.x, __triangleNormal_.y, __triangleNormal_.z);
        const double minEdgeLength = mesh->computeTriangleMinEdgeLength(triangleId);
        // const GEO::vec3 scaledTriangleNormal = triangleNormal * minEdgeLength;

        // retrieve triangle 3D and UV coordinates
        Point2d triPixs[3];
        Point3d triPts[3];
        auto& triangleUvIds = mesh->trisUvIds[triangleId];

        // compute the Bottom-Left minima of the current UDIM for [0,1] range remapping
        Point2d udimBL;
        StaticVector<Point2d>& uvCoords = mesh->uvCoords;
        udimBL.x = std::floor(std::min(std::min(uvCoords[triangleUvIds[0]].x, uvCoords[triangleUvIds[1]].x),uvCoords[triangleUvIds[2]].x));
        udimBL.y = std::floor(std::min(std::min(uvCoords[triangleUvIds[0]].y, uvCoords[triangleUvIds[1]].y),uvCoords[triangleUvIds[2]].y));

        for(int k = 0; k < 3; k++)
        {
            const int pointIndex = (mesh->tris)[triangleId].v[k];
            triPts[k] = (mesh->pts)[pointIndex]; // 3D coordinates
            const int uvPointIndex = triangleUvIds.m[k];

            Point2d uv = uvCoords[uvPointIndex];
            // UDIM: remap coordinates between [0,1]
            uv = uv - udimBL;

            triPixs[k] = uv * texParams.textureSide; // UV coordinates
        }

        // compute triangle bounding box in pixel indexes
        // min values: floor(value)
        // max values: ceil(value)
        Pixel LU, RD;
        LU.x = static_cast<int>(std::floor(std::min(std::min(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
        LU.y = static_cast<int>(std::floor(std::min(std::min(triPixs[0].y, triPixs[1].y), triPixs[2].y)));
        RD.x = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
        RD.y = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].y, triPixs[1].y), triPixs[2].y)));

        // sanity check: clamp values to [0; textureSide]
        int texSide = static_cast<int>(texParams.textureSide);
        LU.x = clamp(LU.x, 0, texSide);
        LU.y = clamp(LU.y, 0, texSide);
        RD.x = clamp(RD.x, 0, texSide);
        RD.y = clamp(RD.y, 0, texSide);

        const Eigen::Matrix3d worldToTriangleMatrix = computeTriangleTransform(*mesh, triangleId, triPixs);
        // const Point3d triangleNormal = me->computeTriangleNormal(triangleId);

        // iterate over bounding box's pixels
        for(int y = LU.y; y < RD.y; ++y)
        {
            for(int x = LU.x; x < RD.x; ++x)
            {
                Pixel pix(x, y); // top-left corner of the pixel
                Point2d barycCoords;

                // test if the pixel is inside triangle
                // and retrieve its barycentric coordinates
                if(!isPixelInTriangle(triPixs, pix, barycCoords))
                {
                    continue;
                }

                // remap 'y' to image coordinates system (inverted Y axis)
                const unsigned int y_ = (texParams.textureSide - 1) - y;
                // 1D pixel index
                unsigned int xyoffset = y_ * texParams.textureSide + x;
                // get 3D coordinates
                // Point3d pt3d = barycentricToCartesian(triPts, Point2d(barycCoords.z, barycCoords.y));
                Point3d pt3d = barycentricToCartesian(triPts, barycCoords);
                GEO::vec3 q(pt3d.x, pt3d.y, pt3d.z);

                // Texel normal (weighted normal from the 3 vertices normals), instead of face normal for better
                // transitions (reduce seams)
                const GEO::vec3 triangleNormal_p = mesh_facet_interpolate_normal_at_point(sparseMesh, triangleId, q);
                // const GEO::vec3 triangleNormal_p = GEO::vec3(triangleNormal.m); // to use the triangle normal instead
                const GEO::vec3 scaledTriangleNormal = triangleNormal_p * minEdgeLength * 10;

                const double epsilon = 0.00001;
                GEO::vec3 qA1 = q - (scaledTriangleNormal * epsilon);
                GEO::vec3 qB1 = q + scaledTriangleNormal;
                double t = 0.0;
                GEO::index_t f = 0.0;
                bool intersection = denseMeshAABB.segment_nearest_intersection(qA1, qB1, t, f);
                if(intersection)
                {
                    computeNormalHeight(*denseMeshAABB.mesh(), 1.0, t, f, worldToTriangleMatrix, q, qA1, qB1,
                                        heightMap[xyoffset], normalMap[xyoffset]);
                }
                else
                {
                    GEO::vec3 qA2 = q + (scaledTriangleNormal * epsilon);
                    GEO::vec3 qB2 = q - scaledTriangleNormal;
                    bool intersection = denseMeshAABB.segment_nearest_intersection(qA2, qB2, t, f);
                    if(intersection)
                    {
                        computeNormalHeight(*denseMeshAABB.mesh(), -1.0, t, f, worldToTriangleMatrix, q, qA2, qB2,
                                            heightMap[xyoffset], normalMap[xyoffset]);
                    }
                    else
                    {
                        heightMap[xyoffset] = 0.0f;
                        normalMap[xyoffset] = ColorRGBf(0.0f, 0.0f, 0.0f);
                    }
                }
            }
        }
    }

    // Save Normal Map
    if(bumpMappingParams.bumpType == EBumpMappingType::Normal && bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::NONE)
    {
        unsigned int outTextureSide = texParams.textureSide;
        // downscale texture if required
        if(texParams.downscale > 1)
        {
            ALICEVISION_LOG_INFO("Downscaling normal map (" << texParams.downscale << "x).");
            std::vector<ColorRGBf> resizedBuffer;
            outTextureSide = texParams.textureSide / texParams.downscale;
            // use nearest-neighbor interpolation to avoid meaningless interpolation of normals on edges.
            const std::string interpolation = "box";
            imageAlgo::resizeImage(texParams.textureSide, texParams.textureSide, texParams.downscale, normalMap,
                                   resizedBuffer, interpolation);

            std::swap(resizedBuffer, normalMap);
        }

        // X: -1 to +1 : Red : 0 to 255
        // Y: -1 to +1 : Green : 0 to 255
        // Z: 0 to -1 : Blue : 128 to 255 OR 0 to 255 (like Blender)
        for(unsigned int i = 0; i < normalMap.size(); ++i)
            // normalMap[i] = ColorRGBf(normalMap[i].r * 0.5 + 0.5, normalMap[i].g * 0.5 + 0.5, normalMap[i].b); // B:
            // 0:+1 => 0-255
            normalMap[i] = ColorRGBf(normalMap[i].r * 0.5 + 0.5, normalMap[i].g * 0.5 + 0.5,
                                    normalMap[i].b * 0.5 + 0.5); // B: -1:+1 => 0-255 which means 0:+1 => 128-255

        const std::string name = "Normal_" + std::to_string(1001 + atlasID) + "." + EImageFileType_enumToString(bumpMappingParams.bumpMappingFileType);
        bfs::path normalMapPath = outPath / name;
        ALICEVISION_LOG_INFO("Writing normal map: " << normalMapPath.string());

        imageIO::OutputFileColorSpace outputColorSpace(imageIO::EImageColorSpace::NO_CONVERSION,imageIO::EImageColorSpace::NO_CONVERSION);
        imageIO::writeImage(normalMapPath.string(), outTextureSide, outTextureSide, normalMap, imageIO::EImageQuality::OPTIMIZED, outputColorSpace);
    }

    // Save Height Maps
    if(bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::NONE || bumpMappingParams.displacementFileType != imageIO::EImageFileType::NONE)
    {
        unsigned int outTextureSide = texParams.textureSide;
        if(texParams.downscale > 1)
        {
            ALICEVISION_LOG_INFO("Downscaling height map (" << texParams.downscale << "x).");
            std::vector<float> resizedBuffer;
            outTextureSide = texParams.textureSide / texParams.downscale;
            imageAlgo::resizeImage(texParams.textureSide, texParams.textureSide, texParams.downscale, heightMap,
                                   resizedBuffer);
            std::swap(resizedBuffer, heightMap);
        }

        // Height maps are only in .EXR at this time, so this will never be executed.
        // 
        //if(bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::EXR)
        //{
        //    // Y: [-1, 0, +1] => [0, 128, 255]
        //    for(unsigned int i = 0; i < heightMap.size(); ++i)
        //        heightMap[i] = heightMap[i] * 0.5 + 0.5;
        //}

        // Save Bump Map
        imageIO::OutputFileColorSpace outputColorSpace(imageIO::EImageColorSpace::AUTO);
        if(bumpMappingParams.bumpType == EBumpMappingType::Height)
        {
            const std::string bumpName = "Bump_" + std::to_string(1001 + atlasID) + "." + EImageFileType_enumToString(bumpMappingParams.bumpMappingFileType);
            bfs::path bumpMapPath = outPath / bumpName;
            ALICEVISION_LOG_INFO("Writing bump map: " << bumpMapPath);
            imageIO::writeImage(bumpMapPath.string(), outTextureSide, outTextureSide, heightMap, imageIO::EImageQuality::OPTIMIZED, outputColorSpace);
        }
        // Save Displacement Map
        if(bumpMappingParams.displacementFileType != imageIO::EImageFileType::NONE)
        {
            const std::string dispName = "Displacement_" + std::to_string(1001 + atlasID) + "." + EImageFileType_enumToString(bumpMappingParams.displacementFileType);
            bfs::path dispMapPath = outPath / dispName;
            ALICEVISION_LOG_INFO("Writing displacement map: " << dispMapPath);
            imageIO::writeImage(dispMapPath.string(), outTextureSide, outTextureSide, heightMap, imageIO::EImageQuality::OPTIMIZED, outputColorSpace);
        }
    }
}

} // namespace mesh
} // namespace aliceVision
