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

#include <boost/algorithm/string/case_conv.hpp> 

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
    }
    throw std::out_of_range("Unrecognized EVisibilityRemappingMethod");
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
        atlasId++;
    }
}

void Texturing::updateAtlases()
{
    ALICEVISION_LOG_INFO("updateAtlases");
    // Fill atlases (1 atlas per material) with triangles issued from mesh subdivision
    _atlases.clear();
    _atlases.resize(std::max(1, mesh->nmtls));
    for(int triangleID = 0; triangleID < mesh->trisMtlIds().size(); triangleID++)
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
    mvsUtils::ImagesCache imageCache(&mp, texParams.processColorspace, texParams.correctEV);
    imageCache.setCacheSize(2);
    ALICEVISION_LOG_INFO("Images loaded from cache with: " + imageCache.ECorrectEV_enumToString(texParams.correctEV));

    //calculate the maximum number of atlases in memory in MB
    system::MemoryInfo memInfo = system::getMemoryInfo();
    const std::size_t imageMaxMemSize =  mp.getMaxImageWidth() * mp.getMaxImageHeight() * sizeof(Color) / std::pow(2,20); //MB
    const std::size_t imagePyramidMaxMemSize = texParams.nbBand * imageMaxMemSize;
    const std::size_t atlasContribMemSize = texParams.textureSide * texParams.textureSide * (sizeof(Color)+sizeof(float)) / std::pow(2,20); //MB
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
                                const std::vector<size_t>& atlasIDs, mvsUtils::ImagesCache& imageCache, const bfs::path& outPath, imageIO::EImageFileType textureFileType)
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
            for (int k = 0; k < 3; k++)
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
        mvsUtils::ImagesCache::ImgSharedPtr imgPtr = imageCache.getImg_sync(camId);
        const Image& camImg = *imgPtr;

        // Calculate laplacianPyramid
        std::vector<Image> pyramidL; //laplacian pyramid
        camImg.laplacianPyramid(pyramidL, texParams.nbBand, texParams.multiBandDownscale);

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

                    for(int k = 0; k < 3; k++)
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
                    for(int y = LU.y; y < RD.y; y++)
                    {
                       for(int x = LU.x; x < RD.x; x++)
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
                           if(camImg.getInterpolateColor(pixRC) == Color(0.f, 0.f, 0.f))
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
        Image resizedColorBuffer;

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

void Texturing::loadOBJWithAtlas(const std::string& filename, bool flipNormals)
{
    // Clear internal data
    clear();
    mesh = new Mesh();
    // Load .obj
    if(!mesh->loadFromObjAscii(filename.c_str()))
    {
        throw std::runtime_error("Unable to load: " + filename);
    }

    // Handle normals flipping
    if(flipNormals)
        mesh->invertTriangleOrientations();

    // Fill atlases (1 atlas per material) with corresponding rectangles
    // if no material, create only one atlas with all triangles
    _atlases.resize(std::max(1, mesh->nmtls));
    for(int triangleID = 0; triangleID < mesh->trisMtlIds().size(); triangleID++)
    {
        unsigned int atlasID = mesh->nmtls ? mesh->trisMtlIds()[triangleID] : 0;
        _atlases[atlasID].push_back(triangleID);
    }
}

void Texturing::remapVisibilities(EVisibilityRemappingMethod remappingMethod, const Mesh& refMesh)
{
  if (refMesh.pointsVisibilities.empty())
    throw std::runtime_error("Texturing: Cannot remap visibilities as there is no reference points.");

  // remap visibilities from the reference onto the mesh
  if(remappingMethod == EVisibilityRemappingMethod::PullPush || remappingMethod == mesh::EVisibilityRemappingMethod::Pull)
    remapMeshVisibilities_pullVerticesVisibility(refMesh, *mesh);
  if(remappingMethod == EVisibilityRemappingMethod::PullPush || remappingMethod == mesh::EVisibilityRemappingMethod::Push)
    remapMeshVisibilities_pushVerticesVisibilityToTriangles(refMesh, *mesh);
  if(mesh->pointsVisibilities.empty())
    throw std::runtime_error("No visibility after visibility remapping.");
}

void Texturing::replaceMesh(const std::string& otherMeshPath, bool flipNormals)
{
    // keep previous mesh/visibilities as reference
    Mesh* refMesh = mesh;
    // set pointers to null to avoid deallocation by 'loadFromObj'
    mesh->pointsVisibilities.resize(0);
    mesh = nullptr;
    
    // load input obj file
    loadOBJWithAtlas(otherMeshPath, flipNormals);
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

void Texturing::saveAsOBJ(const bfs::path& dir, const std::string& basename, imageIO::EImageFileType textureFileType)
{
    ALICEVISION_LOG_INFO("Writing obj and mtl file.");

    std::string objFilename = (dir / (basename + ".obj")).string();
    std::string mtlName = (basename + ".mtl");
    std::string mtlFilename = (dir / mtlName).string();

    // create .OBJ file
    FILE* fobj = fopen(objFilename.c_str(), "w");

    // header
    fprintf(fobj, "# \n");
    fprintf(fobj, "# Wavefront OBJ file\n");
    fprintf(fobj, "# Created with AliceVision\n");
    fprintf(fobj, "# \n");
    fprintf(fobj, "mtllib %s\n\n", mtlName.c_str());
    fprintf(fobj, "g TexturedMesh\n");

    // write vertices
    auto vertices = mesh->pts;
    for(int i = 0; i < vertices.size(); ++i)
        fprintf(fobj, "v %f %f %f\n", vertices[i].x, vertices[i].y, vertices[i].z);

    // write UV coordinates
    for(int i=0; i < mesh->uvCoords.size(); ++i)
        fprintf(fobj, "vt %f %f\n", mesh->uvCoords[i].x, mesh->uvCoords[i].y);

    // write faces per texture atlas
    for(std::size_t atlasId=0; atlasId < _atlases.size(); ++atlasId)
    {
        const std::size_t textureId = 1001 + atlasId; // starts at '1001' for UDIM compatibility
        fprintf(fobj, "usemtl TextureAtlas_%i\n", textureId);
        for(const auto triangleID : _atlases[atlasId])
        {
            // vertex IDs
            int vertexID1 = mesh->tris[triangleID].v[0];
            int vertexID2 = mesh->tris[triangleID].v[1];
            int vertexID3 = mesh->tris[triangleID].v[2];

            int uvID1 = mesh->trisUvIds[triangleID].m[0];
            int uvID2 = mesh->trisUvIds[triangleID].m[1];
            int uvID3 = mesh->trisUvIds[triangleID].m[2];

            fprintf(fobj, "f %i/%i %i/%i %i/%i\n", vertexID1 + 1, uvID1 + 1, vertexID2 + 1, uvID2 + 1, vertexID3 + 1, uvID3 + 1); // indexed from 1
        }
    }
    fclose(fobj);

    // create .MTL material file
    FILE* fmtl = fopen(mtlFilename.c_str(), "w");

    // header
    fprintf(fmtl, "# \n");
    fprintf(fmtl, "# Wavefront material file\n");
    fprintf(fmtl, "# Created with AliceVision\n");
    fprintf(fmtl, "# \n\n");

    // for each atlas, create a new material with associated texture
    for(size_t atlasId=0; atlasId < _atlases.size(); ++atlasId)
    {
        const std::size_t textureId = 1001 + atlasId; // starts at '1001' for UDIM compatibility
        const std::string textureName = "texture_" + std::to_string(textureId) + "." + imageIO::EImageFileType_enumToString(textureFileType);

        fprintf(fmtl, "newmtl TextureAtlas_%i\n", textureId);
        fprintf(fmtl, "Ka  0.6 0.6 0.6\n");
        fprintf(fmtl, "Kd  0.6 0.6 0.6\n");
        fprintf(fmtl, "Ks  0.0 0.0 0.0\n");
        fprintf(fmtl, "d  1.0\n");
        fprintf(fmtl, "Ns  0.0\n");
        fprintf(fmtl, "illum 2\n");
        fprintf(fmtl, "map_Kd %s\n", textureName.c_str());
    }
    fclose(fmtl);

    ALICEVISION_LOG_INFO("Writing done: " << std::endl
                         << "\t- obj file: " << objFilename << std::endl
                         << "\t- mtl file: " << mtlFilename);
}

} // namespace mesh
} // namespace aliceVision
