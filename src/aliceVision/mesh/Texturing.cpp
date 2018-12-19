// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Texturing.hpp"
#include "geoMesh.hpp"
#include "UVAtlas.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/mvsData/Pixel.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <geogram/basic/common.h>
#include <geogram/basic/geometry_nd.h>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/parameterization/mesh_atlas_maker.h>

#include <boost/algorithm/string/case_conv.hpp> 

#include <map>
#include <set>

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

EVisibilityRemappingMethod EVisibilityRemappingMethod_stringToEnum(const std::string& method)
{
    std::string m = method;
    boost::to_lower(m);
    if (m == "pull")
        return EVisibilityRemappingMethod::Pull;
    if (m == "push")
        return EVisibilityRemappingMethod::Push;
    if (m == "pullpush")
        return EVisibilityRemappingMethod::PullPush;
    throw std::out_of_range("Invalid unwrap method " + method);
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

void Texturing::generateUVs(mvsUtils::MultiViewParams& mp)
{
    if(!me)
        throw std::runtime_error("Can't generate UVs without a mesh");

    // automatic uv atlasing
    ALICEVISION_LOG_INFO("Generating UVs (textureSize: " << texParams.textureSize << "; padding: " << texParams.padding << ").");
    UVAtlas mua(*me, mp, pointsVisibilities, texParams.textureSize, texParams.padding);
    // create a new mesh to store data
    Mesh* m = new Mesh();
    m->pts = new StaticVector<Point3d>();
    m->pts->reserve(me->pts->size());
    m->tris = new StaticVector<Mesh::triangle>();
    m->tris->reserve(me->tris->size());
    trisUvIds.reserve(me->tris->size());
    uvCoords.reserve(me->pts->size());
    _atlases.clear();
    _atlases.resize(mua.atlases().size());

    std::map<int, int> vertexCache;
    PointsVisibility* updatedPointsCams = new PointsVisibility;
    updatedPointsCams->reserve(pointsVisibilities->size());

    int atlasId = 0;
    int triangleCount = 0;

    for(auto& charts : mua.atlases())
    {
        for(auto& chart : charts)
        {
            std::map<int, int> uvCache;

            Pixel offset = chart.targetLU;
            offset = offset - chart.sourceLU;

            // for each triangle in this chart
            for(size_t i = 0 ; i<chart.triangleIDs.size(); ++i)
            {
                int triangleID = chart.triangleIDs[i];
                // register triangle in corresponding atlas
                _atlases[atlasId].push_back(triangleCount);

                Mesh::triangle t;
                Voxel triUv;
                // for each point
                for(int k = 0; k < 3; ++k)
                {
                    int pointId = (*me->tris)[triangleID].v[k];
                    // get 3d triangle points
                    Point3d p = (*me->pts)[pointId];
                    Point2d uvPix;
                    if(chart.refCameraID != -1)
                    {
                        Point2d pix;
                        mp.getPixelFor3DPoint(&pix, p, chart.refCameraID);
                        if(mp.isPixelInImage(pix, chart.refCameraID))
                        {
                            // compute the final pixel coordinates
                            uvPix = (pix + Point2d(offset.x, offset.y)) / (float)mua.textureSize();
                            uvPix.y = 1.0 - uvPix.y;
                            if(uvPix.x >= mua.textureSize() || uvPix.y >= mua.textureSize())
                                uvPix = Point2d();
                        }
                    }

                    auto it = vertexCache.find(pointId);
                    int newPointIdx;
                    int uvIdx;
                    if(it == vertexCache.end())
                    {
                        m->pts->push_back(p);
                        newPointIdx = m->pts->size() - 1;
                        // map point visibilities
                        StaticVector<int>* pOther = new StaticVector<int>();
                        StaticVector<int>* pRef = (*pointsVisibilities)[pointId];
                        if(pRef)
                            *pOther = *pRef;
                        updatedPointsCams->push_back(pOther);
                        // update cache
                        vertexCache[pointId] = newPointIdx;
                    }
                    else
                    {
                        newPointIdx = it->second;
                    }
                    t.v[k] = newPointIdx;
                    // store uv coord and triangle mapping
                    auto uvcacheIt = uvCache.find(newPointIdx);
                    if(uvcacheIt == uvCache.end())
                    {
                        uvCoords.push_back(uvPix);
                        uvIdx = uvCoords.size() - 1;
                        uvCache[newPointIdx] = uvIdx;
                    }
                    else
                        uvIdx = uvcacheIt->second;
                    triUv.m[k] = uvIdx;
                }
                m->tris->push_back(t);
                trisUvIds.push_back(triUv);
                triangleCount++;
            }
        }
        atlasId++;
    }

    // replace internal mesh
    std::swap(me, m);
    delete m;
    // replace visibilities
    std::swap(pointsVisibilities, updatedPointsCams);
    deleteArrayOfArrays<int>(&updatedPointsCams);
}

void Texturing::generateTextures(const mvsUtils::MultiViewParams &mp,
                                 const boost::filesystem::path &outPath, EImageFileType textureFileType)
{
    mvsUtils::ImagesCache imageCache(&mp, 0, false);
    for(size_t atlasID = 0; atlasID < _atlases.size(); ++atlasID)
        generateTexture(mp, atlasID, imageCache, outPath, textureFileType);
}


/// accumulates colors and keeps count for providing average
struct AccuColor {
    Color colorSum;
    unsigned int count = 0;

    unsigned int add(const Color& color)
    {
        colorSum = colorSum + color;
        return ++count;
    }

    Color average() const {
        return count > 0 ? colorSum / (float)count : colorSum;
    }

    void operator+(const Color& other)
    {
        add(other);
    }

    AccuColor& operator+=(const Color& other)
    {
        add(other);
        return *this;
    }
};


void Texturing::generateTexture(const mvsUtils::MultiViewParams& mp,
                                size_t atlasID, mvsUtils::ImagesCache& imageCache, const bfs::path& outPath, EImageFileType textureFileType)
{
    if(atlasID >= _atlases.size())
        throw std::runtime_error("Invalid atlas ID " + std::to_string(atlasID));

    unsigned int textureSize = texParams.textureSize * texParams.textureSize;
    std::vector<int> colorIDs(textureSize, -1);

    std::vector<std::vector<unsigned int>> camTriangles(mp.ncams);

    ALICEVISION_LOG_INFO("Generating texture for atlas " << atlasID + 1 << "/" << _atlases.size()
              << " (" << _atlases[atlasID].size() << " triangles).");

    // iterate over atlas' triangles
    for(size_t i = 0; i < _atlases[atlasID].size(); ++i)
    {
        int triangleId = _atlases[atlasID][i];

        // Fuse visibilities of the 3 vertices
        std::vector<int> allTriCams;
        for (int k = 0; k < 3; k++)
        {
            const int pointIndex = (*me->tris)[triangleId].v[k];
            const StaticVector<int>* pointVisibilities = (*pointsVisibilities)[pointIndex];
            if (pointVisibilities != nullptr)
            {
                std::copy(pointVisibilities->begin(), pointVisibilities->end(), std::inserter(allTriCams, allTriCams.end()));
            }
        }
        if (allTriCams.empty())
        {
            // triangle without visibility
            ALICEVISION_LOG_TRACE("No visibility for triangle " << triangleId << " in texture atlas " << atlasID << ".");
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
            triangleNormal = me->computeTriangleNormal(triangleId);
            triangleCenter = me->computeTriangleCenterOfGravity(triangleId);
        }
        using ScoreCamId = std::tuple<int, double, int>;
        std::vector<ScoreCamId> scorePerCamId; // <nbVertex, score, camId>
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

            const Mesh::triangle_proj tProj = me->getTriangleProjection(triangleId, &mp, camId, w, h);
            const int nbVertex = me->getTriangleNbVertexInImage(tProj, w, h, 20);
            if(nbVertex == 0)
                // No triangle vertex in the image
                continue;

            const double area = me->computeTriangleProjectionArea(tProj);
            const double score = area * double(verticesSupport);
            scorePerCamId.emplace_back(nbVertex, score, camId);
        }
        if (scorePerCamId.empty())
        {
            // triangle without visibility
            ALICEVISION_LOG_TRACE("No visibility for triangle " << triangleId << " in texture atlas " << atlasID << " after scoring!!");
            continue;
        }

        std::sort(scorePerCamId.begin(), scorePerCamId.end(), std::greater<ScoreCamId>());
        const double minScore = texParams.bestScoreThreshold * std::get<1>(scorePerCamId.front()); // bestScoreThreshold * bestScore
        const bool bestIsPartial = (std::get<0>(scorePerCamId.front()) < 3);
        int nbCumulatedVertices = 0;
        const int maxNbVerticesForFusion = texParams.maxNbImagesForFusion * 3;
        for(int i = 0; i < scorePerCamId.size(); ++i)
        {
            if (!bestIsPartial && i > 0)
            {
                bool triVisIsPartial = (std::get<0>(scorePerCamId[i]) < 3);
                nbCumulatedVertices += std::get<0>(scorePerCamId[i]);
                if(maxNbVerticesForFusion != 0 && nbCumulatedVertices > maxNbVerticesForFusion)
                    break;
                if(std::get<1>(scorePerCamId[i]) < minScore)
                    // The best image fully see the triangle and has a much better score, so only rely on the first ones
                    break;
            }

            const int camId = std::get<2>(scorePerCamId[i]);
            camTriangles[camId].push_back(triangleId);
        }
    }

    ALICEVISION_LOG_INFO("Reading pixel color.");

    std::vector<AccuColor> perPixelColors(textureSize);

    // iterate over triangles for each camera
    int camId = 0;
    for(const std::vector<unsigned int>& triangles : camTriangles)
    {
        ALICEVISION_LOG_INFO(" - camera " << camId + 1 << "/" << mp.ncams << " (" << triangles.size() << " triangles)");

        imageCache.refreshData(camId);
        #pragma omp parallel for
        for(int ti = 0; ti < triangles.size(); ++ti)
        {
            const unsigned int triangleId = triangles[ti];
            // retrieve triangle 3D and UV coordinates
            Point2d triPixs[3];
            Point3d triPts[3];

            for(int k = 0; k < 3; k++)
            {
                const int pointIndex = (*me->tris)[triangleId].v[k];
                triPts[k] = (*me->pts)[pointIndex];                               // 3D coordinates
                const int uvPointIndex = trisUvIds[triangleId].m[k];
                triPixs[k] = uvCoords[uvPointIndex] * texParams.textureSize;   // UV coordinates
            }

            // compute triangle bounding box in pixel indexes
            // min values: floor(value)
            // max values: ceil(value)
            Pixel LU, RD;
            LU.x = static_cast<int>(std::floor(std::min(std::min(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
            LU.y = static_cast<int>(std::floor(std::min(std::min(triPixs[0].y, triPixs[1].y), triPixs[2].y)));
            RD.x = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].x, triPixs[1].x), triPixs[2].x)));
            RD.y = static_cast<int>(std::ceil(std::max(std::max(triPixs[0].y, triPixs[1].y), triPixs[2].y)));

            // sanity check: clamp values to [0; textureSize]
            int texSide = static_cast<int>(texParams.textureSize);
            LU.x = clamp(LU.x, 0, texSide);
            LU.y = clamp(LU.y, 0, texSide);
            RD.x = clamp(RD.x, 0, texSide);
            RD.y = clamp(RD.y, 0, texSide);

            // iterate over bounding box's pixels
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
                    const unsigned int y_ = (texParams.textureSize - 1) - y;
                    // 1D pixel index
                    unsigned int xyoffset = y_ * texParams.textureSize + x;
                    // get 3D coordinates
                    Point3d pt3d = barycentricToCartesian(triPts, barycCoords);
                    // get 2D coordinates in source image
                    Point2d pixRC;
                    mp.getPixelFor3DPoint(&pixRC, pt3d, camId);
                    // exclude out of bounds pixels
                    if(!mp.isPixelInImage(pixRC, camId))
                        continue;
                    Color color = imageCache.getPixelValueInterpolated(&pixRC, camId);
                    // If the color is pure zero, we consider it as an invalid pixel.
                    // After correction of radial distortion, some pixels are invalid.
                    // TODO: use an alpha channel instead.
                    if(color == Color(0.f, 0.f, 0.f))
                        continue;
                    // fill the accumulated color map for this pixel
                    perPixelColors[xyoffset] += color;
                    // fill the colorID map
                    colorIDs[xyoffset] = xyoffset;
                }
            }
        }
        // increment current cam index
        camId++;
    }
    camTriangles.clear();

    if(!texParams.fillHoles && texParams.padding > 0)
    {
        ALICEVISION_LOG_INFO("Edge padding (" << texParams.padding << " pixels).");
        // edge padding (dilate gutter)
        for(unsigned int g = 0; g < texParams.padding; ++g)
        {
            for(unsigned int y = 1; y < texParams.textureSize-1; ++y)
            {
                unsigned int yoffset = y * texParams.textureSize;
                for(unsigned int x = 1; x < texParams.textureSize-1; ++x)
                {
                    unsigned int xyoffset = yoffset + x;
                    if(colorIDs[xyoffset] > 0)
                        continue;
                    else if(colorIDs[xyoffset-1] > 0)
                    {
                        colorIDs[xyoffset] = (xyoffset-1)*-1;
                    }
                    else if(colorIDs[xyoffset+1] > 0)
                    {
                        colorIDs[xyoffset] = (xyoffset+1)*-1;
                    }
                    else if(colorIDs[xyoffset+texParams.textureSize] > 0)
                    {
                        colorIDs[xyoffset] = (xyoffset+texParams.textureSize)*-1;
                    }
                    else if(colorIDs[xyoffset-texParams.textureSize] > 0)
                    {
                        colorIDs[xyoffset] = (xyoffset-texParams.textureSize)*-1;
                    }
                }
            }
            for(unsigned int i=0; i < textureSize; ++i)
            {
                if(colorIDs[i] < 0)
                    colorIDs[i] = colorIDs[colorIDs[i]*-1];
            }
        }
    }

    ALICEVISION_LOG_INFO("Computing final (average) color.");

    // save texture image
    std::vector<Color> colorBuffer(texParams.textureSize * texParams.textureSize);
    std::vector<float> alphaBuffer;
    if(texParams.fillHoles)
        alphaBuffer.resize(colorBuffer.size(), 0.0f);

    for(unsigned int yp = 0; yp < texParams.textureSize; ++yp)
    {
        unsigned int yoffset = yp * texParams.textureSize;
        for(unsigned int xp = 0; xp < texParams.textureSize; ++xp)
        {
            unsigned int xyoffset = yoffset + xp;
            int colorID = colorIDs[xyoffset];
            Color color;
            if(colorID >= 0)
            {
                color = perPixelColors[colorID].average();
                if(texParams.fillHoles)
                    alphaBuffer[xyoffset] = 1.0f;
            }
            colorBuffer[xyoffset] = color;
        }
    }

    perPixelColors.clear();
    colorIDs.clear();

    std::string textureName = "texture_" + std::to_string(atlasID) + "." + EImageFileType_enumToString(textureFileType);
    bfs::path texturePath = outPath / textureName;
    ALICEVISION_LOG_INFO("Writing texture file: " << texturePath.string());

    unsigned int outTextureSide = texParams.textureSize;

    // texture holes filling
    if(texParams.fillHoles)
    {
        ALICEVISION_LOG_INFO("Filling texture holes.");
        imageIO::fillHoles(texParams.textureSize, texParams.textureSize, colorBuffer, alphaBuffer);
        alphaBuffer.clear();
    }
    // downscale texture if required
    if(texParams.downscale > 1)
    {
        std::vector<Color> resizedColorBuffer;
        outTextureSide = texParams.textureSize / texParams.downscale;

        ALICEVISION_LOG_INFO("Downscaling texture (" << texParams.downscale << "x).");
        imageIO::resizeImage(texParams.textureSize, texParams.textureSize, texParams.downscale, colorBuffer, resizedColorBuffer);
        std::swap(resizedColorBuffer, colorBuffer);
    }
    imageIO::writeImage(texturePath.string(), outTextureSide, outTextureSide, colorBuffer);
}


void Texturing::clear()
{
    trisMtlIds.clear();
    uvCoords.clear();
    trisUvIds.clear();
    normals.clear();
    trisNormalsIds.clear();
    _atlases.clear();

    if(pointsVisibilities != nullptr)
    {
        deleteArrayOfArrays<int>(&pointsVisibilities);
        pointsVisibilities = nullptr;
    }

    delete me;
    me = nullptr;
}

void Texturing::loadFromOBJ(const std::string& filename, bool flipNormals)
{
    // Clear internal data
    clear();
    me = new Mesh();
    // Load .obj
    if(!me->loadFromObjAscii(nmtls, trisMtlIds, normals, trisNormalsIds, uvCoords, trisUvIds,
                             filename.c_str()))
    {
        throw std::runtime_error("Unable to load: " + filename);
    }

    // Handle normals flipping
    if(flipNormals)
        me->invertTriangleOrientations();

    // Fill atlases (1 atlas per material) with corresponding rectangles
    // if no material, create only one atlas with all triangles
    _atlases.resize(std::max(1, nmtls));
    for(int triangleID = 0; triangleID < trisMtlIds.size(); triangleID++)
    {
        unsigned int atlasID = nmtls ? trisMtlIds[triangleID] : 0;
        _atlases[atlasID].push_back(triangleID);
    }
}

void Texturing::loadFromMeshing(const std::string& meshFilepath, const std::string& visibilitiesFilepath)
{
    clear();
    me = new Mesh();
    if(!me->loadFromBin(meshFilepath))
    {
        throw std::runtime_error("Unable to load: " + meshFilepath);
    }
    pointsVisibilities = loadArrayOfArraysFromFile<int>(visibilitiesFilepath);
    if(pointsVisibilities->size() != me->pts->size())
        throw std::runtime_error("Error: Reference mesh and associated visibilities don't have the same size.");
}

void Texturing::replaceMesh(const std::string& otherMeshPath, bool flipNormals)
{
    // keep previous mesh/visibilities as reference
    Mesh* refMesh = me;
    PointsVisibility* refVisibilities = pointsVisibilities;
    // set pointers to null to avoid deallocation by 'loadFromObj'
    me = nullptr;
    pointsVisibilities = nullptr;
    // load input obj file
    loadFromOBJ(otherMeshPath, flipNormals);
    // allocate pointsVisibilities for new internal mesh
    pointsVisibilities = new PointsVisibility();
    // remap visibilities from reconstruction onto input mesh
    if(texParams.visibilityRemappingMethod & EVisibilityRemappingMethod::Pull)
        remapMeshVisibilities_pullVerticesVisibility(*refMesh, *refVisibilities, *me, *pointsVisibilities);
    if (texParams.visibilityRemappingMethod & EVisibilityRemappingMethod::Push)
        remapMeshVisibilities_pushVerticesVisibilityToTriangles(*refMesh, *refVisibilities, *me, *pointsVisibilities);
    if(pointsVisibilities->empty())
        throw std::runtime_error("No visibility after visibility remapping.");

    // delete ref mesh and visibilities
    delete refMesh;
    deleteArrayOfArrays(&refVisibilities);
}

void Texturing::unwrap(mvsUtils::MultiViewParams& mp, EUnwrapMethod method)
{
    if(method == mesh::EUnwrapMethod::Basic)
    {
        // generate UV coordinates based on automatic uv atlas
        generateUVs(mp);
    }
    else
    {
        GEO::initialize();
        GEO::Mesh mesh;
        toGeoMesh(*me, mesh);

        // perform parametrization with Geogram
        const GEO::ChartParameterizer param = (method == mesh::EUnwrapMethod::ABF) ? GEO::PARAM_ABF : GEO::PARAM_SPECTRAL_LSCM;

        ALICEVISION_LOG_INFO("Start mesh atlasing (using Geogram " << EUnwrapMethod_enumToString(method) << ").");
        GEO::mesh_make_atlas(mesh, 45.0, param);
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
        GEO::mesh_save(mesh, tmpObjPath);
        // replace initial mesh
        replaceMesh(tmpObjPath);
        // remove temp mesh
        bfs::remove(tmpObjPath);
    }
}

void Texturing::saveAsOBJ(const bfs::path& dir, const std::string& basename, EImageFileType textureFileType)
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
    auto vertices = me->pts;
    for(int i = 0; i < vertices->size(); ++i)
        fprintf(fobj, "v %f %f %f\n", (*vertices)[i].x, (*vertices)[i].y, (*vertices)[i].z);

    // write UV coordinates
    for(int i=0; i < uvCoords.size(); ++i)
        fprintf(fobj, "vt %f %f\n", uvCoords[i].x, uvCoords[i].y);

    // write faces per texture atlas
    for(size_t atlasID=0; atlasID < _atlases.size(); ++atlasID)
    {
        fprintf(fobj, "usemtl TextureAtlas_%i\n", atlasID);
        for(const auto triangleID : _atlases[atlasID])
        {
            // vertex IDs
            int vertexID1 = (*me->tris)[triangleID].v[0];
            int vertexID2 = (*me->tris)[triangleID].v[1];
            int vertexID3 = (*me->tris)[triangleID].v[2];

            int uvID1 = trisUvIds[triangleID].m[0];
            int uvID2 = trisUvIds[triangleID].m[1];
            int uvID3 = trisUvIds[triangleID].m[2];

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
    for(size_t atlasID=0; atlasID < _atlases.size(); ++atlasID)
    {
        std::string textureName = "texture_" + std::to_string(atlasID) + "." + EImageFileType_enumToString(textureFileType);
        fprintf(fmtl, "\n");
        fprintf(fmtl, "newmtl TextureAtlas_%i\n", atlasID);
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
