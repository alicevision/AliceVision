// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Voxel.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/meshVisibility.hpp>
#include <aliceVision/stl/bitmask.hpp>

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

namespace aliceVision {
namespace mesh {

/**
 * @brief Available mesh unwrapping methods
 */
enum class EUnwrapMethod {
    Basic = 0, //< Basic unwrapping based on visibilities
    ABF = 1,   //< Geogram: ABF++
    LSCM = 2   //< Geogram: Spectral LSCM
};

/**
 * @brief returns the EUnwrapMethod enum from a string.
 * @param[in] method the input string.
 * @return the associated EUnwrapMethod enum.
 */
EUnwrapMethod EUnwrapMethod_stringToEnum(const std::string& method);

/**
 * @brief converts an EUnwrapMethod enum to a string.
 * @param[in] method the EUnwrapMethod enum to convert.
 * @return the string associated to the EUnwrapMethod enum.
 */
std::string EUnwrapMethod_enumToString(EUnwrapMethod method);

/**
 * @brief Method to remap visibilities from the reconstruction onto an other mesh.
 */
enum EVisibilityRemappingMethod {
    Pull = 1,    //< For each vertex of the input mesh, pull the visibilities from the closest vertex in the reconstruction.
    Push = 2,    //< For each vertex of the reconstruction, push the visibilities to the closest triangle in the input mesh.
    PullPush = Pull | Push  //< Combine results from Pull and Push results.
};

ALICEVISION_BITMASK(EVisibilityRemappingMethod);

EVisibilityRemappingMethod EVisibilityRemappingMethod_stringToEnum(const std::string& method);
std::string EVisibilityRemappingMethod_enumToString(EVisibilityRemappingMethod method);



struct TexturingParams
{
    unsigned int textureSide = 8192;
    unsigned int downscale = 1;
    bool useUDIM = true;
    bool fillHoles = false;
    unsigned int padding = 5;

    // Multi-band blending
    unsigned int nbBand = 4;
    unsigned int multiBandDownscale = 4;
    std::vector<int> multiBandNbContrib = {1, 5, 10, 0}; // number of contributions per frequency band for the multi-band blending

    bool useScore = true;
    double bestScoreThreshold = 0.1; //< 0.0 to disable filtering based on threshold to relative best score
    double angleHardThreshold = 90.0; //< 0.0 to disable angle hard threshold filtering

    imageIO::EImageColorSpace processColorspace = imageIO::EImageColorSpace::SRGB; // colorspace for the texturing internal computation
    mvsUtils::ImagesCache::ECorrectEV correctEV{mvsUtils::ImagesCache::ECorrectEV::NO_CORRECTION};

    bool forceVisibleByAllVertices = false; //< triangle visibility is based on the union of vertices visiblity
    EVisibilityRemappingMethod visibilityRemappingMethod = EVisibilityRemappingMethod::PullPush;

    float subdivisionTargetRatio = 0.8;
};

struct Texturing
{
    TexturingParams texParams;
    Mesh* mesh = nullptr;

    /// texture atlas to 3D triangle ids
    std::vector<std::vector<int>> _atlases;

    ~Texturing()
    {
        delete mesh;
    }

public:

    /// Clear internal mesh data
    void clear();

    /// Load a mesh from a .obj file and initialize internal structures
    void loadOBJWithAtlas(const std::string& filename, bool flipNormals=false);

    /**
     * @brief Remap visibilities
     *
     * @param[in] remappingMethod the remapping method
     * @param[in] refMesh the reference mesh
     * @param[in] refPointsVisibilities the reference visibilities
     */
    void remapVisibilities(EVisibilityRemappingMethod remappingMethod, const Mesh& refMesh);

    /**
     * @brief Replace inner mesh with the mesh loaded from 'otherMeshPath'
     *        and remap visibilities from the first to the second
     *
     * @param otherMeshPath the mesh to load
     * @param flipNormals whether to flip normals when loading the mesh
     */
    void replaceMesh(const std::string& otherMeshPath, bool flipNormals=false);

    /// Returns whether UV coordinates are available
    inline bool hasUVs() const { return !mesh->uvCoords.empty(); }

    /**
     * @brief Unwrap mesh with the given 'method'.
     *
     * Requires internal mesh 'me' to be initialized.
     */
    void unwrap(mvsUtils::MultiViewParams& mp, EUnwrapMethod method);

    /**
     * @brief Generate automatic texture atlasing and UV coordinates based on points visibilities with the "Basic" method.
     *
     * Requires internal mesh 'me' to be initialized.
     *
     * @param mp
     */
    void generateUVsBasicMethod(mvsUtils::MultiViewParams &mp);

    /**
     * @brief Update texture atlases, useful when the internal mesh has been sudivise
     *
     * Requires internal mesh to be initialized
     */
    void updateAtlases();

    // Create buffer for the set of output textures
    struct AccuImage
    {
        Image img;
        std::vector<float> imgCount;

        void resize(int width, int height)
        {
            img.resize(width, height);
            imgCount.resize(width * height);
        }
    };
    struct AccuPyramid
    {
        std::vector<AccuImage> pyramid;

        void init(int nbLevels, int imgWidth, int imgHeight)
        {
            pyramid.resize(nbLevels);
            for(auto& accuImage : pyramid)
                accuImage.resize(imgWidth, imgHeight);
        }
    };

    /// Generate texture files for all texture atlases
    void generateTextures(const mvsUtils::MultiViewParams& mp,
                          const bfs::path &outPath, imageIO::EImageFileType textureFileType = imageIO::EImageFileType::PNG);

    /// Generate texture files for the given sub-set of texture atlases
    void generateTexturesSubSet(const mvsUtils::MultiViewParams& mp,
                         const std::vector<size_t>& atlasIDs, mvsUtils::ImagesCache& imageCache,
                         const bfs::path &outPath, imageIO::EImageFileType textureFileType = imageIO::EImageFileType::PNG);

    ///Fill holes and write texture files for the given texture atlas
    void writeTexture(AccuImage& atlasTexture, const std::size_t atlasID, const bfs::path& outPath,
                      imageIO::EImageFileType textureFileType, const int level);

    /// Save textured mesh as an OBJ + MTL file
    void saveAsOBJ(const bfs::path& dir, const std::string& basename, imageIO::EImageFileType textureFileType = imageIO::EImageFileType::PNG);
};

} // namespace mesh
} // namespace aliceVision
