// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/image.hpp>
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

namespace GEO {
class MeshFacetsAABB;
}

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

std::string EVisibilityRemappingMethod_enumToString(EVisibilityRemappingMethod method);
EVisibilityRemappingMethod EVisibilityRemappingMethod_stringToEnum(const std::string& method);


struct TexturingParams
{
    int maxNbImagesForFusion = 3; //< max number of images to combine to create the final texture
    double bestScoreThreshold = 0.0; //< 0.0 to disable filtering based on threshold to relative best score
    double angleHardThreshold = 90.0; //< 0.0 to disable angle hard threshold filtering
    bool forceVisibleByAllVertices = false; //< triangle visibility is based on the union of vertices visiblity
    EVisibilityRemappingMethod visibilityRemappingMethod = EVisibilityRemappingMethod::PullPush;

    unsigned int textureSide = 8192;
    unsigned int padding = 15;
    unsigned int downscale = 2;
    bool fillHoles = false;
};

struct Texturing
{
    TexturingParams texParams;

    int nmtls = 0;
    StaticVector<int> trisMtlIds;
    StaticVector<Point2d> uvCoords;
    StaticVector<Voxel> trisUvIds;
    StaticVector<Point3d> normals;
    StaticVector<Voxel> trisNormalsIds;
    PointsVisibility* pointsVisibilities = nullptr;
    Mesh* me = nullptr;

    /// texture atlas to 3D triangle ids
    std::vector<std::vector<int>> _atlases;

    ~Texturing()
    {
        if(pointsVisibilities != nullptr)
            deleteArrayOfArrays<int>(&pointsVisibilities);
        delete me;
    }

public:

    /// Clear internal mesh data
    void clear();

    /// Load a mesh from a .obj file and initialize internal structures
    void loadFromOBJ(const std::string& filename, bool flipNormals=false);

    /**
     * @brief Load a mesh from a dense reconstruction.
     *
     * @param meshFilepath the path to the .bin mesh file
     * @param visibilitiesFilepath the path to the .bin points visibilities file
     */
    void loadFromMeshing(const std::string& meshFilepath, const std::string& visibilitiesFilepath);

    /**
     * @brief Replace inner mesh with the mesh loaded from 'otherMeshPath'
     *        and remap visibilities from the first to the second
     *
     * @param otherMeshPath the mesh to load
     * @param flipNormals whether to flip normals when loading the mesh
     */
    void replaceMesh(const std::string& otherMeshPath, bool flipNormals=false);

    /// Returns whether UV coordinates are available
    inline bool hasUVs() const { return !uvCoords.empty(); }

    /**
     * @brief Unwrap mesh with the given 'method'.
     *
     * Requires internal mesh 'me' to be initialized.
     */
    void unwrap(mvsUtils::MultiViewParams& mp, EUnwrapMethod method);

    /**
     * @brief Generate automatic texture atlasing and UV coordinates based on points visibilities
     *
     * Requires internal mesh 'me' to be initialized.
     *
     * @param mp
     */
    void generateUVs(mvsUtils::MultiViewParams &mp);

    /// Generate texture files for all texture atlases
    void generateTextures(const mvsUtils::MultiViewParams& mp,
                          const bfs::path &outPath, EImageFileType textureFileType = EImageFileType::PNG);

    /// Generate texture files for the given texture atlas index
    void generateTexture(const mvsUtils::MultiViewParams& mp,
                         size_t atlasID, mvsUtils::ImagesCache& imageCache,
                         const bfs::path &outPath, EImageFileType textureFileType = EImageFileType::PNG);


    void generateHeightAndNormalMaps(const mvsUtils::MultiViewParams& mp,
      const Mesh& denseMesh,
      const bfs::path &outPath, EImageFileType textureFileType = EImageFileType::PNG);

    void _generateHeightAndNormalMaps(const mvsUtils::MultiViewParams& mp,
      const GEO::MeshFacetsAABB& denseMeshAABB,
      size_t atlasID, mvsUtils::ImagesCache& imageCache,
      const bfs::path &outPath, EImageFileType textureFileType);

    /// Save textured mesh as an OBJ + MTL file
    void saveAsOBJ(const bfs::path& dir, const std::string& basename, EImageFileType textureFileType = EImageFileType::PNG);
};

} // namespace mesh
} // namespace aliceVision
