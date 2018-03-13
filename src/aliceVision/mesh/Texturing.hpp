// This file is part of the AliceVision project.
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

#include <boost/filesystem.hpp>

namespace bfs = boost::filesystem;

namespace aliceVision {
namespace mesh {

struct TexturingParams
{
    unsigned int textureSide = 8192;
    unsigned int padding = 15;
    unsigned int downscale = 2;
    bool fillHoles = false;
};

struct Texturing
{
    TexturingParams texParams;

    int nmtls = 0;
    StaticVector<int>* trisMtlIds = nullptr;
    StaticVector<Point2d>* uvCoords = nullptr;
    StaticVector<Voxel>* trisUvIds = nullptr;
    StaticVector<Point3d>* normals = nullptr;
    StaticVector<Voxel>* trisNormalsIds = nullptr;
    Mesh* me = nullptr;

    /// texture atlas to 3D triangle ids
    std::vector<std::vector<int>> _atlases;

    ~Texturing()
    {
        delete trisMtlIds;
        delete uvCoords;
        delete trisUvIds;
        delete normals;
        delete trisNormalsIds;
        delete me;
    }

public:
    /// Load a mesh from a .obj file and initialize internal structures
    void loadFromOBJ(const std::string& filename, bool flipNormals=false);

    inline bool hasUVs() const { return uvCoords != nullptr && !uvCoords->empty(); }

    /**
     * @brief Generate automatic texture atlasing and UV coordinates based on points visibilities
     *
     * Requires internal mesh 'me' to be initialized.
     * Note that mesh data will be updated by this process so that only textured triangles are kept.
     *
     * @param mp
     * @param ptsCams visibilities of internal mesh's points
     * @return visibilities of new internal mesh's points
     */
    StaticVector<StaticVector<int>*>* generateUVs(mvsUtils::MultiViewParams &mp, StaticVector<StaticVector<int> *> *ptsCams);

    /// Generate texture files for all texture atlases
    void generateTextures(const mvsUtils::MultiViewParams& mp, StaticVector<StaticVector<int>*>* ptsCams,
                          const bfs::path &outPath, EImageFileType textureFileType = EImageFileType::PNG);

    /// Generate texture files for the given texture atlas index
    void generateTexture(const mvsUtils::MultiViewParams& mp, StaticVector<StaticVector<int>*>* ptsCams,
                         size_t atlasID, mvsUtils::ImagesCache& imageCache,
                         const bfs::path &outPath, EImageFileType textureFileType = EImageFileType::PNG);

    /// Save textured mesh as an OBJ + MTL file
    void saveAsOBJ(const bfs::path& dir, const std::string& basename, EImageFileType textureFileType = EImageFileType::PNG);
};

} // namespace mesh
} // namespace aliceVision
