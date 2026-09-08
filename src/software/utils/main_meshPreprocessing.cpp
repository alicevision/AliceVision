// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>


#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <meshoptimizer.h>
#include <aliceVision/mesh/Octree.hpp>

#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <stack>
#include <queue>
#include <limits>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


void buildChildAggregateMesh(const mesh::OctreeNode& parent, mesh::IndexedMeshStreams& out)
{
    out.positions.clear();
    out.normals.clear();
    out.indices.clear();

    std::size_t totalVertexCount = 0;
    std::size_t totalIndexCount = 0;

    for (const auto& child : parent.getChildren())
    {
        if (!child || !child->hasFaces())
        {
            continue;
        }

        totalVertexCount += child->getVertices().size();
        totalIndexCount += child->getIndices().size();
    }

    out.positions.reserve(totalVertexCount);
    out.normals.reserve(totalVertexCount);
    out.indices.reserve(totalIndexCount);

    unsigned int baseVertexOffset = 0;

    for (const auto& child : parent.getChildren())
    {
        if (!child || !child->hasFaces())
        {
            continue;
        }

        const std::vector<Vec3f>& childVertices = child->getVertices();
        const std::vector<Vec3f>& childNormals = child->getNormals();

        out.positions.insert(out.positions.end(), childVertices.begin(), childVertices.end());
        out.normals.insert(out.normals.end(), childNormals.begin(), childNormals.end());

        for (const unsigned & idx : child->getIndices())
        {
            out.indices.push_back(baseVertexOffset + idx);
        }

        baseVertexOffset += static_cast<unsigned int>(childVertices.size());
    }
}

mesh::IndexedMeshStreams buildIndexedMeshStreams(const mesh::OctreeNode& node)
{
    mesh::IndexedMeshStreams mesh;
    mesh.positions = node.getVertices();
    mesh.normals = node.getNormals();
    mesh.indices = node.getIndices();

    return mesh;
}

void weldIndexedMeshStreams(mesh::IndexedMeshStreams& mesh)
{
    if (mesh.positions.empty() || mesh.indices.empty())
    {
        return;
    }

    std::vector<unsigned int> remap(mesh.positions.size());

    const meshopt_Stream streams[] = {
        {mesh.positions.data(), sizeof(Vec3f), sizeof(Vec3f)},
        {mesh.normals.data(), sizeof(Vec3f), sizeof(Vec3f)},
    };

    const std::size_t uniqueVertexCount = meshopt_generateVertexRemapMulti(
        remap.data(),
        mesh.indices.data(),
        mesh.indices.size(),
        mesh.positions.size(),
        streams,
        2);

    std::vector<Vec3f> weldedPositions(uniqueVertexCount);
    std::vector<Vec3f> weldedNormals(uniqueVertexCount);
    std::vector<unsigned int> weldedIndices(mesh.indices.size());

    meshopt_remapVertexBuffer(
        weldedPositions.data(),
        mesh.positions.data(),
        mesh.positions.size(),
        sizeof(Vec3f),
        remap.data());
    meshopt_remapVertexBuffer(
        weldedNormals.data(),
        mesh.normals.data(),
        mesh.normals.size(),
        sizeof(Vec3f),
        remap.data());
    meshopt_remapIndexBuffer(
        weldedIndices.data(),
        mesh.indices.data(),
        mesh.indices.size(),
        remap.data());

    mesh.positions = std::move(weldedPositions);
    mesh.normals = std::move(weldedNormals);
    mesh.indices = std::move(weldedIndices);
}

bool simplifyIndexedMeshStreams(const mesh::IndexedMeshStreams& input,
                                mesh::IndexedMeshStreams& output,
                                std::size_t targetTriangleCount,
                                float targetError,
                                float& resultError,
                                unsigned int options = meshopt_SimplifyLockBorder)
{
    output = input;

    if (input.positions.empty() || input.indices.size() < 3)
    {
        resultError = 0.0f;
        return false;
    }

    const std::size_t inputTriangleCount = input.indices.size() / 3;

    if (targetTriangleCount >= inputTriangleCount)
    {
        resultError = 0.0f;
        return false;
    }

    const float scale = meshopt_simplifyScale(
        input.positions[0].data(),
        input.positions.size(),
        sizeof(Vec3f));

    const float relativeTargetError = (targetError < 1.0f) ? targetError / scale : 1.0f;

    const std::size_t targetIndexCount = targetTriangleCount * 3;
    std::vector<unsigned int> simplifiedIndices(input.indices.size());
    float localError = 0.0f;
    std::size_t simplifiedIndexCount = 0;

    if (input.normals.size() == input.positions.size())
    {
        //Let ignore those normals as they may be more noisy than useful.
        static constexpr float normalWeights[3] = {0.0f, 0.0f, 0.0f};

        simplifiedIndexCount = meshopt_simplifyWithAttributes(
            simplifiedIndices.data(),
            input.indices.data(),
            input.indices.size(),
            input.positions[0].data(),
            input.positions.size(),
            sizeof(Vec3f),
            input.normals[0].data(),
            sizeof(Vec3f),
            normalWeights,
            3,
            nullptr,
            targetIndexCount,
            relativeTargetError,
            options,
            &localError);
    }
    else
    {
        simplifiedIndexCount = meshopt_simplify(
            simplifiedIndices.data(),
            input.indices.data(),
            input.indices.size(),
            input.positions[0].data(),
            input.positions.size(),
            sizeof(Vec3f),
            targetIndexCount,
            relativeTargetError,
            options,
            &localError);
    }

    if (simplifiedIndexCount < 3)
    {
        resultError = localError * scale;
        return false;
    }

    simplifiedIndices.resize(simplifiedIndexCount);

    std::vector<unsigned int> remap(input.positions.size());
    const std::size_t simplifiedVertexCount = meshopt_optimizeVertexFetchRemap(
        remap.data(),
        simplifiedIndices.data(),
        simplifiedIndices.size(),
        input.positions.size());

    output.positions.resize(simplifiedVertexCount);
    output.indices.resize(simplifiedIndices.size());

    meshopt_remapVertexBuffer(
        output.positions.data(),
        input.positions.data(),
        input.positions.size(),
        sizeof(Vec3f),
        remap.data());
    meshopt_remapIndexBuffer(
        output.indices.data(),
        simplifiedIndices.data(),
        simplifiedIndices.size(),
        remap.data());

    if (input.normals.size() == input.positions.size())
    {
        output.normals.resize(simplifiedVertexCount);
        meshopt_remapVertexBuffer(
            output.normals.data(),
            input.normals.data(),
            input.normals.size(),
            sizeof(Vec3f),
            remap.data());
    }
    else
    {
        output.normals.clear();
    }

    ALICEVISION_LOG_INFO(scale);
    ALICEVISION_LOG_INFO(localError);
    resultError = localError * scale;

    return true;
}

std::vector<mesh::IndexedMeshStreams> buildLeafMeshes(const mesh::OctreeNode& octree)
{
    std::vector<mesh::IndexedMeshStreams> meshes;

    for (const mesh::OctreeNode* node : octree.getLeaves())
    {
        if (!node->hasFaces())
        {
            continue;
        }

        mesh::IndexedMeshStreams mesh = buildIndexedMeshStreams(*node);

       
        weldIndexedMeshStreams(mesh);

        meshes.push_back(std::move(mesh));
    }

    return meshes;
}


bool importScene(const std::string & path, std::vector<Vec3f> & vertices, std::vector<Vec3f> & normals, std::vector<unsigned> & indices)
{
    vertices.clear();
    normals.clear();
    indices.clear();

    Assimp::Importer importer;

    const aiScene *scene = importer.ReadFile(
        path,
        aiProcess_Triangulate           |  // triangles only
        aiProcess_GenSmoothNormals      |  // generate normals if missing
        aiProcess_JoinIdenticalVertices |  // deduplicate vertices
        aiProcess_FlipUVs               |  // Qt / Vulkan UV convention
        aiProcess_PreTransformVertices  |  // bake node transforms
        aiProcess_ValidateDataStructure
    );

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) 
    {
        ALICEVISION_LOG_ERROR("Assimp : failed to load : " << importer.GetErrorString());
        return false;
    }

    if (scene->mNumMeshes != 1) 
    {
        ALICEVISION_LOG_ERROR("Assimp : This application only supports 1 mesh per file");
        return false;
    }

    const aiMesh *mesh = scene->mMeshes[0];
    
    if (!mesh || mesh->mNumVertices == 0 || mesh->mNumFaces == 0)
    {
        ALICEVISION_LOG_ERROR("Invalid mesh.");
        return false;
    }

    vertices.resize(mesh->mNumVertices);
    normals.resize(mesh->mNumVertices);
    indices.resize(mesh->mNumFaces * 3);
    
    // Vertices
    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
        
        Vec3f & vertex = vertices[v];
        Vec3f & normal = normals[v];

        vertex[0] = mesh->mVertices[v].x;
        vertex[1] = mesh->mVertices[v].y;
        vertex[2] = mesh->mVertices[v].z;
        normal[0] = mesh->mNormals[v].x;
        normal[1] = mesh->mNormals[v].y;
        normal[2] = mesh->mNormals[v].z;
    }

    // Indices
    int pos = 0;
    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) 
    {
        const aiFace &face = mesh->mFaces[f];
        // aiProcess_Triangulate guarantees 3 indices per face

        unsigned * oindices = &indices[pos];

        oindices[0] = face.mIndices[0];
        oindices[1] = face.mIndices[1];
        oindices[2] = face.mIndices[2];

        pos += 3;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string inputFilename;
    std::string outputFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputFilename)->required(), "Input mesh.")
        ("output,o", po::value<std::string>(&outputFilename)->required(), "Output mesh.");


    CmdLine cmdline("Preprocessing mesh for visualization.\n"
                    "AliceVision meshPreprocessing");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    mesh::IndexedMeshStreams mesh;

    ALICEVISION_LOG_INFO("Importing mesh.");
    if (!importScene(inputFilename, mesh.positions, mesh.normals, mesh.indices))
    {
        ALICEVISION_LOG_ERROR("Failed loading scene.");
        return EXIT_FAILURE;
    }

    /*ALICEVISION_LOG_INFO("Computing bounds.");
    octree.computeBounds();
    
    ALICEVISION_LOG_INFO("Build root.");
    IndexedMeshStreams mesh = buildIndexedMeshStreams(octree);
    IndexedMeshStreams output;

    
    ALICEVISION_LOG_INFO("First lossless level");
    float error;
    simplifyIndexedMeshStreams(mesh, output, 0, 0.0f, error);
    
    ALICEVISION_LOG_ERROR(error);
    ALICEVISION_LOG_ERROR(mesh.indices.size() / 3);
    ALICEVISION_LOG_ERROR(output.indices.size() / 3);

    for (int i = 0; i < 20; i++)
    {
        float target = 0.00022 * pow(1.5, i);
        simplifyIndexedMeshStreams(mesh, output, 0, target, error);
    
        ALICEVISION_LOG_ERROR(error);
        ALICEVISION_LOG_ERROR(mesh.indices.size() / 3);
        ALICEVISION_LOG_ERROR(output.indices.size() / 3);
    }


    ALICEVISION_LOG_INFO("Last level");
    simplifyIndexedMeshStreams(mesh, output, 5000000, 1.0f, error);
    
    ALICEVISION_LOG_ERROR(error);
    ALICEVISION_LOG_ERROR(mesh.indices.size() / 3);
    ALICEVISION_LOG_ERROR(output.indices.size() / 3);*/

    
    /*size_t maxTriangles = 5000;

    ALICEVISION_LOG_INFO("Splitting.");
    octree.buildDown(maxTriangles, 0.01);

    ALICEVISION_LOG_INFO("Balancing.");
    octree.balance();*/

    /*IndexedMeshStats meshStats;
    std::vector<IndexedMeshStreams> leafMeshes = buildLeafMeshes(octree, &meshStats);

    ALICEVISION_LOG_INFO(
        "Built " << leafMeshes.size() << " welded leaf meshes ("
        << meshStats.vertexCountBeforeWeld << " -> " << meshStats.vertexCountAfterWeld
        << " vertices across " << meshStats.triangleCount << " triangles).");
*/

    return EXIT_SUCCESS;
}
