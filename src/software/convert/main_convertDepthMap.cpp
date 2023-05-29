// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/image/io.hpp>

#include <assimp/Exporter.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <ostream>
#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

void createAssimpSceneFromDepthMap(int rc,
                                   const mvsUtils::MultiViewParams& mp,
                                   const image::Image<float>& depthMap,
                                   aiMesh* out_aimeshPtr)
{
    std::vector<int> indexPerPixel(static_cast<std::size_t>(depthMap.Width() * depthMap.Height()), -1);
    std::vector<Vec3f> verticesPositions;
    verticesPositions.reserve(indexPerPixel.size());

    // get vertices list from pixels and depths

    for(int y = 0; y < depthMap.Height(); ++y)
    {
        for(int x = 0; x < depthMap.Width(); ++x)
        {
            const float depthValue = depthMap(y, x);

            // check pixel depth is valid
            if(!std::isfinite(depthValue) || depthValue <= 0.f)
                continue;

            // pixel 3d projection
            const Point3d p = mp.CArr[rc] + (mp.iCamArr[rc] * Point2d(static_cast<double>(x), static_cast<double>(y))).normalize() * depthValue;

            indexPerPixel[static_cast<std::size_t>(y * depthMap.Width() + x)] = static_cast<int>(verticesPositions.size());
            verticesPositions.push_back({static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
        }
    }

    // create faces vertices indexes list

    std::vector<std::size_t> facesIndexes;
    facesIndexes.reserve(2 * 3 * verticesPositions.size());

    for(int y = 0; y < depthMap.Height() - 1; ++y)
    {
        for(int x = 0; x < depthMap.Width() - 1; ++x)
        {
            const int pixelIndexA = indexPerPixel[static_cast<std::size_t>(y * depthMap.Width() + x)];
            const int pixelIndexB = indexPerPixel[static_cast<std::size_t>((y + 1) * depthMap.Width() + x)];
            const int pixelIndexC = indexPerPixel[static_cast<std::size_t>((y + 1) * depthMap.Width() + x + 1)];
            const int pixelIndexD = indexPerPixel[static_cast<std::size_t>(y * depthMap.Width() + x + 1)];

            // cast indices to std::size_t once for readability
            const std::size_t sPixelIndexA = static_cast<std::size_t>(pixelIndexA);
            const std::size_t sPixelIndexB = static_cast<std::size_t>(pixelIndexB);
            const std::size_t sPixelIndexC = static_cast<std::size_t>(pixelIndexC);
            const std::size_t sPixelIndexD = static_cast<std::size_t>(pixelIndexD);

            if(pixelIndexA != -1 &&
               pixelIndexB != -1 &&
               pixelIndexC != -1)
            {
                facesIndexes.push_back(sPixelIndexA);
                facesIndexes.push_back(sPixelIndexB);
                facesIndexes.push_back(sPixelIndexC);
            }

            if(pixelIndexC != -1 &&
               pixelIndexD != -1 &&
               pixelIndexA != -1)
            {
                facesIndexes.push_back(sPixelIndexC);
                facesIndexes.push_back(sPixelIndexD);
                facesIndexes.push_back(sPixelIndexA);
            }
        }
    }

    // build Assimp mesh vertices

    out_aimeshPtr->mMaterialIndex = 0;
    out_aimeshPtr->mNumVertices = verticesPositions.size();
    out_aimeshPtr->mVertices = new aiVector3D[out_aimeshPtr->mNumVertices];

    for(std::size_t i = 0; i < out_aimeshPtr->mNumVertices; ++i)
    {
        const auto& vertex = verticesPositions[i];
        auto& mVertex = out_aimeshPtr->mVertices[i];

        mVertex.x =  vertex[0];
        mVertex.y = -vertex[1]; // openGL display ?
        mVertex.z = -vertex[2]; // openGL display ?
    }

    // build Assimp mesh faces

    out_aimeshPtr->mNumFaces = facesIndexes.size() / 3;
    out_aimeshPtr->mFaces = new aiFace[out_aimeshPtr->mNumFaces];

    for(std::size_t i = 0; i < out_aimeshPtr->mNumFaces; ++i)
    {
        const std::size_t firstVertexIdx = (i * 3);
        auto& mFace = out_aimeshPtr->mFaces[i];

        mFace.mNumIndices = 3;
        mFace.mIndices = new unsigned int[3];
        mFace.mIndices[0] = static_cast<unsigned int>(facesIndexes.at(firstVertexIdx + 0));
        mFace.mIndices[1] = static_cast<unsigned int>(facesIndexes.at(firstVertexIdx + 1));
        mFace.mIndices[2] = static_cast<unsigned int>(facesIndexes.at(firstVertexIdx + 2));
    }

    ALICEVISION_LOG_DEBUG("Mesh information (rc: " << rc << "): " << std::endl
                          << "\t- # valid pixels: " << verticesPositions.size() << std::endl
                          << "\t- # vertices: " << out_aimeshPtr->mNumVertices << std::endl
                          << "\t- # faces: " << out_aimeshPtr->mNumFaces);
}

/**
 * @brief Convert DepthMap to Mesh
 */
int aliceVision_main(int argc, char** argv)
{
    ALICEVISION_COMMANDLINE_START

    // command-line parameters
    std::string sfmDataFilename;
    std::string depthMapsFolder;
    std::string outputFolder;
    // program range
    int rangeStart = -1;
    int rangeSize = -1;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
            "SfMData file.")
        ("depthMapsFolder", po::value<std::string>(&depthMapsFolder)->required(),
            "Input depth map folder.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Output folder for depth maps meshes.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
            "Compute a sub-range of N images (N=rangeSize).");

    CmdLine cmdline("The program allows to convert depth maps to mesh format.\n"
                    "AliceVision convertDepthMap");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    // check command-line execution
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
      return EXIT_FAILURE;
    }

    // initialize multi-view parameters
    mvsUtils::MultiViewParams mp(sfmData, "", depthMapsFolder, "", true);

    // build camera indexes list
    std::vector<int> cams;
    cams.reserve(mp.ncams);

    // TODO: chunks / multithreading
    for(int rc = 0; rc < std::min(rangeStart + rangeSize, mp.ncams); rc++)
        cams.push_back(rc);

    // we do not need mtl file
    const std::string formatId = "objnomtl";
    const unsigned int pPreprocessing = 0u;

    // create Assimp exporter
    Assimp::Exporter exporter;

    // process all cameras depth maps
    for(const int rc : cams)
    {
        ALICEVISION_LOG_INFO("Convert depth map to obj (rc: " << rc << ")");

        // get camera view id
        const int viewId = mp.getViewId(rc);

        // get depth map filepath
        const std::string depthMapFilepath = getFileNameFromIndex(mp, rc, mvsUtils::EFileType::depthMap, 1); // scale 1 -> depth map estimation folder

        // get output depth map obj path
        const std::string depthMapObjFilepath = (fs::path(outputFolder) / fs::path(std::to_string(viewId) + "_depthMap.obj")).string();

        // create mesh from depth map
        if(fs::exists(depthMapFilepath))
        {
            // read depth map
            image::Image<float> depthMap;
            image::readImage(depthMapFilepath, depthMap, image::EImageColorSpace::NO_CONVERSION);

            // create Assimp scene
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

            ALICEVISION_LOG_DEBUG("Build Assimp mesh (rc: " << rc << ")");

            // build Assimp mesh structure from depth map image
            createAssimpSceneFromDepthMap(rc, mp, depthMap, scene.mMeshes[0]);

            ALICEVISION_LOG_DEBUG("Export Assimp mesh (rc: " << rc << ")");

            // write output mesh
            exporter.Export(&scene, formatId, depthMapObjFilepath, pPreprocessing);
        }

        ALICEVISION_LOG_INFO("Convert depth map to obj (rc: " << rc << ") done.");
    }

    ALICEVISION_COMMANDLINE_END
}
