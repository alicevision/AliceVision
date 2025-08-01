// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/Mesh.hpp>

#include <boost/program_options.hpp>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <ostream>
#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

/**
 * @brief Convert Mesh
 */
int aliceVision_main(int argc, char** argv)
{
    // timer initialization
    system::Timer timer;

    // command-line required parameters 
    std::string inputMeshPath;
    std::string outputMeshPath;

    // command-line optional parameters 
    bool flipNormals = false;
    bool copyTextures = true;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputMesh", po::value<std::string>(&inputMeshPath)->required(),
         "Mesh file path (*.obj, *.fbx, *.gltf, *.glb, *.stl, *.ply).")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(),
         "Output file path (*.obj, *.fbx, *.gltf, *.glb, *.stl *.ply).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
         "Flip face normals. It can be needed as it depends on the vertices order in triangles and the "
         "convention changes from one software to another.")
        ("copyTextures", po::value<bool>(&copyTextures)->default_value(copyTextures),
         "Copy input mesh texture files to the output mesh folder.");
    // clang-format on

    CmdLine cmdline("The program allows to convert a mesh to another mesh format.\n"
                    "AliceVision convertMesh");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // check first mesh file path
    if (!inputMeshPath.empty() && !utils::exists(inputMeshPath) && !fs::is_regular_file(inputMeshPath))
    {
        ALICEVISION_LOG_ERROR("The input mesh file doesn't exist");
        return EXIT_FAILURE;
    }

    // check output file path
    if (outputMeshPath.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid output");
        return EXIT_FAILURE;
    }

    // ensure output folder exists
    {
        const std::string outputFolderPart = fs::path(outputMeshPath).parent_path().string();

        if (!outputFolderPart.empty() && !utils::exists(outputFolderPart))
        {
            if (!fs::create_directory(outputFolderPart))
            {
                ALICEVISION_LOG_ERROR("Cannot create output folder");
                return EXIT_FAILURE;
            }
        }
    }

    // load input mesh
    ALICEVISION_LOG_INFO("Loading input mesh.");
    mesh::Texturing texturing;
    texturing.loadWithMaterial(inputMeshPath, flipNormals);
    mesh::Mesh* inputMesh = texturing.mesh;

    // check if mesh is loaded
    if (!inputMesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    // check if mesh is not empty
    if (inputMesh->pts.empty() || inputMesh->tris.empty())
    {
        ALICEVISION_LOG_ERROR("Empty input mesh from the file: " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << inputMesh->pts.size() << " vertices and " << inputMesh->tris.size() << " facets.");
        return EXIT_FAILURE;
    }

    // copy textures files from the input mesh folder to the output mesh folder
    if (copyTextures)
    {
        const std::string outTypeStr = std::filesystem::path(outputMeshPath).extension().string().substr(1);
        const mesh::EFileType outType = mesh::EFileType_stringToEnum(outTypeStr);

        // only copy textures for mesh formats that support textures
        if (outType == mesh::EFileType::OBJ || 
            outType == mesh::EFileType::FBX || 
            outType == mesh::EFileType::GLTF || 
            outType == mesh::EFileType::GLB)
        {
            for (const auto& texturePath : texturing.material.getAllTextures())
            {
                ALICEVISION_LOG_DEBUG("Copying texture file: " << texturePath);

                const fs::path srcPath = fs::path(inputMeshPath).parent_path() / texturePath;
                const fs::path dstPath = fs::path(outputMeshPath).parent_path() / texturePath;

                fs::copy(srcPath, dstPath);
            }
        }
    }
    
    // save output mesh
    ALICEVISION_LOG_INFO("Saving output mesh.");
    {
        const auto outPath = std::filesystem::path(outputMeshPath);
        const mesh::EFileType outFileType = mesh::EFileType_stringToEnum(outPath.extension().string().substr(1));

        texturing.saveAs(outPath.parent_path().string(), outPath.stem().string(), outFileType);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

    return EXIT_SUCCESS;
}
