// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/Mesh.hpp>

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

/**
 * @brief Convert Mesh
 */
int aliceVision_main(int argc, char** argv)
{
    // timer initialization

    system::Timer timer;

    // command-line parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputMeshPath;
    std::string outputFilePath;

    po::options_description allParams("AliceVision convertMesh\n"
                                      "The program allows to convert a mesh to another mesh format.");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
      ("inputMesh", po::value<std::string>(&inputMeshPath)->default_value(inputMeshPath),
        "Mesh file path (*.obj, *.mesh, *.meshb, *.ply, *.off, *.stl).")
      ("output,o", po::value<std::string>(&outputFilePath)->default_value(outputFilePath),
        "Output file path for the new mesh file (*.obj, *.mesh, *.meshb, *.ply, *.off, *.stl)");

    po::options_description logParams("Log parameters");
    logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check first mesh file path
    if(!inputMeshPath.empty() && !fs::exists(inputMeshPath) && !fs::is_regular_file(inputMeshPath))
    {
        ALICEVISION_LOG_ERROR("The input mesh file doesn't exist");
        return EXIT_FAILURE;
    }

    // check output file path
    if(outputFilePath.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid output");
        return EXIT_FAILURE;
    }

    // ensure output folder exists
    {
        const std::string outputFolderPart = fs::path(outputFilePath).parent_path().string();

        if(!outputFolderPart.empty() && !fs::exists(outputFolderPart))
        {
            if(!fs::create_directory(outputFolderPart))
            {
                ALICEVISION_LOG_ERROR("Cannot create output folder");
                return EXIT_FAILURE;
            }
        }
    }

    // load input mesh
    mesh::Texturing texturing;
    texturing.loadWithAtlas(inputMeshPath);
    mesh::Mesh* inputMesh = texturing.mesh;

    if(!inputMesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    if(inputMesh->pts.empty() || inputMesh->tris.empty())
    {
        ALICEVISION_LOG_ERROR("Error: empty mesh from the file " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << inputMesh->pts.size() << " vertices and " << inputMesh->tris.size()
                                             << " facets.");
        return EXIT_FAILURE;
    }

    // save output mesh
    ALICEVISION_LOG_INFO("Convert mesh.");
    inputMesh->save(outputFilePath);

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

    return EXIT_SUCCESS;
}
