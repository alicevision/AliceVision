// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputMeshPath;
    std::string outputMeshPath;

    bool keepLargestMeshOnly = false;
    double removeLargeTrianglesFactor = 60.0;

    int smoothNIter = 10;
    float lambda = 1.0f;

    po::options_description allParams("AliceVision meshFiltering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh (OBJ file format).")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("keepLargestMeshOnly", po::value<bool>(&keepLargestMeshOnly)->default_value(keepLargestMeshOnly),
            "Keep only the largest connected triangles group.")
        ("removeLargeTrianglesFactor", po::value<double>(&removeLargeTrianglesFactor)->default_value(removeLargeTrianglesFactor),
            "Remove all large triangles. We consider a triangle as large if one edge is bigger than N times the average edge length. Put zero to disable it.")
        ("iterations", po::value<int>(&smoothNIter)->default_value(smoothNIter),
            "Number of smoothing iterations.")
        ("lambda", po::value<float>(&lambda)->default_value(lambda),
            "Smoothing size.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    bfs::path outDirectory = bfs::path(outputMeshPath).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    mesh::Texturing texturing;
    texturing.loadFromOBJ(inputMeshPath);
    mesh::Mesh* mesh = texturing.me;

    if(!mesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    if(mesh->pts->empty() || mesh->tris->empty())
    {
        ALICEVISION_LOG_ERROR("Error: empty mesh from the file " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << mesh->pts->size() << " vertices and " << mesh->tris->size() << " facets.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");
    ALICEVISION_LOG_INFO("Input mesh: " << mesh->pts->size() << " vertices and " << mesh->tris->size() << " facets.");

    if(removeLargeTrianglesFactor != 0.0)
    {
        mesh->filterLargeEdgeTriangles(removeLargeTrianglesFactor);
        ALICEVISION_LOG_INFO("Mesh after large triangles removal: " << mesh->pts->size() << " vertices and " << mesh->tris->size() << " facets.");
    }

    mesh::MeshEnergyOpt meOpt(nullptr);
    {
        ALICEVISION_LOG_INFO("Start mesh filtering.");
        meOpt.addMesh(mesh);
        meOpt.init();
        meOpt.cleanMesh(10);

        StaticVectorBool* ptsCanMove = nullptr;
        meOpt.optimizeSmooth(lambda, smoothNIter, ptsCanMove);

        ALICEVISION_LOG_INFO("Mesh filtering done: " << meOpt.pts->size() << " vertices and " << meOpt.tris->size() << " facets.");
    }

    if(keepLargestMeshOnly)
    {
        StaticVector<int>* trisIdsToStay = meOpt.getLargestConnectedComponentTrisIds();
        meOpt.letJustTringlesIdsInMesh(trisIdsToStay);
        delete trisIdsToStay;
        ALICEVISION_LOG_INFO("Mesh after keepLargestMeshOnly: " << meOpt.pts->size() << " vertices and " << meOpt.tris->size() << " facets.");
    }

    mesh::Mesh outMesh;
    outMesh.addMesh(&meOpt);

    ALICEVISION_COUT("Output mesh: " << mesh->pts->size() << " vertices and " << mesh->tris->size() << " facets.");

    if(outMesh.pts->empty() || outMesh.tris->empty())
    {
        ALICEVISION_CERR("Failed: the output mesh is empty.");
        ALICEVISION_LOG_INFO("Output mesh: " << outMesh.pts->size() << " vertices and " << outMesh.tris->size() << " facets.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Save mesh.");

    // Save output mesh
    outMesh.saveToObj(outputMeshPath);

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
