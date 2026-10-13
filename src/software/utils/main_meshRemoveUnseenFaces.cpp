// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/meshVisibility.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/sfmMvsUtils/visibility.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <geogram/basic/common.h>

#include <boost/program_options.hpp>

#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = std::filesystem;
namespace po = boost::program_options;

fs::path absolutePathNoExt(const fs::path& p) { return p.parent_path() / p.stem(); }

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string sfmDataFilename;
    std::string inputMeshFilepath;     // Model to texture (HighPoly for diffuse, LowPoly for Diffuse+Normal)
    std::string outputMeshPath;

    int minObservations = 1;
    int minVertices = 3;

    const bool flipNormals = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath)->required(),
         "Input mesh to analyse the cameras visibilities.")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(),
         "Output mesh.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("minObservations", po::value<int>(&minObservations)->default_value(minObservations),
         "Minimal number of observation to keep the vertex.")
        ("minVertices", po::value<int>(&minVertices)->default_value(minVertices),
         "Minimal number of visible vertices to remove the triangle.");
    // clang-format on

    CmdLine cmdline("AliceVision Mesh Remove Unseen Faces");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    GEO::initialize();

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataFilename.empty())
    {
        ALICEVISION_LOG_INFO("Load SfMData.");
        if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL_DENSE))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
            return EXIT_FAILURE;
        }
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, "");

    const fs::path outDirectory = fs::path(outputMeshPath).parent_path();
    if (!fs::is_directory(outDirectory))
        fs::create_directory(outDirectory);

    // load input mesh (to texture) obj file
    ALICEVISION_LOG_INFO("Load input mesh.");
    mesh::Texturing texturing;
    texturing.loadWithAtlas(inputMeshFilepath);
    if (!texturing.mesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshFilepath);
        return EXIT_FAILURE;
    }
    mesh::Mesh& mesh = *texturing.mesh;

    ALICEVISION_LOG_INFO("Remap visibilities from mesh itself.");
    mesh::remapMeshVisibilities_meshItself(mp, mesh);

    StaticVectorBool trisToStay(mesh.tris.size(), true);

    ALICEVISION_LOG_INFO("Convert vertices visibility into triangles visibility: " << mesh.tris.size() << " triangles.");
#pragma omp parallel for
    for (int i = 0; i < mesh.tris.size(); ++i)
    {
        int count = 0;
        for(int tv = 0; tv < 3; ++tv)
        {
            int vi = mesh.tris[i].v[tv];
            if(mesh.pointsVisibilities[vi].size() < minObservations)
            {
                ++count;
            }
        }
        if(count >= minVertices)
        {
            trisToStay[i] = false;
        }
    }

    ALICEVISION_LOG_INFO("Remove selected triangles from the mesh.");
    mesh.letJustTringlesIdsInMesh(trisToStay);
    // clear  free points created by triangles removal
    {
        ALICEVISION_LOG_INFO("Remove free points from the mesh.");
        StaticVector<int> ptIdToNewPtId;
        mesh.removeFreePointsFromMesh(ptIdToNewPtId);
    }

    ALICEVISION_LOG_INFO("Save the mesh: " << outputMeshPath);
    // Save output mesh
    mesh.save(outputMeshPath);

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
