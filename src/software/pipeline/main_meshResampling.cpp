// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_preprocessing.h>
#include <geogram/mesh/mesh_decimate.h>
#include <geogram/mesh/mesh_remesh.h>

#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>

#include <boost/program_options.hpp>

#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = std::filesystem;
namespace po = boost::program_options;

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;
    std::string inputMeshPath;
    std::string outputMeshPath;

    float simplificationFactor = 0;
    int fixedNbVertices = 0;
    int minVertices = 0;
    int maxVertices = 0;
    unsigned int nbLloydIter = 40;
    bool flipNormals = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputMeshPath)->required(),
         "Input Mesh (OBJ file format).")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(),
         "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("simplificationFactor", po::value<float>(&simplificationFactor)->default_value(simplificationFactor),
         "Simplification factor.")
        ("nbVertices", po::value<int>(&fixedNbVertices)->default_value(fixedNbVertices),
         "Fixed number of output vertices.")
        ("minVertices", po::value<int>(&minVertices)->default_value(minVertices),
         "Minimum number of output vertices.")
        ("maxVertices", po::value<int>(&maxVertices)->default_value(maxVertices),
         "Maximum number of output vertices.")
        ("nbLloydIter", po::value<unsigned int>(&nbLloydIter)->default_value(nbLloydIter),
         "Number of iterations for Lloyd pre-smoothing.")
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
         "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the "
         "convention changes from one software to another.");
    // clang-format on

    CmdLine cmdline("AliceVision meshResampling");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    fs::path outDirectory = fs::path(outputMeshPath).parent_path();
    if (!fs::is_directory(outDirectory))
        fs::create_directory(outDirectory);

    GEO::initialize();

    ALICEVISION_LOG_INFO("Geogram initialized.");

    GEO::Mesh M_in, M_out;
    {
        if (!GEO::mesh_load(inputMeshPath, M_in))
        {
            ALICEVISION_LOG_ERROR("Failed to load mesh file: \"" << inputMeshPath << "\".");
            return 1;
        }
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");

    int nbInputPoints = M_in.vertices.nb();
    int nbOutputPoints = 0;
    if (fixedNbVertices != 0)
    {
        nbOutputPoints = fixedNbVertices;
    }
    else
    {
        if (simplificationFactor != 0.0)
        {
            nbOutputPoints = simplificationFactor * nbInputPoints;
        }
        if (minVertices != 0)
        {
            if (nbInputPoints > minVertices && nbOutputPoints < minVertices)
                nbOutputPoints = minVertices;
        }
        if (maxVertices != 0)
        {
            if (nbInputPoints > maxVertices && nbOutputPoints > maxVertices)
                nbOutputPoints = maxVertices;
        }
    }

    ALICEVISION_LOG_INFO("Input mesh: " << nbInputPoints << " vertices and " << M_in.facets.nb() << " facets.");
    ALICEVISION_LOG_INFO("Target output mesh: " << nbOutputPoints << " vertices.");

    {
        GEO::CmdLine::import_arg_group("standard");
        GEO::CmdLine::import_arg_group("remesh");  // needed for remesh_smooth
        GEO::CmdLine::import_arg_group("algo");
        GEO::CmdLine::import_arg_group("post");
        GEO::CmdLine::import_arg_group("opt");
        GEO::CmdLine::import_arg_group("poly");

        const unsigned int nbNewtonIter = 0;
        const unsigned int newtonM = 0;

        ALICEVISION_LOG_INFO("Start mesh resampling.");
        GEO::remesh_smooth(M_in,
                           M_out,
                           nbOutputPoints,
                           3,             // 3 dimensions
                           nbLloydIter,   // Number of iterations for Lloyd pre-smoothing
                           nbNewtonIter,  // Number of iterations for Newton-CVT
                           newtonM        // Number of evaluations for Hessian approximation
        );
        ALICEVISION_LOG_INFO("Mesh resampling done.");
    }
    ALICEVISION_LOG_INFO("Output mesh: " << M_out.vertices.nb() << " vertices and " << M_out.facets.nb() << " facets.");

    if (M_out.facets.nb() == 0)
    {
        ALICEVISION_LOG_ERROR("The output mesh is empty.");
        return 1;
    }
    if (flipNormals)
    {
        for (GEO::index_t i = 0; i < M_out.facets.nb(); ++i)
        {
            M_out.facets.flip(i);
        }
    }

    ALICEVISION_LOG_INFO("Save mesh.");
    if (!GEO::mesh_save(M_out, outputMeshPath))
    {
        ALICEVISION_LOG_ERROR("Failed to save mesh file: \"" << outputMeshPath << "\".");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
