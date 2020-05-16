// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <EigenTypes.h>
#include <MeshTypes.h>
#include <SDFilter.h>
#include <MeshNormalFilter.h>
#include <MeshNormalDenoising.h>

#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include <OpenMesh/Core/IO/IOManager.hh>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputMeshPath;
    std::string outputMeshPath;

    int denoisingIterations = 5; // OuterIterations
    float meshUpdateClosenessWeight = 0.001;
    int meshUpdateIterations = 20;
    int meshUpdateMethod = SDFilter::MeshFilterParameters::ITERATIVE_UPDATE;
    float lambda = 2.0;
    float eta = 1.5;
    float mu = 1.5;
    float nu = 0.3;

    po::options_description allParams("AliceVision meshDenoising");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh (OBJ file format).")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("denoisingIterations", po::value<int>(&denoisingIterations)->default_value(denoisingIterations),
            "Number of denoising iterations.")
        ("meshUpdateClosenessWeight", po::value<float>(&meshUpdateClosenessWeight)->default_value(meshUpdateClosenessWeight),
            "Closeness weight for mesh update, must be positive.")
        ("lambda", po::value<float>(&lambda)->default_value(lambda),
            "Regularization weight.")
        ("eta", po::value<float>(&eta)->default_value(eta),
            "Gaussian standard deviation for spatial weight, scaled by the average distance between adjacent face centroids. Must be positive.")
        ("mu", po::value<float>(&mu)->default_value(mu),
            "Gaussian standard deviation for guidance weight.")
        ("nu", po::value<float>(&nu)->default_value(nu),
            "Gaussian standard deviation for signal weight.")
        ("meshUpdateMethod", po::value<int>(&meshUpdateMethod)->default_value(meshUpdateMethod),
            "Mesh Update Method: \n"
            "* ITERATIVE_UPDATE(" BOOST_PP_STRINGIZE(SDFilter::MeshFilterParameters::ITERATIVE_UPDATE) ") (default): ShapeUp styled iterative solver\n"
            "* POISSON_UPDATE(" BOOST_PP_STRINGIZE(SDFilter::MeshFilterParameters::POISSON_UPDATE) "): Poisson-based update from [Wang et al. 2015] \"Rolling guidance normal filter for geometric processing\"\n");

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


    TriMesh inMesh;
    if(!OpenMesh::IO::read_mesh(inMesh, inputMeshPath.c_str()))
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }
    if(inMesh.n_vertices() == 0 || inMesh.n_faces() == 0)
    {
        ALICEVISION_LOG_ERROR("Empty mesh from the file: " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << inMesh.n_vertices() << " vertices and " << inMesh.n_faces() << " facets.");
        return EXIT_FAILURE;
    }

//    #ifdef USE_OPENMP
    Eigen::initParallel();
//    #endif

    // Load option file
    SDFilter::MeshDenoisingParameters param;
    param.outer_iterations = denoisingIterations;
    param.mesh_update_method = (SDFilter::MeshFilterParameters::MeshUpdateMethod)meshUpdateMethod;
    param.mesh_update_closeness_weight = meshUpdateClosenessWeight;
    param.mesh_update_iter = meshUpdateIterations;
    param.lambda = lambda;
    param.eta = eta;
    param.mu = mu;
    param.nu = nu;

//    enum LinearSolverType
//    {
//            CG,
//            LDLT
//    };
//    // Parameters related to termination criteria
//    int max_iter;                   // Max number of iterations
//    double avg_disp_eps;    // Max average per-signal displacement threshold between two iterations for determining convergence
//    bool normalize_iterates;        // Normalization of the filtered normals in each iteration

    if(!param.valid_parameters())
    {
        ALICEVISION_LOG_ERROR("Invalid filter options. Aborting...");
        return EXIT_FAILURE;
    }
    param.output();

    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");
    ALICEVISION_LOG_INFO("Input mesh: " << inMesh.n_vertices() << " vertices and " << inMesh.n_faces() << " facets.");

    TriMesh outMesh;
    ALICEVISION_LOG_INFO("Mesh normalization.");
    // Normalize the input mesh
    Eigen::Vector3d original_center;
    double original_scale;
    SDFilter::normalize_mesh(inMesh, original_center, original_scale);
    if(true)
    {
        ALICEVISION_LOG_INFO("Start mesh denoising.");
        // Filter the normals and construct the output mesh
        SDFilter::MeshNormalDenoising denoiser(inMesh);
        denoiser.denoise(param, outMesh);
        ALICEVISION_LOG_INFO("Mesh denoising done.");
    }
    else
    {
        ALICEVISION_LOG_INFO("Start mesh filtering.");
        // Filter the normals and construct the output mesh
        SDFilter::MeshNormalFilter filter(inMesh);
        filter.filter(param, outMesh);
        ALICEVISION_LOG_INFO("Mesh filtering done.");
    }
    SDFilter::restore_mesh(outMesh, original_center, original_scale);

    ALICEVISION_LOG_INFO("Output mesh: " << outMesh.n_vertices() << " vertices and " << outMesh.n_faces() << " facets.");

    if(outMesh.n_faces() == 0)
    {
        ALICEVISION_LOG_ERROR("Failed: the output mesh is empty.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Save mesh.");
    // Save output mesh
    if(!OpenMesh::IO::write_mesh(outMesh, outputMeshPath))
    {
        ALICEVISION_LOG_ERROR("Failed to save mesh file: \"" << outputMeshPath << "\".");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
