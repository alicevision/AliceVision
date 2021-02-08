// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_repair.h>
#include <geogram/mesh/mesh_fill_holes.h>
#include <geogram/mesh/mesh_intersection.h>
#include <geogram/mesh/mesh_geometry.h>

#include <geogram/basic/command_line.h>
#include <geogram/basic/command_line_args.h>

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

enum class EOperationType : unsigned char
{
    BOOLEAN_UNION = 0,
    BOOLEAN_INTERSECTION = 1,
    BOOLEAN_DIFFERENCE = 2
};

/**
 * @brief get informations about each operation type
 * @return String
 */
std::string EOperationType_informations()
{
    return "Operation types used to merge two meshes:\n"
           "* boolean_union: Create a new mesh with the combined volume of the two input meshes.\n"
           "* boolean_intersection: Create a new mesh from the intersected volumes of the two input meshes.\n"
           "* boolean_difference: Create a new mesh from the volume of the first input mesh substracted by the second input mesh.\n";
}

/**
 * @brief convert an enum EOperationType to its corresponding string
 * @param EOperationType
 * @return String
 */
std::string EOperationType_enumToString(EOperationType operationType)
{
  switch(operationType)
  {
      case EOperationType::BOOLEAN_UNION:         return "boolean_union";
      case EOperationType::BOOLEAN_INTERSECTION:  return "boolean_intersection";
      case EOperationType::BOOLEAN_DIFFERENCE:    return "boolean_difference";
  }
  throw std::out_of_range("Invalid operationType enum: " + std::to_string(int(operationType)));
}

/**
 * @brief convert a string operationType to its corresponding enum EOperationType
 * @param String
 * @return EOperationType
 */
EOperationType EOperationType_stringToEnum(const std::string& operationType)
{
  std::string type = operationType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); // tolower

  if(type == "boolean_union")         return EOperationType::BOOLEAN_UNION;
  if(type == "boolean_intersection")  return EOperationType::BOOLEAN_INTERSECTION;
  if(type == "boolean_difference")    return EOperationType::BOOLEAN_DIFFERENCE;
  throw std::out_of_range("Invalid operationType: " + operationType);
}

std::ostream& operator<<(std::ostream& os, const EOperationType operationType)
{
    os << EOperationType_enumToString(operationType);
    return os;
}

std::istream& operator>>(std::istream& in, EOperationType& operationType)
{
    std::string token;
    in >> token;
    operationType = EOperationType_stringToEnum(token);
    return in;
}

/**
 * @brief Pre/Post-processes a mesh for boolean operations.
 * @details Triangulates the facets, collapses the small edges
 *  and removes self-intersections.
 *
 * @ from Geogram library boolean operation examples
 */
void fixMeshForBooleanOperations(GEO::Mesh& m)
{
    GEO::mesh_repair(m, GEO::MeshRepairMode(GEO::MESH_REPAIR_COLOCATE | GEO::MESH_REPAIR_DUP_F), 1e-3 * GEO::surface_average_edge_length(m));
    GEO::tessellate_facets(m, 3);
    GEO::mesh_remove_intersections(m);
}

/**
 * @brief Merge two meshes
 */
int aliceVision_main(int argc, char** argv)
{
    // timer initialization

    system::Timer timer;

    // command-line parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputFirstMeshPath;
    std::string inputSecondMeshPath;
    std::string outputFilePath;
    std::string operationTypeName = EOperationType_enumToString(EOperationType::BOOLEAN_UNION);

    bool preProcess = true;
    bool postProcess = true;

    po::options_description allParams("AliceVision mergeMeshes\n"
                                      "The program takes two meshes and applies a boolean operation on them.");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
      ("inputFirstMesh", po::value<std::string>(&inputFirstMeshPath)->default_value(inputFirstMeshPath),
        "First mesh file path (*.obj, *.mesh, *.meshb, *.ply, *.off, *.stl).")
      ("inputSecondMesh", po::value<std::string>(&inputSecondMeshPath)->default_value(inputSecondMeshPath),
        "Second mesh file path (*.obj, *.mesh, *.meshb, *.ply, *.off, *.stl).")
      ("output,o", po::value<std::string>(&outputFilePath)->default_value(outputFilePath),
        "Output file path for the new mesh file (*.obj, *.mesh, *.meshb, *.ply, *.off, *.stl)");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
      ("mergeOperation",po::value<std::string>(&operationTypeName)->default_value(operationTypeName),
        EOperationType_informations().c_str())
      ("preProcess", po::value<bool>(&preProcess)->default_value(preProcess),
        "Pre-process input meshes in order to avoid geometric errors in the merging process")
      ("postProcess", po::value<bool>(&postProcess)->default_value(postProcess),
        "Post-process output mesh in order to avoid future geometric errors");

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
    if(!inputFirstMeshPath.empty() && !fs::exists(inputFirstMeshPath) && !fs::is_regular_file(inputFirstMeshPath))
    {
        ALICEVISION_LOG_ERROR("The first mesh file doesn't exist");
        return EXIT_FAILURE;
    }

    // check second mesh file path
    if(!inputSecondMeshPath.empty() && !fs::exists(inputSecondMeshPath) && !fs::is_regular_file(inputSecondMeshPath))
    {
        ALICEVISION_LOG_ERROR("The second mesh file doesn't exist");
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

    // check and set operation type
    const EOperationType operationType = EOperationType_stringToEnum(operationTypeName);

    GEO::initialize();
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");

    ALICEVISION_LOG_INFO("Geogram initialized.");

    GEO::Mesh inputFirstMesh, inputSecondMesh, outputMesh;
    
    // load first input mesh
    if(!GEO::mesh_load(inputFirstMeshPath, inputFirstMesh))
    {
        ALICEVISION_LOG_ERROR("Failed to load mesh file: \"" << inputFirstMeshPath << "\".");
        return EXIT_FAILURE;
    }

    // load second input mesh
    if(!GEO::mesh_load(inputSecondMeshPath, inputSecondMesh))
    {
        ALICEVISION_LOG_ERROR("Failed to load mesh file: \"" << inputSecondMeshPath << "\".");
        return EXIT_FAILURE;
    }
    
    // pre-process input meshes
    if(preProcess)
    {
        ALICEVISION_LOG_INFO("Pre-process input meshes...");
        fixMeshForBooleanOperations(inputFirstMesh);
        fixMeshForBooleanOperations(inputSecondMesh);
    }

    // merge mesh
    ALICEVISION_LOG_INFO("Merging meshes...");
    switch(operationType)
    {
        case EOperationType::BOOLEAN_UNION:
        {
          ALICEVISION_LOG_INFO("BOOLEAN_UNION");
          GEO::mesh_union(outputMesh, inputFirstMesh, inputSecondMesh);
        }
        break;
        case EOperationType::BOOLEAN_INTERSECTION:
        {
          ALICEVISION_LOG_INFO("BOOLEAN_INTERSECTION");
          GEO::mesh_intersection(outputMesh, inputFirstMesh, inputSecondMesh);
        }
        break;
        case EOperationType::BOOLEAN_DIFFERENCE:
        {
          ALICEVISION_LOG_INFO("BOOLEAN_DIFFERENCE");
          GEO::mesh_difference(outputMesh, inputFirstMesh, inputSecondMesh);
        }
        break;
    }

    // post-process final mesh
    if(postProcess)
    {
        ALICEVISION_LOG_INFO("Post-process final mesh...");
        fixMeshForBooleanOperations(outputMesh);
    }

    // save output mesh
    ALICEVISION_LOG_INFO("Save mesh.");
    if(!GEO::mesh_save(outputMesh, outputFilePath))
    {
        ALICEVISION_LOG_ERROR("Failed to save mesh file: \"" << outputFilePath << "\".");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputFilePath << "\" saved.");
    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));

    return EXIT_SUCCESS;
}