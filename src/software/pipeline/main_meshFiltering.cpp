// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mvsUtils/common.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 4
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

enum class ESubsetType : unsigned char
{
    ALL = 0,
    SURFACE_BOUNDARIES = 1,
    SURFACE_INNER_PART = 2
};

/**
 * @brief get informations about each subset type
 * @return String
 */
std::string ESubsetType_informations()
{
    return "Subset types used:\n"
           "* all: the entire mesh.\n"
           "* surface_boundaries: mesh surface boundaries.\n"
           "* surface_inner_part: mesh surface inner part.\n";
}

/**
 * @brief convert an enum ESubsetType to its corresponding string
 * @param ESubsetType
 * @return String
 */
std::string ESubsetType_enumToString(ESubsetType subsetType)
{
  switch(subsetType)
  {
      case ESubsetType::ALL:                 return "all";
      case ESubsetType::SURFACE_BOUNDARIES:  return "surface_boundaries";
      case ESubsetType::SURFACE_INNER_PART:  return "surface_inner_part";
  }
  throw std::out_of_range("Invalid SubsetType enum: " + std::to_string(int(subsetType)));
}

/**
 * @brief convert a string subsetType to its corresponding enum ESubsetType
 * @param String
 * @return ESubsetType
 */
ESubsetType ESubsetType_stringToEnum(const std::string& subsetType)
{
  std::string type = subsetType;
  std::transform(type.begin(), type.end(), type.begin(), ::tolower); // tolower

  if(type == "all")                 return ESubsetType::ALL;
  if(type == "surface_boundaries")  return ESubsetType::SURFACE_BOUNDARIES;
  if(type == "surface_inner_part")  return ESubsetType::SURFACE_INNER_PART;
  throw std::out_of_range("Invalid filterType: " + subsetType);
}

std::ostream& operator<<(std::ostream& os, const ESubsetType subsetType)
{
    os << ESubsetType_enumToString(subsetType);
    return os;
}

std::istream& operator>>(std::istream& in, ESubsetType& subsetType)
{
    std::string token;
    in >> token;
    subsetType = ESubsetType_stringToEnum(token);
    return in;
}

int aliceVision_main(int argc, char* argv[])
{
    // timer initialization

    system::Timer timer;

    // command-line required parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputMeshPath;
    std::string outputMeshPath;

    bool keepLargestMeshOnly = false;

    // command-line smoothing parameters

    std::string smoothingSubsetTypeName = ESubsetType_enumToString(ESubsetType::ALL);
    int smoothingBoundariesNeighbours = 0;
    int smoothNIter = 10;
    float lambda = 1.0f;

    // command-line filtering parameters

    std::string filteringSubsetTypeName = ESubsetType_enumToString(ESubsetType::ALL);
    int filteringIterations = 1;
    double filterLargeTrianglesFactor = 60.0;
    double filterTrianglesRatio = 0.0;

    po::options_description allParams("AliceVision meshFiltering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("keepLargestMeshOnly", po::value<bool>(&keepLargestMeshOnly)->default_value(keepLargestMeshOnly),
            "Keep only the largest connected triangles group.")
        ("smoothingSubset",po::value<std::string>(&smoothingSubsetTypeName)->default_value(smoothingSubsetTypeName),
            ESubsetType_informations().c_str())
        ("smoothingBoundariesNeighbours", po::value<int>(&smoothingBoundariesNeighbours)->default_value(smoothingBoundariesNeighbours),
            "Neighbours of the boudaries to consider.")
        ("smoothingIterations", po::value<int>(&smoothNIter)->default_value(smoothNIter),
            "Number of smoothing iterations.")
        ("smoothingLambda", po::value<float>(&lambda)->default_value(lambda),
            "Smoothing size.")
        ("filteringSubset",po::value<std::string>(&filteringSubsetTypeName)->default_value(filteringSubsetTypeName),
            ESubsetType_informations().c_str())
        ("filteringIterations", po::value<int>(&filteringIterations)->default_value(filteringIterations),
            "Number of mesh filtering iterations.")
        ("filterLargeTrianglesFactor", po::value<double>(&filterLargeTrianglesFactor)->default_value(filterLargeTrianglesFactor),
            "Remove all large triangles. We consider a triangle as large if one edge is bigger than N times the average edge length. Put zero to disable it.")
        ("filterTrianglesRatio", po::value<double>(&filterTrianglesRatio)->default_value(filterTrianglesRatio),
            "Remove all triangles by ratio (largest edge /smallest edge). Put zero to disable it.");

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

    // check and set smoothing subset type
    const ESubsetType smoothingSubsetType = ESubsetType_stringToEnum(smoothingSubsetTypeName);

    // check and set filtering subset type
    const ESubsetType filteringSubsetType = ESubsetType_stringToEnum(filteringSubsetTypeName);

    bfs::path outDirectory = bfs::path(outputMeshPath).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    mesh::Texturing texturing;
    texturing.loadWithAtlas(inputMeshPath);
    mesh::Mesh* mesh = texturing.mesh;

    if(!mesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    if(mesh->pts.empty() || mesh->tris.empty())
    {
        ALICEVISION_LOG_ERROR("Error: empty mesh from the file " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");
    ALICEVISION_LOG_INFO("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");

    StaticVectorBool ptsCanMove; // empty if smoothingSubsetType is ALL

    // lock filter subset vertices
    switch(smoothingSubsetType)
    {
        case ESubsetType::ALL: break; // nothing to lock

        case ESubsetType::SURFACE_BOUNDARIES:
        mesh->lockSurfaceBoundaries(smoothingBoundariesNeighbours, ptsCanMove, true); // invert = true (lock surface inner part)
        break;

        case ESubsetType::SURFACE_INNER_PART:
        mesh->lockSurfaceBoundaries(smoothingBoundariesNeighbours, ptsCanMove, false); // invert = false (lock surface boundaries)
        break;
    }
    

    // filtering
    if((filterLargeTrianglesFactor != 0.0) || (filterTrianglesRatio != 0.0))
    {
        ALICEVISION_LOG_INFO("Start mesh filtering.");

        for(int i = 0; i < filteringIterations; ++i)
        {
          ALICEVISION_LOG_INFO("Mesh filtering: iteration " << i);

          StaticVectorBool trisToStay(mesh->tris.size(), true);
          StaticVectorBool trisInFilterSubset; // empty if filteringSubsetType is ALL

          switch(filteringSubsetType)
          {
              case ESubsetType::ALL: break; // nothing to do 

              case ESubsetType::SURFACE_BOUNDARIES:
              mesh->getSurfaceBoundaries(trisInFilterSubset); // invert = false (get surface boundaries)
              break;

              case ESubsetType::SURFACE_INNER_PART:
              mesh->getSurfaceBoundaries(trisInFilterSubset, true); // invert = true (get surface inner part)
              break;
          }

          if(filterLargeTrianglesFactor != 0.0)
              mesh->filterLargeEdgeTriangles(filterLargeTrianglesFactor, trisInFilterSubset, trisToStay);

          if(filterTrianglesRatio != 0.0)
              mesh->filterTrianglesByRatio(filterTrianglesRatio, trisInFilterSubset, trisToStay);

          mesh->letJustTringlesIdsInMesh(trisToStay);
        }
        ALICEVISION_LOG_INFO("Mesh filtering done: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");
    }

    // smoothing
    mesh::MeshEnergyOpt meOpt(nullptr);
    {
        ALICEVISION_LOG_INFO("Start mesh smoothing.");
        meOpt.addMesh(*mesh);
        meOpt.init();
        meOpt.cleanMesh(10);
        meOpt.optimizeSmooth(lambda, smoothNIter, ptsCanMove);
        ALICEVISION_LOG_INFO("Mesh smoothing done: " << meOpt.pts.size() << " vertices and " << meOpt.tris.size() << " facets.");
    }

    if(keepLargestMeshOnly)
    {
        StaticVector<int> trisIdsToStay;
        meOpt.getLargestConnectedComponentTrisIds(trisIdsToStay);
        meOpt.letJustTringlesIdsInMesh(trisIdsToStay);
        ALICEVISION_LOG_INFO("Mesh after keepLargestMeshOnly: " << meOpt.pts.size() << " vertices and " << meOpt.tris.size() << " facets.");
    }
    
    // clear potential free points created by triangles removal in previous cleaning operations 
    StaticVector<int> ptIdToNewPtId;
    meOpt.removeFreePointsFromMesh(ptIdToNewPtId);
    ptIdToNewPtId.clear();

    mesh::Mesh outMesh;
    outMesh.addMesh(meOpt);

    ALICEVISION_COUT("Output mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");

    if(outMesh.pts.empty() || outMesh.tris.empty())
    {
        ALICEVISION_CERR("Failed: the output mesh is empty.");
        ALICEVISION_LOG_INFO("Output mesh: " << outMesh.pts.size() << " vertices and " << outMesh.tris.size() << " facets.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Save mesh.");

    // Save output mesh
    outMesh.save(outputMeshPath);

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
