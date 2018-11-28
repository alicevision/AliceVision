// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/meshVisibility.hpp>
#include <aliceVision/mvsData/image.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <geogram/basic/common.h>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;

bfs::path absolutePathNoExt(const bfs::path& p)
{
    return p.parent_path() / p.stem();
}

int main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string inputDenseReconstruction;
    std::string inputMeshFilepath;
    std::string inputRefMeshFilepath;
    std::string outputFolder;
    std::string imagesFolder;
    std::string outTextureFileTypeName = EImageFileType_enumToString(EImageFileType::PNG);
    std::string outNormalMapFileTypeName = EImageFileType_enumToString(EImageFileType::PNG);
    std::string outHeightMapFileTypeName = EImageFileType_enumToString(EImageFileType::NONE);
    bool flipNormals = false;
    mesh::TexturingParams texParams;
    std::string unwrapMethod = mesh::EUnwrapMethod_enumToString(mesh::EUnwrapMethod::Basic);
    std::string visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_enumToString(texParams.visibilityRemappingMethod);

    po::options_description allParams("AliceVision texturing");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
          "SfMData file.")
        ("inputDenseReconstruction", po::value<std::string>(&inputDenseReconstruction),
            "Path to the dense reconstruction (mesh with per vertex visibility).")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Folder for output mesh: OBJ, material and texture files.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("imagesFolder", po::value<std::string>(&imagesFolder),
          "Use images from a specific folder instead of those specify in the SfMData file.\n"
          "Filename should be the image uid.")
        ("outputTextureFileType", po::value<std::string>(&outTextureFileTypeName)->default_value(outTextureFileTypeName),
          EImageFileType_informations().c_str())
        ("outputNormalMapFileType", po::value<std::string>(&outNormalMapFileTypeName)->default_value(outNormalMapFileTypeName),
          EImageFileType_informations().c_str())
        ("outputHeightMapFileType", po::value<std::string>(&outHeightMapFileTypeName)->default_value(outHeightMapFileTypeName),
          EImageFileType_informations().c_str())
        ("textureSide", po::value<unsigned int>(&texParams.textureSide)->default_value(texParams.textureSide),
          "Output texture size")
        ("downscale", po::value<unsigned int>(&texParams.downscale)->default_value(texParams.downscale),
            "Texture downscale factor")
        ("unwrapMethod", po::value<std::string>(&unwrapMethod)->default_value(unwrapMethod),
            "Method to unwrap input mesh if it does not have UV coordinates.\n"
            " * Basic (> 600k faces) fast and simple. Can generate multiple atlases.\n"
            " * LSCM (<= 600k faces): optimize space. Generates one atlas.\n"
            " * ABF (<= 300k faces): optimize space and stretch. Generates one atlas.'")
        ("fillHoles", po::value<bool>(&texParams.fillHoles)->default_value(texParams.fillHoles),
            "Fill texture holes with plausible values.")
        ("padding", po::value<unsigned int>(&texParams.padding)->default_value(texParams.padding),
            "Texture edge padding size in pixel")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath),
            "Optional input mesh to texture. By default, it will texture the inputReconstructionMesh.")
        ("inputRefMesh", po::value<std::string>(&inputRefMeshFilepath),
          "Optional input mesh to compute height maps and normal maps. If not provided, no additional maps with geometric information will be generated.")
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
            "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the convention change from one software to another.")
        ("maxNbImagesForFusion", po::value<int>(&texParams.maxNbImagesForFusion)->default_value(texParams.maxNbImagesForFusion),
            "Max number of images to combine to create the final texture.")
        ("bestScoreThreshold", po::value<double>(&texParams.bestScoreThreshold)->default_value(texParams.bestScoreThreshold),
            "(0.0 to disable filtering based on threshold to relative best score).")
        ("angleHardThreshold", po::value<double>(&texParams.angleHardThreshold)->default_value(texParams.angleHardThreshold),
            "(0.0 to disable angle hard threshold filtering).")
        ("forceVisibleByAllVertices", po::value<bool>(&texParams.forceVisibleByAllVertices)->default_value(texParams.forceVisibleByAllVertices),
            "triangle visibility is based on the union of vertices visiblity.")
        ("visibilityRemappingMethod", po::value<std::string>(&visibilityRemappingMethod)->default_value(visibilityRemappingMethod),
            "Method to remap visibilities from the reconstruction to the input mesh.\n"
            " * Pull: For each vertex of the input mesh, pull the visibilities from the closest vertex in the reconstruction.\n"
            " * Push: For each vertex of the reconstruction, push the visibilities to the closest triangle in the input mesh.\n"
            " * PullPush: Combine results from Pull and Push results.'");

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

    texParams.visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_stringToEnum(visibilityRemappingMethod);
    // set output texture file type
    const EImageFileType outputTextureFileType = (inputDenseReconstruction.empty() || sfmDataFilename.empty()) ? EImageFileType::NONE : EImageFileType_stringToEnum(outTextureFileTypeName);
    const EImageFileType outputNormalMapFileType = (inputRefMeshFilepath.empty()) ? EImageFileType::NONE : EImageFileType_stringToEnum(outNormalMapFileTypeName);
    const EImageFileType outputHeightMapFileType = (inputRefMeshFilepath.empty()) ? EImageFileType::NONE : EImageFileType_stringToEnum(outHeightMapFileTypeName);

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataFilename.empty())
    {
      if (!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
      {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
      }
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, imagesFolder);

    mesh::Texturing texturing;
    texturing.texParams = texParams;

    GEO::initialize();

    // load dense reconstruction
    if (!inputDenseReconstruction.empty())
    {
      ALICEVISION_LOG_INFO("Load input dense reconstruction: " << inputDenseReconstruction);
      const fs::path reconstructionMeshFolder = fs::path(inputDenseReconstruction).parent_path();
      texturing.loadFromMeshing(inputDenseReconstruction, (reconstructionMeshFolder/"meshPtsCamsFromDGC.bin").string());
    }

    fs::create_directory(outputFolder);

    // texturing from input mesh
    if(!inputMeshFilepath.empty())
    {
      ALICEVISION_LOG_INFO("Load input Mesh to texture: " << inputMeshFilepath);
      texturing.replaceMesh(inputMeshFilepath, flipNormals);
    }

    if(!texturing.hasUVs())
    {
      ALICEVISION_LOG_INFO("Input texturing has no UV coordinates, start unwrapping (" << unwrapMethod << ")");
      texturing.unwrap(mp, mesh::EUnwrapMethod_stringToEnum(unwrapMethod));
      ALICEVISION_LOG_INFO("Unwrapping done.");
    }

    // save final obj file
    if (!inputDenseReconstruction.empty() || !inputMeshFilepath.empty())
    {
      ALICEVISION_LOG_INFO("Save mesh as OBJ: " << outputTextureFileType);
      texturing.saveAsOBJ(outputFolder, "texturedMesh", outputTextureFileType, outputNormalMapFileType, outputHeightMapFileType);
    }

    // generate textures
    if (!inputDenseReconstruction.empty() && !sfmDataFilename.empty())
    {
      ALICEVISION_LOG_INFO("Generate textures.");
      texturing.generateTextures(mp, outputFolder, outputTextureFileType);
    }

    if(!inputRefMeshFilepath.empty() &&
       (outputNormalMapFileType != EImageFileType::NONE || outputHeightMapFileType != EImageFileType::NONE))
    {
      ALICEVISION_LOG_INFO("Generate height and normal maps.");
      mesh::Mesh denseMesh;
      {
        int nmtls;
        StaticVector<int> trisMtlIds;
        StaticVector<Point3d> normals;
        StaticVector<Voxel> trisNormalsIds;
        StaticVector<Point2d> uvCoords;
        StaticVector<Voxel> trisUvIds;
        if (!denseMesh.loadFromObjAscii(nmtls, trisMtlIds, normals, trisNormalsIds, uvCoords, trisUvIds, inputRefMeshFilepath))
        {
          throw std::runtime_error("Unable to load: " + inputRefMeshFilepath);
        }
      }
      ALICEVISION_LOG_INFO("Generate height and normal maps.");
      texturing.generateNormalAndHeightMaps(mp,
        denseMesh, outputFolder, outputNormalMapFileType, outputHeightMapFileType);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
