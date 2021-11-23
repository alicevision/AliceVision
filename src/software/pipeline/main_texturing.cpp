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
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/sfmMvsUtils/visibility.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <geogram/basic/common.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

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

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;

    std::string inputMeshFilepath;    // Model to texture (HighPoly for diffuse, LowPoly for Diffuse+Normal)
    std::string inputRefMeshFilepath; // HighPoly for NormalMap
    aliceVision::mesh::EFileType outputMeshFileType;

    std::string outputFolder;
    std::string imagesFolder;
    std::string processColorspaceName = imageIO::EImageColorSpace_enumToString(imageIO::EImageColorSpace::SRGB);
    bool flipNormals = false;
    bool correctEV = false;

    mesh::TexturingParams texParams;
    std::string unwrapMethod = mesh::EUnwrapMethod_enumToString(mesh::EUnwrapMethod::Basic);
    std::string visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_enumToString(texParams.visibilityRemappingMethod);

    mesh::BumpMappingParams bumpMappingParams;
    imageIO::EImageFileType normalFileType;
    imageIO::EImageFileType heightFileType;

    po::options_description allParams("AliceVision texturing");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
          "Dense point cloud SfMData file.")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath)->required(),
            "Input mesh to texture.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Folder for output mesh");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("inputRefMesh", po::value<std::string>(&inputRefMeshFilepath),
            "Optional input mesh to compute height maps and normal maps. If not provided, no additional maps with geometric information will be generated.")
        ("imagesFolder", po::value<std::string>(&imagesFolder),
          "Use images from a specific folder instead of those specify in the SfMData file.\n"
          "Filename should be the image uid.")
        ("textureSide", po::value<unsigned int>(&texParams.textureSide)->default_value(texParams.textureSide),
            "Output texture size")
        ("downscale", po::value<unsigned int>(&texParams.downscale)->default_value(texParams.downscale),
            "Texture downscale factor")
        ("outputMeshFileType", po::value<aliceVision::mesh::EFileType>(&outputMeshFileType)->default_value(aliceVision::mesh::EFileType::OBJ),
            "output mesh file type")
        ("colorMappingFileType", po::value<imageIO::EImageFileType>(&texParams.textureFileType)->default_value(texParams.textureFileType),
          imageIO::EImageFileType_informations().c_str())(
        "heightFileType", po::value<imageIO::EImageFileType>(&heightFileType)->default_value(imageIO::EImageFileType::NONE),
            imageIO::EImageFileType_informations().c_str())
        ("normalFileType", po::value<imageIO::EImageFileType>(&normalFileType)->default_value(imageIO::EImageFileType::NONE),
            imageIO::EImageFileType_informations().c_str())
        ("displacementMappingFileType", po::value<imageIO::EImageFileType>(&bumpMappingParams.displacementFileType)->default_value(bumpMappingParams.displacementFileType),
            imageIO::EImageFileType_informations().c_str())
        ("bumpType", po::value<mesh::EBumpMappingType>(&bumpMappingParams.bumpType)->default_value(bumpMappingParams.bumpType),
            "Use HeightMap for displacement or bump mapping")
        ("unwrapMethod", po::value<std::string>(&unwrapMethod)->default_value(unwrapMethod),
            "Method to unwrap input mesh if it does not have UV coordinates.\n"
            " * Basic (> 600k faces) fast and simple. Can generate multiple atlases.\n"
            " * LSCM (<= 600k faces): optimize space. Generates one atlas.\n"
            " * ABF (<= 300k faces): optimize space and stretch. Generates one atlas.'")
        ("useUDIM", po::value<bool>(&texParams.useUDIM)->default_value(texParams.useUDIM),
            "Use UDIM UV mapping.")
        ("fillHoles", po::value<bool>(&texParams.fillHoles)->default_value(texParams.fillHoles),
            "Fill texture holes with plausible values.")
        ("padding", po::value<unsigned int>(&texParams.padding)->default_value(texParams.padding),
            "Texture edge padding size in pixel")
        ("multiBandDownscale", po::value<unsigned int>(&texParams.multiBandDownscale)->default_value(texParams.multiBandDownscale),
            "Width of frequency bands.")
        ("multiBandNbContrib", po::value<std::vector<int>>(&texParams.multiBandNbContrib)->default_value(texParams.multiBandNbContrib)->multitoken(),
             "Number of contributions per frequency band.")
        ("useScore", po::value<bool>(&texParams.useScore)->default_value(texParams.useScore),
             "Use triangles scores (based on observations and re-projected areas in source images) for weighting contributions.")
        ("bestScoreThreshold", po::value<double>(&texParams.bestScoreThreshold)->default_value(texParams.bestScoreThreshold),
            "(0.0 to disable filtering based on threshold to relative best score).")
        ("angleHardThreshold", po::value<double>(&texParams.angleHardThreshold)->default_value(texParams.angleHardThreshold),
            "(0.0 to disable angle hard threshold filtering).")
        ("processColorspace", po::value<std::string>(&processColorspaceName)->default_value(processColorspaceName),
            "Colorspace for the texturing internal computation (does not impact the output file colorspace).")
        ("correctEV", po::value<bool>(&correctEV)->default_value(correctEV),
            "Option to uniformize images exposure.")
        ("forceVisibleByAllVertices", po::value<bool>(&texParams.forceVisibleByAllVertices)->default_value(texParams.forceVisibleByAllVertices),
            "triangle visibility is based on the union of vertices visiblity.")
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
            "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the convention change from one software to another.")
        ("visibilityRemappingMethod", po::value<std::string>(&visibilityRemappingMethod)->default_value(visibilityRemappingMethod),
            "Method to remap visibilities from the reconstruction to the input mesh.\n"
            " * Pull: For each vertex of the input mesh, pull the visibilities from the closest vertex in the reconstruction.\n"
            " * Push: For each vertex of the reconstruction, push the visibilities to the closest triangle in the input mesh.\n"
            " * PullPush: Combine results from Pull and Push results.'")
        ("subdivisionTargetRatio", po::value<float>(&texParams.subdivisionTargetRatio)->default_value(texParams.subdivisionTargetRatio),
            "Percentage of the density of the reconstruction as the target for the subdivision (0: disable subdivision, 0.5: half density of the reconstruction, 1: full density of the reconstruction).");

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

    // set bump mapping file type
    bumpMappingParams.bumpMappingFileType = (bumpMappingParams.bumpType == mesh::EBumpMappingType::Normal) ? normalFileType : heightFileType;

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    GEO::initialize();

    texParams.visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_stringToEnum(visibilityRemappingMethod);
    texParams.processColorspace = imageIO::EImageColorSpace_stringToEnum(processColorspaceName);

    texParams.correctEV = mvsUtils::ECorrectEV::NO_CORRECTION;
    if(correctEV) { texParams.correctEV = mvsUtils::ECorrectEV::APPLY_CORRECTION; }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if(!sfmDataFilename.empty())
    {
        ALICEVISION_LOG_INFO("Load dense point cloud.");
        if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL_DENSE))
        {
          ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
          return EXIT_FAILURE;
        }
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, imagesFolder);

    mesh::Texturing mesh;
    mesh.texParams = texParams;

    fs::create_directory(outputFolder);

    // load input mesh (to texture) obj file
    ALICEVISION_LOG_INFO("Load input mesh.");
    mesh.clear();
    mesh.loadWithAtlas(inputMeshFilepath, flipNormals);

    // load reference dense point cloud with visibilities
    ALICEVISION_LOG_INFO("Convert dense point cloud into ref mesh");
    mesh::Mesh refMesh;
    mvsUtils::createRefMeshFromDenseSfMData(refMesh, sfmData, mp);

    // generate UVs if necessary
    if(!mesh.hasUVs())
    {
        // Need visibilities to compute unwrap
        mesh.remapVisibilities(texParams.visibilityRemappingMethod, mp, refMesh);
        ALICEVISION_LOG_INFO("Input mesh has no UV coordinates, start unwrapping (" + unwrapMethod + ")");
        mesh.unwrap(mp, mesh::EUnwrapMethod_stringToEnum(unwrapMethod));
        ALICEVISION_LOG_INFO("Unwrapping done.");
    }

    // save final obj file
    if(!inputMeshFilepath.empty())
    {
        mesh.saveAs(outputFolder, "texturedMesh", outputMeshFileType, texParams.textureFileType, bumpMappingParams);
    }

    if(texParams.subdivisionTargetRatio > 0)
    {
        const bool remapVisibilities = false;
        const int nbSubdiv = mesh.mesh->subdivideMesh(refMesh, texParams.subdivisionTargetRatio, remapVisibilities);
        ALICEVISION_LOG_INFO("Number of triangle subdivisions: " << nbSubdiv);

        mesh.updateAtlases();

        // need to recompute visibilities for all vertices
        mesh.mesh->pointsVisibilities.clear();
    }

    if(mesh.mesh->pointsVisibilities.empty())
    {
        mesh.remapVisibilities(texParams.visibilityRemappingMethod, mp, refMesh);

        // DEBUG: export subdivided mesh
        // mesh.saveAsOBJ(outputFolder, "subdividedMesh", outputTextureFileType);
    }

    // generate diffuse textures
    if(!inputMeshFilepath.empty() && !sfmDataFilename.empty() && texParams.textureFileType != imageIO::EImageFileType::NONE)
    {
        ALICEVISION_LOG_INFO("Generate textures.");
        mesh.generateTextures(mp, outputFolder, texParams.textureFileType);
    }


    if(!inputRefMeshFilepath.empty() && !inputMeshFilepath.empty() &&
       (bumpMappingParams.bumpMappingFileType != imageIO::EImageFileType::NONE ||
        bumpMappingParams.displacementFileType != imageIO::EImageFileType::NONE))
    {
        ALICEVISION_LOG_INFO("Generate height and normal maps.");

        mesh::Mesh denseMesh;
        denseMesh.load(inputRefMeshFilepath);

        mesh.generateNormalAndHeightMaps(mp, denseMesh, outputFolder, bumpMappingParams);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
