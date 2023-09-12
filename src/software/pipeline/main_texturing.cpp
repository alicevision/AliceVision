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
    std::string inputRefMeshFilepath;  // HighPoly for NormalMap
    aliceVision::mesh::EFileType outputMeshFileType;

    std::string outputFolder;
    std::string imagesFolder;
    std::string normalsFolder;

    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::SRGB;
    image::EImageColorSpace outputColorSpace = image::EImageColorSpace::AUTO;
    bool flipNormals = false;
    bool correctEV = true;

    mesh::TexturingParams texParams;
    std::string unwrapMethod = mesh::EUnwrapMethod_enumToString(mesh::EUnwrapMethod::Basic);
    std::string visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_enumToString(texParams.visibilityRemappingMethod);

    mesh::BumpMappingParams bumpMappingParams;
    image::EImageFileType normalFileType;
    image::EImageFileType heightFileType;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Dense point cloud SfMData file.")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath)->required(),
         "Input mesh to texture.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Folder for output mesh.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("inputRefMesh", po::value<std::string>(&inputRefMeshFilepath),
         "Optional input mesh to compute height maps and normal maps. "
         "If not provided, no additional maps with geometric information will be generated.")
        ("imagesFolder", po::value<std::string>(&imagesFolder),
         "Use images from a specific folder instead of those specify in the SfMData file.\n"
         "Filename should be the image UID.")
        ("normalsFolder", po::value<std::string>(&normalsFolder),
         "Use normal maps from a specific folder to texture the mesh.\n"
         "Filename should be: UID_normalMap.")
        ("textureSide", po::value<unsigned int>(&texParams.textureSide)->default_value(texParams.textureSide),
         "Output texture size.")
        ("downscale", po::value<unsigned int>(&texParams.downscale)->default_value(texParams.downscale),
         "Texture downscale factor.")
        ("outputMeshFileType", po::value<aliceVision::mesh::EFileType>(&outputMeshFileType)->default_value(aliceVision::mesh::EFileType::OBJ),
         "Output mesh file type.")
        ("colorMappingFileType", po::value<image::EImageFileType>(&texParams.textureFileType)->default_value(texParams.textureFileType),
         image::EImageFileType_informations().c_str())
        ("heightFileType", po::value<image::EImageFileType>(&heightFileType)->default_value(image::EImageFileType::NONE),
         image::EImageFileType_informations().c_str())
        ("normalFileType", po::value<image::EImageFileType>(&normalFileType)->default_value(image::EImageFileType::NONE),
         image::EImageFileType_informations().c_str())
        ("displacementMappingFileType", po::value<image::EImageFileType>(&bumpMappingParams.displacementFileType)->default_value(bumpMappingParams.displacementFileType),
         image::EImageFileType_informations().c_str())
        ("bumpType", po::value<mesh::EBumpMappingType>(&bumpMappingParams.bumpType)->default_value(bumpMappingParams.bumpType),
         "Use HeightMap for displacement or bump mapping.")
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
         "Texture edge padding size in pixels.")
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
        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         "Color space for the texturing internal computation (does not impact the output file color space).")
        ("outputColorSpace", po::value<image::EImageColorSpace>(&outputColorSpace)->default_value(outputColorSpace),
         "Output file colorspace.")
        ("correctEV", po::value<bool>(&correctEV)->default_value(correctEV),
         "Option to uniformize images exposure.")
        ("forceVisibleByAllVertices", po::value<bool>(&texParams.forceVisibleByAllVertices)->default_value(texParams.forceVisibleByAllVertices),
         "Triangle visibility is based on the union of vertices visiblity.")
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
         "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the "
         "convention changes from one software to another.")
        ("visibilityRemappingMethod", po::value<std::string>(&visibilityRemappingMethod)->default_value(visibilityRemappingMethod),
            "Method to remap visibilities from the reconstruction to the input mesh.\n"
            " * Pull: For each vertex of the input mesh, pull the visibilities from the closest vertex in the reconstruction.\n"
            " * Push: For each vertex of the reconstruction, push the visibilities to the closest triangle in the input mesh.\n"
         " * PullPush: Combine results from Pull and Push results.'")
        ("subdivisionTargetRatio", po::value<float>(&texParams.subdivisionTargetRatio)->default_value(texParams.subdivisionTargetRatio),
         "Percentage of the density of the reconstruction as the target for the subdivision "
         "(0: disable subdivision, 0.5: half density of the reconstruction, 1: full density of the reconstruction).");
    // clang-format on

    CmdLine cmdline("AliceVision texturing");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());
    oiio::attribute("threads", std::min(4, static_cast<int>(hwc.getMaxThreads())));
    oiio::attribute("exr_threads", std::min(4, static_cast<int>(hwc.getMaxThreads())));

    // set bump mapping file type
    bumpMappingParams.bumpMappingFileType = (bumpMappingParams.bumpType == mesh::EBumpMappingType::Normal) ? normalFileType : heightFileType;

    GEO::initialize();

    texParams.visibilityRemappingMethod = mesh::EVisibilityRemappingMethod_stringToEnum(visibilityRemappingMethod);
    texParams.workingColorSpace = workingColorSpace;
    texParams.outputColorSpace = outputColorSpace;

    texParams.correctEV = mvsUtils::ECorrectEV::NO_CORRECTION;
    if (correctEV)
    {
        texParams.correctEV = mvsUtils::ECorrectEV::APPLY_CORRECTION;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataFilename.empty())
    {
        ALICEVISION_LOG_INFO("Load dense point cloud.");
        if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL_DENSE))
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
    if (!mesh.hasUVs())
    {
        // Need visibilities to compute unwrap
        mesh.remapVisibilities(texParams.visibilityRemappingMethod, mp, refMesh);
        ALICEVISION_LOG_INFO("Input mesh has no UV coordinates, start unwrapping (" + unwrapMethod + ")");
        mesh.unwrap(mp, mesh::EUnwrapMethod_stringToEnum(unwrapMethod));
        ALICEVISION_LOG_INFO("Unwrapping done.");
    }

    if (texParams.subdivisionTargetRatio > 0)
    {
        const bool remapVisibilities = false;
        const int nbSubdiv = mesh.mesh->subdivideMesh(refMesh, texParams.subdivisionTargetRatio, remapVisibilities);
        ALICEVISION_LOG_INFO("Number of triangle subdivisions: " << nbSubdiv);

        mesh.updateAtlases();

        // need to recompute visibilities for all vertices
        mesh.mesh->pointsVisibilities.clear();
    }

    if (mesh.mesh->pointsVisibilities.empty())
    {
        mesh.remapVisibilities(texParams.visibilityRemappingMethod, mp, refMesh);

        // DEBUG: export subdivided mesh
        // mesh.saveAsOBJ(outputFolder, "subdividedMesh", outputTextureFileType);
    }

    // generate diffuse textures
    if (!inputMeshFilepath.empty() && !sfmDataFilename.empty() && texParams.textureFileType != image::EImageFileType::NONE)
    {
        ALICEVISION_LOG_INFO("Generate textures.");
        mesh.generateTextures(mp, outputFolder, hwc.getMaxMemory(), texParams.textureFileType);
    }

    if (!inputRefMeshFilepath.empty() && !inputMeshFilepath.empty() &&
        (bumpMappingParams.bumpMappingFileType != image::EImageFileType::NONE ||
         bumpMappingParams.displacementFileType != image::EImageFileType::NONE))
    {
        ALICEVISION_LOG_INFO("Generate height and normal maps.");

        mesh::Mesh denseMesh;
        denseMesh.load(inputRefMeshFilepath);

        mesh.generateNormalAndHeightMaps(mp, denseMesh, outputFolder, bumpMappingParams);
    }

    // generate normal maps textures from the fusion of normal maps per image
    if (!normalsFolder.empty() && bumpMappingParams.bumpMappingFileType != image::EImageFileType::NONE)
    {
        ALICEVISION_LOG_INFO("Generate normal maps.");
        mvsUtils::MultiViewParams mpN(sfmData, normalsFolder);
        mesh.generateTextures(mpN, outputFolder, hwc.getMaxMemory(), texParams.textureFileType, mvsUtils::EFileType::normalMap);
    }

    // save final obj file
    if (!inputMeshFilepath.empty())
    {
        mesh.saveAs(outputFolder, "texturedMesh", outputMeshFileType);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
