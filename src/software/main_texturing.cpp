// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/structures/image.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/MultiViewParams.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/meshVisibility.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace aliceVision;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;

bfs::path absolutePathNoExt(const bfs::path& p)
{
    return p.parent_path() / p.stem();
}

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl

int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;
    std::string inputDenseReconstruction;
    std::string inputMeshFilepath;
    std::string outputFolder;
    std::string outTextureFileTypeName = EImageFileType_enumToString(EImageFileType::PNG);
    bool flipNormals = false;
    TexturingParams texParams;

    po::options_description allParams("AliceVision texturing");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file: mvs.ini (the undistorted images and camera poses should be in the same folder)).")
        ("inputDenseReconstruction", po::value<std::string>(&inputDenseReconstruction)->required(),
            "Path to the dense reconstruction (mesh with per vertex visibility).")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Folder for output mesh: OBJ, material and texture files.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("outputTextureFileType", po::value<std::string>(&outTextureFileTypeName)->default_value(outTextureFileTypeName),
          EImageFileType_informations().c_str())
        ("textureSide", po::value<unsigned int>(&texParams.textureSide)->default_value(texParams.textureSide),
            "Output texture size")
        ("padding", po::value<unsigned int>(&texParams.padding)->default_value(texParams.padding),
            "Texture edge padding size in pixel")
        ("downscale", po::value<unsigned int>(&texParams.downscale)->default_value(texParams.downscale),
            "Texture downscale factor")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath),
            "Optional input mesh to texture. By default, it will texture the inputReconstructionMesh.")
        ("flipNormals", po::value<bool>(&flipNormals)->default_value(flipNormals),
            "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the convention change from one software to another.");

    allParams.add(requiredParams).add(optionalParams);

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

    ALICEVISION_COUT("ini file: " << iniFilepath);
    ALICEVISION_COUT("inputMesh: " << inputMeshFilepath);

    // set output texture file type
    const EImageFileType outputTextureFileType = EImageFileType_stringToEnum(outTextureFileTypeName);

    // .ini parsing
    MultiViewInputParams mip(iniFilepath, "", "");
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    MultiViewParams mp(mip.getNbCameras(), &mip, (float) simThr);

    Texturing mesh;
    mesh.texParams = texParams;
    mesh.me = new Mesh();

    if(!mesh.me->loadFromBin(inputDenseReconstruction))
    {
        std::cerr << "Unable to load: " << inputDenseReconstruction << std::endl;
        return EXIT_FAILURE;
    }

    bfs::path reconstructionMeshFolder = bfs::path(inputDenseReconstruction).parent_path();

    mesh::PointsVisibility* ptsCams = loadArrayOfArraysFromFile<int>((reconstructionMeshFolder/"meshPtsCamsFromDGC.bin").string());
    if(ptsCams->size() != mesh.me->pts->size())
        throw std::runtime_error("Error: Reference mesh and associated visibilities don't have the same size.");
    // filterPtsCamsByMinimalPixelSize(refMesh, refPtsCams, &mp);
    bfs::create_directory(outputFolder);

    //std::cout << "Points with no visibilities " << std::count(ptsCams->begin(), ptsCams->end(), nullptr) << std::endl;

    // texturing from input mesh
    if(!inputMeshFilepath.empty())
    {
        ALICEVISION_COUT("An external input mesh is provided, so we remap the visibility from the reconstruction on it.");
        // keep previous mesh as reference
        Mesh* refMesh = mesh.me;
        // load input obj file
        mesh.me = new Mesh();
        mesh.loadFromOBJ(inputMeshFilepath, flipNormals);
        // remap visibilities from reconstruction onto input mesh
        mesh::PointsVisibility otherPtsVisibilities;
        mesh::remapMeshVisibilities(*refMesh, *ptsCams, *mesh.me, otherPtsVisibilities);

        delete refMesh;
        ptsCams->swap(otherPtsVisibilities);
    }
    if(!mesh.hasUVs())
    {
        ALICEVISION_COUT("The input mesh has no UV, so we generate them.");
        // generate UV coordinates based on automatic uv atlas
        auto* updatedPtsCams = mesh.generateUVs(mp, ptsCams);
        std::swap(ptsCams, updatedPtsCams);
        deleteArrayOfArrays<int>(&updatedPtsCams);
        mesh.saveAsOBJ(outputFolder, "texturedMesh", outputTextureFileType);
    }

    // generate textures
    ALICEVISION_COUT("Generate textures.");
    mesh.generateTextures(mp, ptsCams, outputFolder, outputTextureFileType);

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
