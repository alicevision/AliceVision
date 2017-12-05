// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_meshSmooth.hpp>
#include <aliceVision/largeScale/reconstructionPlan.hpp>
#include <aliceVision/planeSweeping/ps_refine_rc.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/common/fileIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;
namespace po = boost::program_options;

bool checkHardwareCompatibility()
{
    if(listCUDADevices(false) < 1)
    {
        std::cerr << "ERROR: no CUDA capable devices were detected." << std::endl;
        return false;
    }
    return true;
}

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
    bool flipNormals = false;
    TexturingParams texParams;

    po::options_description inputParams;

    inputParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file: mvs.ini (the undistorted images and camera poses should be in the same folder)).")
        ("inputDenseReconstruction", po::value<std::string>(&inputDenseReconstruction)->required(),
            "Path to the dense reconstruction (mesh with per vertex visibility).")
        ("output", po::value<std::string>(&outputFolder)->required(),
            "Folder for output mesh: OBJ, material and texture files.")
        ("textureSide", po::value<unsigned int>(&texParams.textureSide),
            "Output texture size")
        ("padding", po::value<unsigned int>(&texParams.padding),
            "Texture edge padding size in pixel")
        ("downscale", po::value<unsigned int>(&texParams.downscale),
            "Texture downscale factor")
        ("inputMesh", po::value<std::string>(&inputMeshFilepath),
            "Optional input mesh to texture. By default, it will texture the inputReconstructionMesh.")
        ("flipNormals", po::bool_switch(&flipNormals),
            "Option to flip face normals. It can be needed as it depends on the vertices order in triangles and the convention change from one software to another.");
    po::variables_map vm;

    try
    {
      po::store(po::parse_command_line(argc, argv, inputParams), vm);

      if(vm.count("help") || (argc == 1))
      {
        ALICEVISION_COUT(inputParams);
        return EXIT_SUCCESS;
      }

      po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }

    // check hardware compatibility
    if(!checkHardwareCompatibility())
        return EXIT_FAILURE;

    ALICEVISION_COUT("ini file: " << iniFilepath);
    ALICEVISION_COUT("inputMesh: " << inputMeshFilepath);

    // .ini parsing
    multiviewInputParams mip(iniFilepath);
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    multiviewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    mv_prematch_cams pc(&mp);

    meshRetex mesh;
    mesh.texParams = texParams;
    mesh.me = new mv_mesh();

    if(!mesh.me->loadFromBin(inputDenseReconstruction))
    {
        std::cerr << "Unable to load: " << inputDenseReconstruction << std::endl;
        return EXIT_FAILURE;
    }

    bfs::path reconstructionMeshFolder = bfs::path(inputDenseReconstruction).parent_path();

    staticVector<staticVector<int>*>* ptsCams = loadArrayOfArraysFromFile<int>((reconstructionMeshFolder/"meshPtsCamsFromDGC.bin").string());
    if(ptsCams->size() != mesh.me->pts->size())
        throw std::runtime_error("Error: Reference mesh and associated visibilities don't have the same size.");
    // filterPtsCamsByMinimalPixelSize(refMesh, refPtsCams, &mp);
    bfs::create_directory(outputFolder);

    //std::cout << "Points with no visibilities " << std::count(ptsCams->begin(), ptsCams->end(), nullptr) << std::endl;

    // texturing from input mesh
    if(!inputMeshFilepath.empty())
    {
        ALICEVISION_COUT("An external input mesh is provided, so we remap the visibility from the reconstruction on it.");
        // remap visibilities from reconstruction onto input mesh
        mv_delaunay_GC delaunayGC(&mp, &pc);
        delaunayGC.initTetrahedralizationFromMeshVertices(mesh.me, false);
        delete mesh.me;
        mesh.me = new mv_mesh();
        mesh.loadFromOBJ(inputMeshFilepath, flipNormals);
        staticVector<staticVector<int>*>* otherPtsCams = delaunayGC.createPtsCamsForAnotherMesh(ptsCams, *mesh.me);
        std::swap(ptsCams, otherPtsCams);    
        deleteArrayOfArrays<int>(&otherPtsCams);
    }
    if(!mesh.hasUVs())
    {
        ALICEVISION_COUT("The input mesh has no UV, so we generate them.");
        // generate UV coordinates based on automatic uv atlas
        auto* updatedPtsCams = mesh.generateUVs(mp, ptsCams);
        std::swap(ptsCams, updatedPtsCams);
        deleteArrayOfArrays<int>(&updatedPtsCams);
        mesh.saveAsOBJ(outputFolder, "texturedMesh");
    }

    // generate textures
    ALICEVISION_COUT("Generate textures.");
    mesh.generateTextures(mp, ptsCams, outputFolder);

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
