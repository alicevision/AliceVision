// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/mesh/MeshEnergyOpt.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/common/common.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace aliceVision;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl

int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string inputMeshPath;
    std::string outputMeshPath;

    int smoothNIter = 10;
    float lambda = 1.0f;

    po::options_description allParams("AliceVision meshFiltering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh (OBJ file format).")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("iterations", po::value<int>(&smoothNIter)->default_value(smoothNIter),
            "")
        ("lambda", po::value<float>(&lambda)->default_value(lambda),
            "");

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

    ALICEVISION_COUT("inputMeshPath: " << inputMeshPath);
    ALICEVISION_COUT("outputMeshPath: " << outputMeshPath);

    bfs::path outDirectory = bfs::path(outputMeshPath).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    mesh::Texturing texturing;
    texturing.me = new mesh::Mesh();
    texturing.loadFromOBJ(inputMeshPath);
    mesh::Mesh* mesh = texturing.me;

    if(!mesh)
    {
        std::cerr << "Error: unable to read input mesh from the file " << inputMeshPath << std::endl;
        return EXIT_FAILURE;
    }

//    if(mesh.n_vertices() == 0 || mesh.n_faces() == 0)
//    {
//        ALICEVISION_CERR("Error: empty mesh from the file " << inputMeshPath);
//        ALICEVISION_CERR("Input mesh: " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " facets.");
//        return EXIT_FAILURE;
//    }

    ALICEVISION_COUT("Mesh \"" << inputMeshPath << "\" loaded");

//    ALICEVISION_COUT("Input mesh: " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " facets.");

    mesh::MeshEnergyOpt meOpt(nullptr);
    {
        ALICEVISION_COUT("Start mesh filtering.");
        meOpt.addMesh(mesh);
        meOpt.init();
        meOpt.cleanMesh(10);

        StaticVectorBool* ptsCanMove = nullptr;
        float epsilon = 0.0f; // unused
        int type = 3; // 0, 1, 2, 3, 4 => only 1 and 3 works
        meOpt.optimizeSmooth(lambda, epsilon, type, smoothNIter, ptsCanMove);

        ALICEVISION_COUT("Mesh filtering done.");
    }

    mesh::Mesh outMesh;
    outMesh.addMesh(&meOpt);

//    ALICEVISION_COUT("Output mesh: " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " facets.");

//    if(mesh.n_faces() == 0)
//    {
//        ALICEVISION_CERR("Failed: the output mesh is empty.");
//        return EXIT_FAILURE;
//    }

    ALICEVISION_COUT("Save mesh");

    // Save output mesh
    outMesh.saveToObj(outputMeshPath);

    ALICEVISION_CERR("Mesh \"" << outputMeshPath << "\" saved.");

    common::printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
