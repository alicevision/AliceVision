// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/VectorT.hh>
#include <OpenMesh/Tools/Decimater/DecimaterT.hh>
#include <OpenMesh/Tools/Decimater/ModQuadricT.hh>

#include <aliceVision/common/common.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;
namespace po = boost::program_options;
using namespace aliceVision;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string inputMeshPath;
    std::string outputMeshPath;
    float simplificationFactor = 0;
    int fixedNbVertices = 0;
    int minVertices = 0;
    int maxVertices = 0;
    bool flipNormals = false;

    po::options_description allParams("AliceVision meshResampling");

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
            "Min number of output vertices.")
        ("maxVertices", po::value<int>(&maxVertices)->default_value(maxVertices),
            "Max number of output vertices.")
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

    bfs::path outDirectory = bfs::path(outputMeshPath).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    // Mesh type
    typedef OpenMesh::TriMesh_ArrayKernelT<>                      Mesh;
    // Decimater type
    typedef OpenMesh::Decimater::DecimaterT< Mesh >               Decimater;
    // Decimation Module Handle type
    typedef OpenMesh::Decimater::ModQuadricT< Mesh >::Handle HModQuadric;

    OpenMesh::IO::_OBJReader_();
    OpenMesh::IO::_OBJWriter_();

    Mesh mesh;
    if(!OpenMesh::IO::read_mesh(mesh, inputMeshPath.c_str()))
    {
        std::cerr << "Error: unable to read input mesh from the file " << inputMeshPath << std::endl;
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Mesh \"" << inputMeshPath << "\" loaded");

    int nbInputPoints = mesh.n_vertices();
    int nbOutputPoints = 0;
    if(fixedNbVertices != 0)
    {
        nbOutputPoints = fixedNbVertices;
    }
    else
    {
        if(simplificationFactor != 0.0)
        {
            nbOutputPoints = simplificationFactor * nbInputPoints;
        }
        if(minVertices != 0)
        {
            if(nbInputPoints > minVertices && nbOutputPoints < minVertices)
              nbOutputPoints = minVertices;
        }
        if(maxVertices != 0)
        {
          if(nbInputPoints > maxVertices && nbOutputPoints > maxVertices)
            nbOutputPoints = maxVertices;
        }
    }

    ALICEVISION_COUT("Input mesh: " << nbInputPoints << " vertices and " << mesh.n_faces() << " facets.");
    ALICEVISION_COUT("Target output mesh: " << nbOutputPoints << " vertices.");

    {
        // a decimater object, connected to a mesh
        Decimater   decimater(mesh);
        // use a quadric module
        HModQuadric hModQuadric;
        // register module at the decimater
        decimater.add(hModQuadric);

        // the way to access the module
        std::cout << decimater.module(hModQuadric).name() << std::endl;

        /*
         * since we need exactly one priority module (non-binary)
         * we have to call set_binary(false) for our priority module
         * in the case of HModQuadric, unset_max_err() calls set_binary(false) internally
         */
        decimater.module(hModQuadric).unset_max_err();
        // let the decimater initialize the mesh and the modules
        decimater.initialize();
        // do decimation
        size_t removedVertices = decimater.decimate_to(nbOutputPoints);
        decimater.mesh().garbage_collection();
    }
    ALICEVISION_COUT("Output mesh: " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " facets.");

    if(mesh.n_faces() == 0)
    {
        ALICEVISION_CERR("Failed: the output mesh is empty.");
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Save mesh");
    // Save output mesh
    if(!OpenMesh::IO::write_mesh(mesh, outputMeshPath))
    {
        ALICEVISION_CERR("Failed to save mesh \"" << outputMeshPath << "\".");
        return EXIT_FAILURE;
    }
    ALICEVISION_CERR("Mesh \"" << outputMeshPath << "\" saved.");

    common::printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
