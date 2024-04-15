// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <boost/program_options.hpp>
#include <boost/functional/hash.hpp>

#include <list>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = std::filesystem;
namespace po = boost::program_options;

using Edge = std::pair<int, int>;

struct EdgeHash 
{
    std::size_t operator()(const Edge & e) const
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, e.first);
        boost::hash_combine(seed, e.second);
        return seed;
    }
};


using MapEdges = std::unordered_map<Edge, std::unordered_set<int>, EdgeHash>;

struct Manifold
{
    using sptr = std::shared_ptr<Manifold>;

    std::vector<int> faceIndices;
};



struct DividingLine
{
    Edge edge;

    std::vector<Manifold::sptr> manifolds;
};



int aliceVision_main(int argc, char* argv[])
{
    std::string inputMeshPath;
    std::string outputMeshPath;

    

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputMeshPath)->required(),
         "Input Mesh (OBJ file format).")
        ("output,o", po::value<std::string>(&outputMeshPath)->required(),
         "Output mesh (OBJ file format).");
    // clang-format on

    CmdLine cmdline("AliceVision meshRepairing");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    mesh::Mesh inputMesh;
    inputMesh.load(inputMeshPath);

    
    ALICEVISION_LOG_INFO("Mesh with " << inputMesh.tris.size() << " triangles and "  << inputMesh.pts.size() << " vertices");

    MapEdges mapEdges;
    std::vector<bool> visitedFaces(inputMesh.tris.size());

    for (int idFace = 0; idFace < inputMesh.tris.size(); idFace++)
    {
        const mesh::Mesh::triangle & triangle = inputMesh.tris[idFace];
        visitedFaces[idFace] = false;
        
        for (int posVertex = 0; posVertex < 3; posVertex++)
        {
            int posNextVertex = posVertex + 1;
            if (posNextVertex == 3) posNextVertex = 0;

            int vertexId = triangle.v[posVertex];
            int nextVertexId = triangle.v[posNextVertex];

            Edge edge = std::make_pair(vertexId, nextVertexId);

            mapEdges[edge].insert(idFace);
        }
    }

    std::vector<Manifold::sptr> manifolds;
    while (1)
    {
        int firstNonVisited = -1;
        for (int idFaceCheck = 0; idFaceCheck < visitedFaces.size(); idFaceCheck++)
        {
            if (!visitedFaces[idFaceCheck])
            {
                firstNonVisited = idFaceCheck;
            }
        }
        if (firstNonVisited < 0)
        {
            break;
        }

        Manifold::sptr manifold = std::make_shared<Manifold>();
        std::list<int> toProcess;
        toProcess.push_back(firstNonVisited);

        while (1)
        {
            if (toProcess.empty())
            {
                break;
            }

            //Choose the next face to process
            int idFace = toProcess.back();
            toProcess.pop_back();
            if (visitedFaces[idFace])
            {
                continue;
            }

            //Process face
            const mesh::Mesh::triangle & triangle = inputMesh.tris[idFace];
            visitedFaces[idFace] = true;
            manifold->faceIndices.push_back(idFace);

            for (int posVertex = 0; posVertex < 3; posVertex++)
            {
                int posNextVertex = posVertex + 1;
                if (posNextVertex == 3) posNextVertex = 0;

                int vertexId = triangle.v[posVertex];
                int nextVertexId = triangle.v[posNextVertex];

                //First, try to find if this edge is weird
                Edge edgeinv = std::make_pair(nextVertexId, vertexId);
                Edge edge = std::make_pair(vertexId, nextVertexId);
                
                //If there is another face with the same directed edge ?
                const auto & setDirect = mapEdges[edge];
                if (setDirect.size() > 1)
                {
                    continue;
                }

                const auto & setIndirect = mapEdges[edgeinv];
                if (setIndirect.size() > 1)
                {
                    //This edge is not manifold
                    continue;
                }
                
                for (int idOtherFace : mapEdges[edgeinv])
                {
                    toProcess.push_back(idOtherFace);
                }
            }

            
        }

        ALICEVISION_LOG_INFO(manifold->faceIndices.size());
        manifolds.push_back(manifold);
    }

    return EXIT_SUCCESS;
}
