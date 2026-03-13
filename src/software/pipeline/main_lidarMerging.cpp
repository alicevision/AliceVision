// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/fuseCut/InputSet.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <boost/program_options.hpp>
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/geometry/Intersection.hpp>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

inline size_t computeHash(const Point3d& pt)
{
    size_t seed = 0;
    stl::hash_combine(seed, size_t(pt.x * 100.0));
    stl::hash_combine(seed, size_t(pt.y * 100.0));
    stl::hash_combine(seed, size_t(pt.z * 100.0));
    return seed;
}

inline size_t computeHash(const mesh::Mesh::triangle& tri)
{
    size_t seed = 0;
    stl::hash_combine(seed, tri.v[0]);
    stl::hash_combine(seed, tri.v[1]);
    stl::hash_combine(seed, tri.v[2]);
    return seed;
}

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string jsonFilename = "";
    std::string outputMeshFilename = "";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&jsonFilename)->required(),
         "Input JSON file.")
        ("output,o", po::value<std::string>(&outputMeshFilename)->required(),
         "Output mesh file.");
    // clang-format on

    CmdLine cmdline("AliceVision lidarMeshing");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    // omp_set_num_threads(std::min(32u, hwc.getMaxThreads()));

    std::ifstream inputfile(jsonFilename);
    if (!inputfile.is_open())
    {
        ALICEVISION_LOG_ERROR("Cannot open JSON input file");
        return EXIT_FAILURE;
    }

    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    fuseCut::InputSet inputsets(boost::json::value_to<fuseCut::InputSet>(jv));

    int setSize = static_cast<int>(inputsets.size());

    mesh::Mesh globalMesh;

    std::unordered_map<size_t, std::vector<size_t>> hashedpts;
    std::unordered_map<size_t, std::vector<size_t>> hashedtris;

    for (int idRef = 0; idRef < setSize; idRef++)
    {
        const fuseCut::Input& refInput = inputsets[idRef];

        mesh::Mesh meshReference;
        try
        {
            meshReference.load(refInput.subMeshPath);
        }
        catch (...)
        {
            ALICEVISION_LOG_ERROR("Can't read mesh " << refInput.subMeshPath);
            return EXIT_FAILURE;
        }

        std::unordered_map<size_t, size_t> transform;

        int count = 0;
        for (int indexPt = 0; indexPt < meshReference.pts.size(); indexPt++)
        {
            const Point3d& pt = meshReference.pts[indexPt];

            // Lookup if the point exist in the global mesh
            size_t hash = computeHash(pt);
            size_t indexFound = 0;
            bool found = false;
            auto it = hashedpts.find(hash);
            if (it != hashedpts.end())
            {
                for (const auto& idx : it->second)
                {
                    const Point3d& otherPt = globalMesh.pts[idx];
                    if (otherPt == pt)
                    {
                        found = true;
                        indexFound = idx;
                        break;
                    }
                }
            }

            if (found)
            {
                transform[indexPt] = indexFound;
                count++;
            }
            else
            {
                // If not found, it's a new point !
                size_t index = globalMesh.pts.size();
                transform[indexPt] = index;
                globalMesh.pts.push_back(pt);
                hashedpts[hash].push_back(index);
            }
        }

        // Copy the triangles
        for (size_t indexTriangle = 0; indexTriangle < meshReference.tris.size(); indexTriangle++)
        {
            const auto& tri = meshReference.tris[indexTriangle];

            int tv1 = transform[tri.v[0]];
            int tv2 = transform[tri.v[1]];
            int tv3 = transform[tri.v[2]];
            mesh::Mesh::triangle newTriangle(tv1, tv2, tv3);

            /*Check if the triangle exists in the global mesh*/
            size_t hash = computeHash(newTriangle);
            bool found = false;
            auto it = hashedtris.find(hash);
            if (it != hashedtris.end())
            {
                for (size_t idxTriCmp : it->second)
                {
                    const auto& triCmp = globalMesh.tris[idxTriCmp];

                    if (triCmp.v[0] == tv1 && triCmp.v[1] == tv2 && triCmp.v[2] == tv3)
                    {
                        found = true;
                        break;
                    }
                }
            }

            if (found)
            {
                continue;
            }

            hashedtris[hash].push_back(globalMesh.tris.size());
            globalMesh.tris.push_back(newTriangle);
        }
    }

    globalMesh.save(outputMeshFilename);

    return EXIT_SUCCESS;
}
