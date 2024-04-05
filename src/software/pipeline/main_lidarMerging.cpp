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
#include <fstream>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;
    

    std::string jsonFilename = "";
    std::string outputMeshFilename = "";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&jsonFilename)->required(),
         "json file.")
        ("output,o", po::value<std::string>(&outputMeshFilename)->required(),
         "Output file.");

    po::options_description optionalParams("Optional parameters");
    // optionalParams.add_options()
    //     ("boundingBox", po::value<fuseCut::BoundingBox>(&boundingBox),
    //      "Specifies a bounding box to reconstruct: position, rotation (Euler ZXY) and scale.")
    //     ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
    //      "Range image index start.")
    //     ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
    //      "Range size.");
    // clang-format on

    CmdLine cmdline("AliceVision lidarMeshing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    //omp_set_num_threads(std::min(32u, hwc.getMaxThreads()));

    std::ifstream inputfile(jsonFilename);
    if (!inputfile.is_open())
    {
        ALICEVISION_LOG_ERROR("Cannot open json input file");
        return EXIT_FAILURE;
    }
    
    
    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    fuseCut::InputSet inputsets(boost::json::value_to<fuseCut::InputSet>(jv));

    int setSize = static_cast<int>(inputsets.size());

    mesh::Mesh globalMesh;

    for (int id = 0; id < setSize; id++)
    {
        mesh::Mesh mesh;
        try
        {
            mesh.load(inputsets[id].subMeshPath);
        }
        catch (...)
        {
            ALICEVISION_LOG_ERROR("Can't read mesh " << inputsets[id].subMeshPath);
            return EXIT_FAILURE;
        }

        globalMesh.addMesh(mesh);
    }

    globalMesh.save(outputMeshFilename);
    
    return EXIT_SUCCESS;
}
