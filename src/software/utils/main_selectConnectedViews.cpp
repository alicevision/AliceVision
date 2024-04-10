// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <fstream>
#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

using MapTCamsPerView = std::map<IndexT, std::vector<IndexT>>;


void saveTCameras(std::ostream& stream, const MapTCamsPerView& tcamsPerView)
{
    if (tcamsPerView.empty())
    {
        return;
    }

    for (auto it = tcamsPerView.begin(); it != tcamsPerView.end(); ++it)
    {
        if(it->second.empty())
        {
            ALICEVISION_LOG_WARNING("The camera " << it->first << " is reconstructed but is not linked to any other camera.");
            continue;
        }
        // RC camera
        stream << it->first << " ";
        // List of TC cameras
        for(IndexT tcam: it->second)
        {
            stream << tcam << " ";
        }
        stream << "\n";
    }
}

bool saveTCameras(const std::string& filepath, const MapTCamsPerView& tcamsPerView)
{
    std::ofstream outStream(filepath);
    if (!outStream.is_open())
    {
        ALICEVISION_LOG_WARNING("saveTCameras: Impossible to open the output specified file: \"" << filepath << "\".");
        return false;
    }

    saveTCameras(outStream, tcamsPerView);

    return !outStream.bad();
}

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string sfmDataFilepath;
    std::string outputFilepath;

    // min / max view angle
    float minViewAngle = 2.0f;
    float maxViewAngle = 70.0f;

    int maxTCams = 10;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputFilepath)->required(),
         "Output file of corresponding image IDs.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("minViewAngle", po::value<float>(&minViewAngle)->default_value(minViewAngle),
         "Minimum angle between two views (select the neighbouring cameras, select depth planes from epipolar segment point).")
        ("maxViewAngle", po::value<float>(&maxViewAngle)->default_value(maxViewAngle),
         "Maximum angle between two views (select the neighbouring cameras, select depth planes from epipolar segment point).")
        ("maxTCams", po::value<int>(&maxTCams)->default_value(maxTCams),
         "Maximum number of neighbour cameras per image.")
        ;
    // clang-format on

    CmdLine cmdline("AliceVision Select Connected Views using landmarks.");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataFilepath.empty())
    {
        ALICEVISION_LOG_INFO("Load SfMData.");
        if (!sfmDataIO::load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData::ALL_DENSE))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilepath << "' cannot be read.");
            return EXIT_FAILURE;
        }
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, "");
    MapTCamsPerView tCamerasPerViews;

    for(int rc = 0; rc < mp.ncams; ++rc)
    {
        // get R camera Tcs list
        const std::vector<int> tCams = mp.findNearestCamsFromLandmarks(rc, maxTCams).getDataWritable();
        std::vector<IndexT> tCamIDs(tCams.size());
        for(std::size_t i = 0; i < tCams.size(); ++i)
        {
            const int camIndex = tCams[i]; // camera index in MVS
            tCamIDs[i] = mp.getViewId(camIndex); // convert to view ID
        }
        tCamerasPerViews[mp.getViewId(rc)] = tCamIDs;
    }

    saveTCameras(outputFilepath, tCamerasPerViews);

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
