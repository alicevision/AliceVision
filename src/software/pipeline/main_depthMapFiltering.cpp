// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/depthMap/computeOnMultiGPUs.hpp>
#include <aliceVision/depthMap/NormalMapEstimator.hpp>

#include <boost/program_options.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;

    std::string sfmDataFilename;
    std::string depthMapsFolder;
    std::string outputFolder;

    // program range
    int rangeStart = -1;
    int rangeSize = -1;

    // min / max view angle
    float minViewAngle = 2.0f;
    float maxViewAngle = 70.0f;

    int minNumOfConsistentCams = 3;
    int minNumOfConsistentCamsWithLowSimilarity = 4;
    float pixToleranceFactor = 2.0f;
    int pixSizeBall = 0;
    int pixSizeBallWithLowSimilarity = 0;
    int nNearestCams = 10;
    bool computeNormalMaps = false;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("depthMapsFolder", po::value<std::string>(&depthMapsFolder)->required(),
         "Input depth map folder.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Output folder for filtered depth maps.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Compute only a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Compute only a sub-range of N images (N=rangeSize).")
        ("minViewAngle", po::value<float>(&minViewAngle)->default_value(minViewAngle),
         "Minimum angle between two views.")
        ("maxViewAngle", po::value<float>(&maxViewAngle)->default_value(maxViewAngle),
         "Maximum angle between two views.")
        ("minNumOfConsistentCams", po::value<int>(&minNumOfConsistentCams)->default_value(minNumOfConsistentCams),
         "Minimal number of consistent cameras to consider the pixel.")
        ("minNumOfConsistentCamsWithLowSimilarity", po::value<int>(&minNumOfConsistentCamsWithLowSimilarity)->default_value(minNumOfConsistentCamsWithLowSimilarity),
         "Minimal number of consistent cameras to consider the pixel when the similarity is weak or ambiguous.")
        ("pixToleranceFactor", po::value<float>(&pixToleranceFactor)->default_value(pixToleranceFactor),
         "Filtering tolerance size factor (in px).")
        ("pixSizeBall", po::value<int>(&pixSizeBall)->default_value(pixSizeBall),
         "Filter ball size (in px).")
        ("pixSizeBallWithLowSimilarity", po::value<int>(&pixSizeBallWithLowSimilarity)->default_value(pixSizeBallWithLowSimilarity),
         "Filter ball size (in px) when the similarity is weak or ambiguous.")
        ("nNearestCams", po::value<int>(&nNearestCams)->default_value(nNearestCams),
         "Number of nearest cameras.")
        ("computeNormalMaps", po::value<bool>(&computeNormalMaps)->default_value(computeNormalMaps),
         "Compute normal maps per depth map.");
    // clang-format on

    CmdLine cmdline("This program filters depth maps to remove values that are not consistent with other depth maps.\n"
                    "AliceVision depthMapFiltering");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // read the input SfM scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // initialization
    mvsUtils::MultiViewParams mp(sfmData, "", depthMapsFolder, outputFolder, mvsUtils::EFileType::depthMap);

    mp.setMinViewAngle(minViewAngle);
    mp.setMaxViewAngle(maxViewAngle);

    std::vector<int> cams;
    cams.reserve(mp.ncams);

    if (rangeSize == -1)
    {
        for (int rc = 0; rc < mp.ncams; rc++)  // process all cameras
            cams.push_back(rc);
    }
    else
    {
        if (rangeStart < 0)
        {
            ALICEVISION_LOG_ERROR("invalid subrange of cameras to process.");
            return EXIT_FAILURE;
        }
        for (int rc = rangeStart; rc < std::min(rangeStart + rangeSize, mp.ncams); ++rc)
            cams.push_back(rc);
        if (cams.empty())
        {
            ALICEVISION_LOG_INFO("No camera to process.");
            return EXIT_SUCCESS;
        }
    }

    ALICEVISION_LOG_INFO("Filter depth maps.");

    {
        fuseCut::Fuser fs(mp);
        fs.filterGroups(cams, pixToleranceFactor, pixSizeBall, pixSizeBallWithLowSimilarity, nNearestCams);
        fs.filterDepthMaps(cams, minNumOfConsistentCams, minNumOfConsistentCamsWithLowSimilarity);
    }

    if (computeNormalMaps)
    {
        int nbGPUs = 0;

        // initialize depth map estimator
        depthMap::NormalMapEstimator normalMapEstimator(mp);

        // estimate normal maps
        depthMap::computeOnMultiGPUs(cams, normalMapEstimator, nbGPUs);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
