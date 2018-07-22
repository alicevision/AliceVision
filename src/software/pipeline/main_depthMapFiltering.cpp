// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/PreMatchCams.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string iniFilepath;
    std::string depthMapFolder;
    std::string outputFolder;
    int rangeStart = -1;
    int rangeSize = -1;

    int minNumOfConsistensCams = 3;
    int minNumOfConsistensCamsWithLowSimilarity = 4;
    int pixSizeBall = 0;
    int pixSizeBallWithLowSimilarity = 0;
    int nNearestCams = 10;

    po::options_description allParams("AliceVision depthMapFiltering\n"
                                      "Filter depth map to remove values that are not consistent with other depth maps");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).")
        ("depthMapFolder", po::value<std::string>(&depthMapFolder)->required(),
            "Input depth map folder.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
            "Output folder for filtered depth maps.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
            "Compute only a sub-range of images from index rangeStart to rangeStart+rangeSize.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
            "Compute only a sub-range of N images (N=rangeSize).")
        ("minNumOfConsistensCams", po::value<int>(&minNumOfConsistensCams)->default_value(minNumOfConsistensCams),
            "Minimal number of consistent cameras to consider the pixel.")
        ("minNumOfConsistensCamsWithLowSimilarity", po::value<int>(&minNumOfConsistensCamsWithLowSimilarity)->default_value(minNumOfConsistensCamsWithLowSimilarity),
            "Minimal number of consistent cameras to consider the pixel when the similarity is weak or ambiguous.")
        ("pixSizeBall", po::value<int>(&pixSizeBall)->default_value(pixSizeBall),
            "Filter ball size (in px).")
        ("pixSizeBallWithLowSimilarity", po::value<int>(&pixSizeBallWithLowSimilarity)->default_value(pixSizeBallWithLowSimilarity),
            "Filter ball size (in px) when the similarity is weak or ambiguous.")
        ("nNearestCams", po::value<int>(&nNearestCams)->default_value(nNearestCams),
            "Number of nearest cameras.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
      ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // .ini and files parsing
    mvsUtils::MultiViewParams mp(iniFilepath, depthMapFolder, outputFolder, true);
    mvsUtils::PreMatchCams pc(&mp);

    StaticVector<int> cams;
    cams.reserve(mp.ncams);

    if(rangeSize == -1)
    {
        for(int rc = 0; rc < mp.ncams; rc++) // process all cameras
            cams.push_back(rc);
    }
    else
    {
        if(rangeStart < 0)
        {
            ALICEVISION_LOG_ERROR("invalid subrange of cameras to process.");
            return EXIT_FAILURE;
        }
        for(int rc = rangeStart; rc < std::min(rangeStart + rangeSize, mp.ncams); ++rc)
            cams.push_back(rc);
        if(cams.empty())
        {
            ALICEVISION_LOG_INFO("No camera to process.");
            return EXIT_SUCCESS;
        }
    }

    ALICEVISION_LOG_INFO("Filter depth maps.");

    {
        fuseCut::Fuser fs(&mp, &pc);
        fs.filterGroups(cams, pixSizeBall, pixSizeBallWithLowSimilarity, nNearestCams);
        fs.filterDepthMaps(cams, minNumOfConsistensCams, minNumOfConsistensCamsWithLowSimilarity);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
