// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <boost/program_options.hpp>

#include <aliceVision/track/Track.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksMerger.cpp>

#include <string>
#include <sstream>
#include <random>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> tracksFilenames;
    std::string outFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputs,i", po::value<std::vector<std::string>>(&tracksFilenames)->multitoken(),
         "Path to sfmDatas to merge.")
        ("output,o", po::value<std::string>(&outFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmMerge");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (tracksFilenames.empty())
    {
        ALICEVISION_LOG_ERROR("At least one tracks input should be given.");
        return EXIT_FAILURE;
    }

    //viewId, featureId is a unique identifier for an observation
    using TuplePoint = std::tuple<feature::EImageDescriberType, IndexT, std::size_t>;

    track::TracksMerger merger;

    for (const auto & path : tracksFilenames)
    {
        track::TracksMap mapTracks;

        ALICEVISION_LOG_INFO("Loading " << path);
        if (!track::loadTracks(mapTracks, path))
        {
            continue;
        }
        
        ALICEVISION_LOG_INFO("File has " << mapTracks.size() << " tracks.");

        if (!merger.addTrackMap(mapTracks))
        {
            ALICEVISION_LOG_ERROR("addTrackMap failed, abort");
            return EXIT_FAILURE;
        }
    }
    
    const auto & resultTracks = merger.getOutputTracks();

    ALICEVISION_LOG_INFO("Saving to " << outFilename);
    ALICEVISION_LOG_INFO("File has " << resultTracks.size() << " tracks.");
    if (!track::saveTracks(resultTracks, outFilename))
    {
        ALICEVISION_LOG_ERROR("Failed to save file");
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}
