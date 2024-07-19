// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/track/trackIO.hpp>

#include <boost/program_options.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string tracksFilename;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    int maxNbMatches = 0;
    int minNbMatches = 0;
    int minInputTrackLength = 2;
    bool filterTrackForks = true;
    bool useOnlyMatchesFromInputFolder = false;

    // user optional parameters
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&tracksFilename)->required(),
         "Path to the tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("maxNumberOfMatches", po::value<int>(&maxNbMatches)->default_value(maxNbMatches),
         "Maximum number of matches per image pair (and per feature type). "
         "This can be useful to have a quick reconstruction overview. 0 means no limit.")
        ("minNumberOfMatches", po::value<int>(&minNbMatches)->default_value(minNbMatches),
         "Minimum number of matches per image pair (and per feature type). "
         "This can be useful to have a meaningful reconstruction with accurate keypoints. 0 means no limit.")
        ("minInputTrackLength", po::value<int>(&minInputTrackLength)->default_value(minInputTrackLength),
         "Minimum track length in input of SfM.")
        ("useOnlyMatchesFromInputFolder", po::value<bool>(&useOnlyMatchesFromInputFolder)->default_value(useOnlyMatchesFromInputFolder),
         "Use only matches from the input matchesFolder parameter.\n"
         "Matches folders previously added to the SfMData file will be ignored.")
        ("filterTrackForks", po::value<bool>(&filterTrackForks)->default_value(filterTrackForks),
         "Enable/Disable the track forks removal. "
         "A track contains a fork when incoherent matches leads to multiple features in the same image for a single track.");
    // clang-format on

    CmdLine cmdline("AliceVision tracksBuilding");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // get imageDescriber type
    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // features reading
    feature::FeaturesPerView featuresPerView;
    ALICEVISION_LOG_INFO("Load features");
    if (!sfm::loadFeaturesPerView(featuresPerView, sfmData, featuresFolders, describerTypes))
    {
        ALICEVISION_LOG_ERROR("Invalid features.");
        return EXIT_FAILURE;
    }

    // matches reading
    matching::PairwiseMatches pairwiseMatches;
    ALICEVISION_LOG_INFO("Load features matches");
    if (!sfm::loadPairwiseMatches(
          pairwiseMatches, sfmData, matchesFolders, describerTypes, maxNbMatches, minNbMatches, useOnlyMatchesFromInputFolder))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches.");
        return EXIT_FAILURE;
    }

    // Create tracks
    track::TracksBuilder tracksBuilder;
    ALICEVISION_LOG_INFO("Track building");
    tracksBuilder.build(pairwiseMatches);

    ALICEVISION_LOG_INFO("Track filtering");
    tracksBuilder.filter(filterTrackForks, minInputTrackLength);

    ALICEVISION_LOG_INFO("Track export to structure");
    track::TracksMap mapTracks;
    tracksBuilder.exportToSTL(mapTracks, &featuresPerView);

    // write the json file with the tree
    ALICEVISION_LOG_INFO("Export to file");
    std::stringstream ss;

    boost::json::value jv = boost::json::value_from(mapTracks);
    std::ofstream of(tracksFilename);
    of << boost::json::serialize(jv);
    of.close();

    return EXIT_SUCCESS;
}
