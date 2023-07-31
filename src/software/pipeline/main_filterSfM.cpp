// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <set>
#include <iterator>
#include <iomanip>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool filterSfMData(SfMData& sfmData, int maxNbObservationsPerLandmark)
{
    #pragma omp parallel for
    for(auto i = 0; i < sfmData.getLandmarks().size(); i++)
    {
        auto landmarkPair = sfmData.getLandmarks().begin();
        std::advance(landmarkPair, i);
        sfmData::Landmark& landmark = landmarkPair->second;

        // check number of observations
        if(landmark.observations.size() <= maxNbObservationsPerLandmark)
            continue;

        // // check angle between observations
        // if(!checkLandmarkMinObservationAngle(sfmData, landmark, minObservationAngle))

        // compute observation scores

        std::vector<std::pair<double, IndexT> > observationScores;
        observationScores.reserve(landmark.observations.size());

        for(const auto& observationPair : landmark.observations)
        {
            const IndexT viewId = observationPair.first;
            const sfmData::View& view = *(sfmData.getViews().at(viewId));
            const geometry::Pose3 pose = sfmData.getPose(view).getTransform();

            observationScores.push_back(std::pair<double, IndexT>(
                (pose.center() - landmark.X).squaredNorm(),
                viewId
            ));
        }

        // sort observations by ascending score order
        std::stable_sort(observationScores.begin(), observationScores.end());
        // take only best observations
        observationScores.resize(maxNbObservationsPerLandmark);

        // replace the observations
        Observations filteredObservations;
        for(auto observationScorePair : observationScores)
        {
            filteredObservations[observationScorePair.second] = landmark.observations[observationScorePair.second];
        }
        landmark.observations = filteredObservations;
    }
    return true;
}

int aliceVision_main(int argc, char *argv[])
{
    // command-line parameters

    // std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputSfmFilename;
    std::string outputSfmFilename;
    int maxNbObservationsPerLandmark = 5;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&outputSfmFilename)->required(),
         "Output SfMData file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxNbObservationsPerLandmark", po::value<int>(&maxNbObservationsPerLandmark)->default_value(maxNbObservationsPerLandmark),
         "Maximum number of allowed observations per landmark.");

    CmdLine cmdline("AliceVision SfM filtering.");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Create output dir
    {
        auto outDir = fs::path(outputSfmFilename).parent_path().string();
        if (!fs::exists(outDir))
            fs::create_directory(outDir);
    }

    // Read the input SfM scene
    SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, inputSfmFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // filter SfM data
    if(filterSfMData(sfmData, maxNbObservationsPerLandmark))
    {
        sfmDataIO::Save(sfmData, outputSfmFilename, sfmDataIO::ESfMData::ALL);
        return EXIT_SUCCESS;
    }

    return EXIT_FAILURE;
}
