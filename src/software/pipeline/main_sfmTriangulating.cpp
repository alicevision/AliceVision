// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>
#include <aliceVision/sfm/pipeline/expanding/SfmTriangulation.hpp>

#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilename;
    std::string sfmDataOutputFilename;
    std::string tracksFilename;

    double minAngleForTriangulation = 1.0;
    size_t minNbObservationsForTriangulation = 0;
    double maxTriangulationError = 8.0;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file, must contain the camera calibration.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "Path to the output SfMData file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxTriangulationError", po::value<double>(&maxTriangulationError)->default_value(maxTriangulationError), "Maximum reprojection error in the triangulation process (in pixels).")
        ("minAngleForTriangulation", po::value<double>(&minAngleForTriangulation)->default_value(minAngleForTriangulation), "Minimum angle for triangulation in degrees.")
        ("minNumberOfObservationsForTriangulation", po::value<std::size_t>(&minNbObservationsForTriangulation)->default_value(minNbObservationsForTriangulation), "Minimum number of observations to triangulate a point.");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Triangulation");
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
    if(!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getValidViews()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    std::set<IndexT> evaluatedTracks;
    std::map<IndexT, sfmData::Landmark> outputLandmarks;
    std::mt19937 randomNumberGenerator;


    // Effectively triangulate tracks
    sfm::SfmTriangulation sfmTriangulation(minNbObservationsForTriangulation, maxTriangulationError);
    if (!sfmTriangulation.process(sfmData, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), 
                            randomNumberGenerator, sfmData.getValidViews(), 
                            evaluatedTracks, outputLandmarks, false))
    {
        ALICEVISION_LOG_ERROR("Triangulation failed.");
        return EXIT_FAILURE;
    }

    auto & landmarks = sfmData.getLandmarks();
    landmarks.clear();

    for (const auto & [landmarkId, outputLandmark] : outputLandmarks)
    {
        if (outputLandmark.getObservations().size() < minNbObservationsForTriangulation)
        {
            continue;
        }

        if (!sfm::SfmTriangulation::checkChierality(sfmData, outputLandmark))
        {
            continue;
        }

        double maxAngle = sfm::SfmTriangulation::getMaximalAngle(sfmData, outputLandmark);
        if (maxAngle < minAngleForTriangulation)
        {
            continue;
        }

        landmarks[landmarkId] = outputLandmark;
    }

    // Save output
    if (!sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << sfmDataOutputFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
