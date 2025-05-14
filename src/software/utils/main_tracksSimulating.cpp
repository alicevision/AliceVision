// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/track/Track.hpp>
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
    double sigmaNoise = 0.0;
    double outlierRatio = 0.0;
    double outlierEpipolarRatio = 0.2;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&tracksFilename)->required(),
         "Path to the tracks file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("sigmaNoise", po::value<double>(&sigmaNoise)->default_value(sigmaNoise))
        ("outlierRatio", po::value<double>(&outlierRatio)->default_value(outlierRatio))
        ("outlierEpipolarRatio", po::value<double>(&outlierEpipolarRatio)->default_value(outlierEpipolarRatio));
        
    // clang-format on

    CmdLine cmdline("AliceVision tracksSimulating");

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

    std::mt19937 generator;
    std::uniform_real_distribution<double> rand(0, 1);
    std::uniform_real_distribution<double> randDepth(0.1, 100.0);
    std::normal_distribution<double> randNoise(0.0, sigmaNoise);
    
    track::TracksMap mapTracks;

    for (const auto & [landmarkId, landmark] : sfmData.getLandmarks())
    {
        const Vec3 point = landmark.X;

        track::Track track;
        track.descType = landmark.descType;

        for (const auto & [viewId, observation] : landmark.getObservations())
        {
            const auto & view = sfmData.getView(viewId);
            const auto & intrinsic = sfmData.getIntrinsic(view.getIntrinsicId());
            const auto & pose = sfmData.getAbsolutePose(view.getPoseId());

            Vec2 obs;

            if (rand(generator) < outlierRatio)
            {
                //This is an outlier
                if (rand(generator) < outlierEpipolarRatio)
                {
                    //Outlier but on the epipolar line
                    Vec3 fakePoint = point * randDepth(generator) / point(2);
                    obs = intrinsic.transformProject(pose.getTransform(), fakePoint.homogeneous(), true);
                }
                else 
                {
                    std::uniform_real_distribution<double> randWidth(0.0, intrinsic.w());
                    std::uniform_real_distribution<double> randHeight(0.0, intrinsic.h());

                    obs.x() = randWidth(generator);
                    obs.y() = randHeight(generator);
                }
            }
            else 
            {
                obs = intrinsic.transformProject(pose.getTransform(), point.homogeneous(), true);
            }

            obs.x() += randNoise(generator);
            obs.y() += randNoise(generator);

            track::TrackItem item;
            item.coords = obs;
            item.featureId = observation.getFeatureId();
            item.scale = observation.getScale();
            
            track.featPerView[viewId] = item;
        }

        mapTracks[landmarkId] = track;
    }

     // write the json file
    ALICEVISION_LOG_INFO("Export to file");
    boost::json::value jv = boost::json::value_from(mapTracks);
    std::ofstream of(tracksFilename);
    of << boost::json::serialize(jv);
    of.close();

    return EXIT_SUCCESS;
}
