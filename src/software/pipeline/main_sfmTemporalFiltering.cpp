// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/main.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/hardwareContext.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/sfm/utils/poseFilter.hpp>

#include <boost/program_options.hpp>

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
    std::string sfmDataOutputFilename;
    std::string outputSfMViewsAndPoses;

    bool filterPosition = true;
    bool filterRotation = true;
    int iterationCount = 100;
    int scaleFactor = 3;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("output,o", po::value<std::string>(&sfmDataOutputFilename)->required(), "SfMData output file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("filterPosition", po::value<bool>(&filterPosition)->default_value(filterPosition), "Whether to filter camera positions.")
        ("filterRotation", po::value<bool>(&filterRotation)->default_value(filterRotation), "Whether to filter camera orientations.")
        ("scaleFactor", po::value<int>(&scaleFactor)->default_value(scaleFactor), "Scale factor to increase the filter range.")
        ("iterationCount", po::value<int>(&iterationCount)->default_value(iterationCount), "Number of filter iterations.")
        ("outputViewsAndPoses", po::value<std::string>(&outputSfMViewsAndPoses)->default_value(outputSfMViewsAndPoses), "Path to the output SfMData file (with only views and poses).");
    // clang-format on

    CmdLine cmdline("AliceVision SfM Temporal Filtering");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if(!cmdline.execute(argc, argv))
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

    sfm::poseFilter::uptr poseFilter = std::make_unique<sfm::poseFilter>();

    if (!poseFilter->process(sfmData, filterPosition, filterRotation, scaleFactor, iterationCount))
    {
        ALICEVISION_LOG_INFO("Error processing sfmData");
        return EXIT_FAILURE;
    }

    sfmDataIO::save(sfmData, sfmDataOutputFilename, sfmDataIO::ESfMData::ALL);

    if (!outputSfMViewsAndPoses.empty())
    {
        sfmDataIO::save(sfmData, outputSfMViewsAndPoses,
            sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)
        );
    }

    return EXIT_SUCCESS;
}