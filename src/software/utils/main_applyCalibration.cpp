// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>

#include <boost/program_options.hpp>

#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
using namespace aliceVision;


int aliceVision_main(int argc, char **argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::string sfmDataCalibratedFilename;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
             "SfMData scene to apply calibration to.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
            "Output SfMData scene.")
        ("calibration,c", po::value<std::string>(&sfmDataCalibratedFilename)->required(),
            "Calibrated SfMData scene.");
    
    CmdLine cmdline("AliceVision applyCalibration");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Load calibrated scene
    sfmData::SfMData sfmDataCalibrated;
    if (!sfmDataIO::Load(sfmDataCalibrated, sfmDataCalibratedFilename, sfmDataIO::ESfMData::INTRINSICS))
    {
        ALICEVISION_LOG_ERROR("The calibrated SfMData file '" << sfmDataCalibratedFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Check that there is exactly one calibrated intrinsic
    const auto& calibratedIntrinsics = sfmDataCalibrated.getIntrinsics();
    if (calibratedIntrinsics.size() == 0)
    {
        ALICEVISION_LOG_ERROR("The calibrated SfMData does not have any intrinsics");
        return EXIT_FAILURE;
    }
    if (calibratedIntrinsics.size() > 1)
    {
        ALICEVISION_LOG_ERROR("The calibrated SfMData has more than 1 intrinsics");
        return EXIT_FAILURE;
    }

    const auto& calibratedIntrinsic = calibratedIntrinsics.begin()->second;
    if (calibratedIntrinsic->getDistortionInitializationMode() != camera::EInitMode::CALIBRATED)
    {
        ALICEVISION_LOG_ERROR("Intrinsic from calibrated SfMData is not calibrated");
        return EXIT_FAILURE;
    }

    // Overwrite input SfMData intrinsics with calibrated one
    auto& intrinsics = sfmData.getIntrinsics();
    for (const auto& [intrinsicId, _] : intrinsics)
    {
        intrinsics[intrinsicId] = calibratedIntrinsic;
    }

    // Save sfmData to disk
    if (!sfmDataIO::Save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
