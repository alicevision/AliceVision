// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <cmath>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
using namespace aliceVision;

std::shared_ptr<camera::IntrinsicBase> findCalibratedIntrinsic(
    const std::shared_ptr<camera::IntrinsicBase>& intrinsic,
    const sfmData::Intrinsics& calibratedIntrinsics)
{
    if (calibratedIntrinsics.size() == 1)
    {
        return calibratedIntrinsics.begin()->second;
    }

    return nullptr;
}

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

    // Special case to handle:
    // If calibrated SfMData filename is an empty string
    // simply copy the input SfMData
    if (sfmDataCalibratedFilename.empty())
    {
        // Save sfmData to disk
        if (!sfmDataIO::Save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }

    // Load calibrated scene
    sfmData::SfMData sfmDataCalibrated;
    if (!sfmDataIO::Load(sfmDataCalibrated, sfmDataCalibratedFilename, sfmDataIO::ESfMData::INTRINSICS))
    {
        ALICEVISION_LOG_ERROR("The calibrated SfMData file '" << sfmDataCalibratedFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Overwrite input SfMData intrinsics with calibrated one
    const auto& calibratedIntrinsics = sfmDataCalibrated.getIntrinsics();
    auto& intrinsics = sfmData.getIntrinsics();
    for (const auto& [intrinsicId, intrinsic] : intrinsics)
    {
        ALICEVISION_LOG_INFO("Processing intrinsic " << intrinsicId);

        // Find corresponding calibrated intrinsic
        const auto calibratedIntrinsic =
            std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(
                findCalibratedIntrinsic(intrinsic, calibratedIntrinsics));
        
        if (!calibratedIntrinsic)
        {
            ALICEVISION_LOG_ERROR("Unable to find a corresponding calibrated intrinsic");
            return EXIT_FAILURE;
        }
        
        const bool isIntrinsicCalibrated = calibratedIntrinsic->getInitializationMode() == camera::EInitMode::CALIBRATED;
        const bool isDistortionCalibrated = calibratedIntrinsic->getDistortionInitializationMode() == camera::EInitMode::CALIBRATED;

        if (!isIntrinsicCalibrated && !isDistortionCalibrated) continue;

        // Aspect ratio of input intrinsic
        const unsigned int width = intrinsic->w();
        const unsigned int height = intrinsic->h();
        const double aspect = static_cast<double>(height) / static_cast<double>(width);

        // Aspect ratio of calibrated intrinsic
        const unsigned int calibrationWidth = calibratedIntrinsic->w();
        const unsigned int calibrationHeight = calibratedIntrinsic->h();
        const double calibrationAspect = static_cast<double>(calibrationHeight) / static_cast<double>(calibrationWidth);

        // Check that aspect ratios are approximately equal
        if (std::abs(aspect - calibrationAspect) > 1e-2)
        {
            ALICEVISION_LOG_ERROR("Intrinsic from input SfMData and calibrated SfMData are incompatible.");
            return EXIT_FAILURE;
        }

        // Copy original intrinsic
        auto newIntrinsic =
            std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(
                camera::createIntrinsic(intrinsic->getType()));
        newIntrinsic->assign(*intrinsic);

        if (isIntrinsicCalibrated)
        {
            // Use calibrated focal length and offset
            const double rx = static_cast<double>(width) / static_cast<double>(calibrationWidth);
            const double ry = static_cast<double>(height) / static_cast<double>(calibrationHeight);
            const double fx = calibratedIntrinsic->getScale().x() * rx;
            const double fy = calibratedIntrinsic->getScale().y() * ry;
            const double ox = calibratedIntrinsic->getOffset().x() * rx;
            const double oy = calibratedIntrinsic->getOffset().y() * ry;
            newIntrinsic->setScale({fx, fy});
            newIntrinsic->setOffset({ox, oy});
            newIntrinsic->setInitializationMode(camera::EInitMode::CALIBRATED);
        }

        if (isDistortionCalibrated)
        {
            // Use calibrated distortion
            newIntrinsic->setDistortionObject(nullptr);
            auto calibratedUndistortion = calibratedIntrinsic->getUndistortion();
            auto undistortion = camera::createUndistortion(
                calibratedUndistortion->getType());
            undistortion->setSize(width, height);
            undistortion->setParameters(calibratedUndistortion->getParameters());
            newIntrinsic->setUndistortionObject(undistortion);
            newIntrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
        }

        // Overwrite intrinsic with new one
        intrinsics.emplace(intrinsicId, newIntrinsic);
    }

    // Save sfmData to disk
    if (!sfmDataIO::Save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
