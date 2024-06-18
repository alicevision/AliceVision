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

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::string sfmDataCalibratedFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData scene to apply calibration to.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.")
        ("calibration,c", po::value<std::string>(&sfmDataCalibratedFilename)->required(),
         "Calibrated SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision applyCalibration");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
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
        if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }

    // Load calibrated scene
    sfmData::SfMData sfmDataCalibrated;
    if (!sfmDataIO::load(sfmDataCalibrated, sfmDataCalibratedFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The calibrated SfMData file '" << sfmDataCalibratedFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Detect calibration setup
    const auto& calibratedIntrinsics = sfmDataCalibrated.getIntrinsics();

    const bool isMonoCam = calibratedIntrinsics.size() == 1;
    const bool isMultiCam = calibratedIntrinsics.size() > 1 && sfmDataCalibrated.getRigs().size() == 1;

    if (!isMonoCam && !isMultiCam)
    {
        ALICEVISION_LOG_ERROR("Unknown calibration setup");
        return EXIT_FAILURE;
    }

    // Apply rig calibration
    if (isMultiCam)
    {
        if (sfmData.getRigs().size() != 1)
        {
            ALICEVISION_LOG_ERROR("There must be exactly 1 rig to apply rig calibration");
            return EXIT_FAILURE;
        }

        // Retrieve calibrated sub-poses
        const auto& calibratedRig = sfmDataCalibrated.getRigs().begin()->second;
        const auto& calibratedSubPoses = calibratedRig.getSubPoses();

        // Retrieve input sub-poses
        auto& rig = sfmData.getRigs().begin()->second;
        auto& subPoses = rig.getSubPoses();

        if (subPoses.size() != calibratedSubPoses.size())
        {
            ALICEVISION_LOG_ERROR("Incoherent number of sub-poses");
            return EXIT_FAILURE;
        }

        // Copy calibrated sub-poses
        for (std::size_t idx = 0; idx < subPoses.size(); ++idx)
        {
            if (calibratedSubPoses[idx].status != sfmData::ERigSubPoseStatus::CONSTANT)
                continue;

            subPoses[idx] = calibratedSubPoses[idx];
            subPoses[idx].status = sfmData::ERigSubPoseStatus::ESTIMATED;
        }

        // Turn off independent pose flag on views
        for (auto& [viewId, view] : sfmData.getViews())
        {
            if (view->isPartOfRig())
            {
                view->setIndependantPose(false);
            }
        }
    }

    // Apply intrinsic and distortion calibration
    auto& intrinsics = sfmData.getIntrinsics();
    for (const auto& [intrinsicId, intrinsic] : intrinsics)
    {
        ALICEVISION_LOG_INFO("Processing intrinsic " << intrinsicId);

        // Find corresponding calibrated intrinsic depending on calibration setup
        std::shared_ptr<camera::IntrinsicScaleOffsetDisto> calibratedIntrinsic = nullptr;
        if (isMonoCam)
        {
            calibratedIntrinsic = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(calibratedIntrinsics.begin()->second);
        }
        else if (isMultiCam)
        {
            IndexT subPoseId = UndefinedIndexT;
            for (const auto& [viewId, view] : sfmData.getViews())
            {
                if (view->getIntrinsicId() != intrinsicId)
                    continue;

                subPoseId = view->getSubPoseId();
                break;
            }

            bool found = false;
            for (const auto& [calibIntrId, calibIntr] : calibratedIntrinsics)
            {
                if (found)
                    break;

                for (const auto& [viewId, view] : sfmDataCalibrated.getViews())
                {
                    if (view->getIntrinsicId() != calibIntrId)
                        continue;

                    if (view->getSubPoseId() != subPoseId)
                        continue;

                    calibratedIntrinsic = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(calibIntr);
                    found = true;
                    break;
                }
            }
        }

        if (!calibratedIntrinsic)
        {
            ALICEVISION_LOG_ERROR("Unable to find a corresponding calibrated intrinsic");
            return EXIT_FAILURE;
        }

        const bool isIntrinsicCalibrated = calibratedIntrinsic->getInitializationMode() == camera::EInitMode::CALIBRATED;
        const bool isDistortionCalibrated = calibratedIntrinsic->getDistortionInitializationMode() == camera::EInitMode::CALIBRATED;

        if (!isIntrinsicCalibrated && !isDistortionCalibrated)
            continue;

        // Aspect ratio of input intrinsic
        const unsigned int width = intrinsic->w();
        const unsigned int height = intrinsic->h();
        const double aspect = static_cast<double>(height) / static_cast<double>(width);

        // Aspect ratio of calibrated intrinsic
        const unsigned int calibrationWidth = calibratedIntrinsic->w();
        const unsigned int calibrationHeight = calibratedIntrinsic->h();
        const double calibrationAspect = static_cast<double>(calibrationHeight) / static_cast<double>(calibrationWidth);

        if(width != calibrationWidth || height != calibrationHeight)
        {
            ALICEVISION_LOG_WARNING(
                    "The image size used for the calibration is different from the size of the sequence.\n" <<
                    "Calibration size: " << calibrationWidth << "x" << calibrationHeight << "\n" <<
                    "Sequence size: " << width << "x" << height);
            ALICEVISION_LOG_WARNING(
                    "Calibration image ratio: " << calibrationAspect << "\n" <<
                    "Sequence image ratio: " << aspect);
        }

        // If aspect ratios are not approximately equal
        if (std::abs(aspect - calibrationAspect) > 1e-2)
        {
            ALICEVISION_LOG_WARNING("Image aspect ratios are not approximately equal, so we need more checks.");

            const bool distortionOnly = (isDistortionCalibrated && !isIntrinsicCalibrated);
            const bool smaller = (width <= calibrationWidth && height <= calibrationHeight);

            ALICEVISION_LOG_WARNING("Distorsion is calibrated: " << isDistortionCalibrated);
            ALICEVISION_LOG_WARNING("Intrinsics are calibrated: " << isIntrinsicCalibrated);
            ALICEVISION_LOG_WARNING("Size is smaller than calibration: " << smaller);
            
            if (distortionOnly == false || smaller == false)
            {
                ALICEVISION_LOG_ERROR("Intrinsic from input SfMData and calibrated SfMData are incompatible.");
                return EXIT_FAILURE;
            }
        }

        // Copy original intrinsic
        std::shared_ptr<camera::IntrinsicScaleOffsetDisto> newIntrinsic(
            dynamic_cast<camera::IntrinsicScaleOffsetDisto*>(
                intrinsic->clone()
            )
        );

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
            newIntrinsic->setInitializationMode(camera::EInitMode::ESTIMATED);
        }

        if (isDistortionCalibrated)
        {
            // Use calibrated distortion
            newIntrinsic->setDistortionObject(nullptr);
            auto calibratedUndistortion = calibratedIntrinsic->getUndistortion();
            auto undistortion = camera::createUndistortion(calibratedUndistortion->getType());
            undistortion->setSize(width, height);
            undistortion->setPixelAspectRatio(calibratedUndistortion->getPixelAspectRatio());
            undistortion->setParameters(calibratedUndistortion->getParameters());


            newIntrinsic->setUndistortionObject(undistortion);
            newIntrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
        }

        // Overwrite intrinsic with new one
        intrinsics.at(intrinsicId) = newIntrinsic;
    }

    // Save sfmData to disk
    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
