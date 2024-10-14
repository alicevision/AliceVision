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
#include <aliceVision/dataio/json.hpp>

#include <boost/program_options.hpp>

#include <fstream>
#include <string>
#include <cmath>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
using namespace aliceVision;

bool applySfmData(sfmData::SfMData & sfmData, const sfmData::SfMData & sfmDataCalibrated)
{
    // Detect calibration setup
    const auto& calibratedIntrinsics = sfmDataCalibrated.getIntrinsics();

    const bool isMonoCam = calibratedIntrinsics.size() == 1;
    const bool isMultiCam = calibratedIntrinsics.size() > 1 && sfmDataCalibrated.getRigs().size() == 1;

    if (!isMonoCam && !isMultiCam)
    {
        ALICEVISION_LOG_ERROR("Unknown calibration setup");
        return false;
    }

    // Apply rig calibration
    if (isMultiCam)
    {
        if (sfmData.getRigs().size() != 1)
        {
            ALICEVISION_LOG_ERROR("There must be exactly 1 rig to apply rig calibration");
            return false;
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
            return false;
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
        {
            continue;
        }

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
                return false;
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
            undistortion->setDesqueezed(calibratedUndistortion->isDesqueezed());
            undistortion->setParameters(calibratedUndistortion->getParameters());

            newIntrinsic->setUndistortionObject(undistortion);
            newIntrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
        }

        // Overwrite intrinsic with new one
        intrinsics.at(intrinsicId) = newIntrinsic;
    }

    return true;
}

bool applyJson(sfmData::SfMData & sfmData, boost::json::value & input)
{
    if (input.kind() != boost::json::kind::object)
    {
        return false;
    }

    if (sfmData.getIntrinsics().size() != 1)
    {
        ALICEVISION_LOG_ERROR("Number of intrinsics is not one");
        return false;
    }

    auto intrinsic = std::dynamic_pointer_cast<camera::IntrinsicScaleOffsetDisto>(sfmData.getIntrinsics().begin()->second);
    if (!camera::isPinhole(intrinsic->getType()))
    {
        ALICEVISION_LOG_ERROR("Original intrinsic is not a pinhole");
        return false;
    }

    auto const& obj = input.get_object();

    const std::string & model = boost::json::value_to<std::string>(obj.at("distortionModel"));
    
    const double & focalLength = boost::json::value_to<double>(obj.at("focalLength"));
    const double & focusDistance = boost::json::value_to<double>(obj.at("focusDistance"));
    const double & filmbackWidth = boost::json::value_to<double>(obj.at("filmbackWidth"));
    const double & filmbackHeight = boost::json::value_to<double>(obj.at("filmbackHeight"));
    const double & filmbackOffsetX = boost::json::value_to<double>(obj.at("filmbackOffsetX"));
    const double & filmbackOffsetY = boost::json::value_to<double>(obj.at("filmbackOffsetY"));
    const double & pixelAspect = boost::json::value_to<double>(obj.at("pixelAspect"));

    if (std::abs(filmbackOffsetX) > 1e-12)
    {
        ALICEVISION_LOG_ERROR("Unsupported value for filmbackOffsetX");
        return false;
    }

    if (std::abs(filmbackOffsetY) > 1e-12)
    {
        ALICEVISION_LOG_ERROR("Unsupported value for filmbackOffsetY");
        return false;
    }
    
    double w = intrinsic->w();
    double h = intrinsic->h();
    double nw = w * pixelAspect;
    double ratio = w / h;
    double ratioDesqueezed = nw / h;
    double filmratio = filmbackWidth / filmbackHeight;

    bool hasSpecialPixelAspect = (std::abs(pixelAspect - 1.0) > 1e-4);

    //Compare image size ratio and filmback size ratio
    bool isDesqueezed = false;
    if ((std::abs(ratio - filmratio) < 1e-2) && hasSpecialPixelAspect)
    {
        ALICEVISION_LOG_INFO("Input image look desqueezed");
        isDesqueezed = true;
    }
    else if (std::abs(ratioDesqueezed - filmratio) > 1e-2)
    {
        ALICEVISION_LOG_ERROR("Incompatible image ratios");
        return false;
    }

    intrinsic->setSensorWidth((isDesqueezed)?filmbackWidth:filmbackWidth/pixelAspect);
    intrinsic->setSensorHeight(filmbackHeight);
    intrinsic->setDistortionObject(nullptr);
    intrinsic->setFocalLength(focalLength, pixelAspect);
    intrinsic->setInitialFocalLength(focalLength, pixelAspect);

    if (model == "anamorphic4")
    {        
        std::vector<double> params = 
        {
            boost::json::value_to<double>(obj.at("cx02Degree2")),
            boost::json::value_to<double>(obj.at("cy02Degree2")),
            boost::json::value_to<double>(obj.at("cx22Degree2")),
            boost::json::value_to<double>(obj.at("cy22Degree2")),
            boost::json::value_to<double>(obj.at("cx04Degree4")),
            boost::json::value_to<double>(obj.at("cy04Degree4")),
            boost::json::value_to<double>(obj.at("cx24Degree4")),
            boost::json::value_to<double>(obj.at("cy24Degree4")),
            boost::json::value_to<double>(obj.at("cx44Degree4")),
            boost::json::value_to<double>(obj.at("cy44Degree4")),
            degreeToRadian(boost::json::value_to<double>(obj.at("lensRotation"))),
            boost::json::value_to<double>(obj.at("squeezeX")),
            boost::json::value_to<double>(obj.at("squeezeY")),
        };

        auto undistortion = camera::createUndistortion(camera::EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4);
        undistortion->setSize(w, h);
        undistortion->setPixelAspectRatio(pixelAspect);
        undistortion->setDesqueezed(isDesqueezed);
        undistortion->setParameters(params);
        intrinsic->setUndistortionObject(undistortion);
        intrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
    }
    else if (model == "classicLD")
    {        
        std::vector<double> params = 
        {
            boost::json::value_to<double>(obj.at("distortion")),
            boost::json::value_to<double>(obj.at("anamorphicSqueeze")),
            boost::json::value_to<double>(obj.at("curvatureX")),
            boost::json::value_to<double>(obj.at("curvatureY")),
            boost::json::value_to<double>(obj.at("quarticDistortion"))
        };

        auto undistortion = camera::createUndistortion(camera::EUNDISTORTION::UNDISTORTION_3DECLASSICLD);
        undistortion->setSize(w, h);
        undistortion->setPixelAspectRatio(pixelAspect);
        undistortion->setDesqueezed(isDesqueezed);
        undistortion->setParameters(params);
        intrinsic->setUndistortionObject(undistortion);
        intrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
    }
    else if (model == "radial4")
    {        
        std::vector<double> params = 
        {
            boost::json::value_to<double>(obj.at("distortionDegree2")),
            boost::json::value_to<double>(obj.at("uDegree2")),
            boost::json::value_to<double>(obj.at("vDegree2")),
            boost::json::value_to<double>(obj.at("quarticDistortionDegree4")),
            boost::json::value_to<double>(obj.at("uDegree4")),
            boost::json::value_to<double>(obj.at("vDegree4")),
            boost::json::value_to<double>(obj.at("phiCylindricDirection")),
            boost::json::value_to<double>(obj.at("bCylindricBending")),
        };

        auto undistortion = camera::createUndistortion(camera::EUNDISTORTION::UNDISTORTION_3DERADIAL4);
        undistortion->setSize(w, h);
        undistortion->setPixelAspectRatio(pixelAspect);
        undistortion->setDesqueezed(isDesqueezed);
        undistortion->setParameters(params);
        intrinsic->setUndistortionObject(undistortion);
        intrinsic->setDistortionInitializationMode(camera::EInitMode::CALIBRATED);
    }
    else 
    {
        ALICEVISION_LOG_ERROR("unknown distortion model");
        return false;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::string calibrationFilename;
    bool useJson = true;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData scene to apply calibration to.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.")
        ("useJson", po::value<bool>(&useJson)->default_value(useJson),
         "Calibration is a Lens calibration file generated using 3Dequalizer instead of an sfmData.")
        ("calibration,c", po::value<std::string>(&calibrationFilename)->required(),
         "Calibration file (SfmData or Lens calibration file).");
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
    if (calibrationFilename.empty())
    {
        // Save sfmData to disk
        if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }

    
    if (useJson)
    {
        boost::json::error_code ec;
        std::ifstream inputfile(calibrationFilename); 
        std::vector<boost::json::value> content = readJsons(inputfile, ec);
        if (content.size() != 1)
        {
            ALICEVISION_LOG_ERROR("Invalid calibration file");
            return EXIT_FAILURE;
        }

        if (!applyJson(sfmData, content[0]))
        {
            ALICEVISION_LOG_ERROR("Error applying calibrated json");
            return EXIT_FAILURE;
        }
    }
    else
    {
        // Load calibrated scene
        sfmData::SfMData sfmDataCalibrated;
        if (!sfmDataIO::load(sfmDataCalibrated, calibrationFilename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("The calibrated SfMData file '" << calibrationFilename << "' cannot be read");
            return EXIT_FAILURE;
        }

        if (!applySfmData(sfmData, sfmDataCalibrated))
        {
            ALICEVISION_LOG_ERROR("Error applying calibrated sfmData");
            return EXIT_FAILURE;
        }
    }

    // Save sfmData to disk
    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
