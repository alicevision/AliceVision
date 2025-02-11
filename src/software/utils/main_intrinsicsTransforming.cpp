// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <boost/program_options.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

/**
 * @brief create a new SfmData where all the non pinhole stuff are removed
 * For example, distortion/undistortion are removed
 * @param sfmData the original sfmData
 * @param outputSfmData the result sfmData
 * @param fakeFov if one intrinsic is non pinhole, what is the required fov for the "fake" camera
 * @return true if everything worked
*/
bool convertToPinhole(const sfmData::SfMData & sfmData, 
                    sfmData::SfMData & outputSfmData,
                    double fakeFov)
{
    outputSfmData.getIntrinsics().clear();

    // Loop over all input intrinsics
    for (const auto & [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        const auto & originalIntrinsic = *intrinsicPtr;

        bool isPinhole = camera::isPinhole(originalIntrinsic.getType());

        //BY default, create a fake camera with a given fov
        double hw = double(originalIntrinsic.w()) * 0.5;
        double fx = hw / std::tan(fakeFov * 0.5);
        double fy = fx;
        double cx = 0.0;
        double cy = 0.0;

        if (isPinhole)
        {
            //IF pinhone, recreate without distortion
            const auto & pinhole = dynamic_cast<const camera::Pinhole &>(originalIntrinsic);
            fx = pinhole.getScale().x();
            fy = pinhole.getScale().y();
            cx = pinhole.getOffset().x();
            cy = pinhole.getOffset().y();
        }

        std::shared_ptr<camera::IntrinsicBase> fakecam = camera::createIntrinsic(
                                                camera::EINTRINSIC::PINHOLE_CAMERA, 
                                                camera::DISTORTION_NONE, 
                                                camera::UNDISTORTION_NONE, 
                                                intrinsicPtr->w(), 
                                                intrinsicPtr->h(),
                                                fx, fy, cx, cy
                                                );
        
        outputSfmData.getIntrinsics().insert({intrinsicId, fakecam});
    }

    return true;
}

/**
 * @brief create a new SfmData where all the intrinsics are converted to equirectangular
 * For example, distortion/undistortion are removed
 * @param sfmData the original sfmData
 * @param outputSfmData the result sfmData
 * @return true if everything worked
*/
bool convertToEquirectangular(const sfmData::SfMData & sfmData, 
                            sfmData::SfMData & outputSfmData)
{
    outputSfmData.getIntrinsics().clear();

    // Loop over all input intrinsics
    for (const auto & [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        const auto & originalIntrinsic = *intrinsicPtr;

        size_t minSize = std::min(originalIntrinsic.w(), originalIntrinsic.h());
        double fx = double(minSize) / M_PI;
        double fy = fx;

        std::shared_ptr<camera::IntrinsicBase> fakecam = camera::createIntrinsic(
                                                camera::EINTRINSIC::EQUIRECTANGULAR_CAMERA, 
                                                camera::DISTORTION_NONE, 
                                                camera::UNDISTORTION_NONE, 
                                                minSize * 2, minSize,
                                                fx, fy, 0, 0
                                                );

        outputSfmData.getIntrinsics().insert({intrinsicId, fakecam});
    }

    return true;
}

/**
 * @brief Convert all obsrvation to simulate that they were observed using the new intrinsics
 * @param sfmData the original sfmData
 * @param outputSfmData the result sfmData
 * @return true if everything worked
*/
bool convertObservations(const sfmData::SfMData & sfmData, 
                        sfmData::SfMData & outputSfmData)
{
    for (auto & [idLandmark, landmark] : outputSfmData.getLandmarks())
    {
        //Copy observations and erase
        const auto observationsCopy = landmark.getObservations();
        landmark.getObservations().clear();

        for (const auto & [idView, obs] : observationsCopy)
        {
            //Ignore non reconstructed views
            const auto & view = sfmData.getView(idView);

            //Undistort observation
            IndexT intrinsicId = view.getIntrinsicId();
            if (intrinsicId == UndefinedIndexT)
            {
                continue;
            }

            const auto & inputIntrinsic = sfmData.getIntrinsic(intrinsicId);
            const auto & outputIntrinsic = outputSfmData.getIntrinsic(intrinsicId);

            const Vec3 intermediate = inputIntrinsic.backProjectUnit(obs.getCoordinates());
            const Vec2 undistorted = outputIntrinsic.project(intermediate.homogeneous(), false);

            if (undistorted.x() < 0 || undistorted.y() < 0)
            {
                continue;
            }

            if (undistorted.x() >= outputIntrinsic.w() || undistorted.y() >= outputIntrinsic.h())
            {
                continue;
            }

            sfmData::Observation outputObservation = obs;
            outputObservation.setCoordinates(undistorted);
            landmark.getObservations()[idView] = outputObservation;
        }
    }
    return true;
}

/**
 * @brief Convert all tracks to simulate that they were observed using the new intrinsics
 * @param inputSfmData the original sfmData
 * @param outputSfmData the  sfmData with updated intrinsics
 * @return true if everything worked
*/
bool convertTracks(const sfmData::SfMData & inputSfmData, 
                  const sfmData::SfMData & outputSfmData,
                  track::TracksHandler& tracksHandler)
{
    track::TracksMap & tracksMap = tracksHandler.getAllTracksMutable();
    const track::TracksPerView tpv = tracksHandler.getTracksPerView();

    for (const auto & [viewId, trackids]: tpv)
    {
        const auto & view = inputSfmData.getView(viewId);
        IndexT intrinsicId = view.getIntrinsicId();
        if (intrinsicId == UndefinedIndexT)
        {
            continue;
        }

        const auto & inputIntrinsic = inputSfmData.getIntrinsic(intrinsicId);
        const auto & outputIntrinsic = outputSfmData.getIntrinsic(intrinsicId);

        for (const auto & trackId : trackids)
        {
            auto & track = tracksMap.at(trackId);
            auto & item = track.featPerView[viewId];

            const Vec3 intermediate = inputIntrinsic.backProjectUnit(item.coords);
            const Vec2 undistorted = outputIntrinsic.project(intermediate.homogeneous(), false);

            item.coords = undistorted;
        }
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputSfmDataFilename;
    std::string outputSfmDataFilename;
    std::string inputTracksFilename;
    std::string outputTracksFilename;
    std::string cameraTypeStr;
    double fakeFov = 90.0;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmDataFilename)->required(),
         "Input SfMData file.")
        ("output,o", po::value<std::string>(&outputSfmDataFilename)->required(),
         "Output folder.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("fakeFov", po::value<double>(&fakeFov)->default_value(fakeFov),
         "Virtual FOV if output is pinhole and input is not.")
        ("type", po::value<std::string>(&cameraTypeStr)->default_value(cameraTypeStr),
         "Default camera model type (pinhole, equidistant, equirectangular).")
        ("inputTracks", po::value<std::string>(&inputTracksFilename)->required(),
         "Input Tracks file.")
        ("outputTracks", po::value<std::string>(&outputTracksFilename)->required(),
         "Output Tracks file.");
    // clang-format on

    CmdLine cmdline("AliceVision prepareDenseScene");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    camera::EINTRINSIC cameraType = camera::EINTRINSIC::PINHOLE_CAMERA;
    if (!cameraTypeStr.empty())
    {
        cameraType = camera::EINTRINSIC_stringToEnum(cameraTypeStr);
    }

    sfmDataIO::ESfMData flagsPart = sfmDataIO::ESfMData(
                sfmDataIO::ESfMData::ALL
            );

    // Read the input SfM scene
    sfmData::SfMData inputSfmData;
    if (!sfmDataIO::load(inputSfmData, inputSfmDataFilename, flagsPart))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    sfmData::SfMData outputSfmData(inputSfmData);
    if (cameraType == camera::EINTRINSIC::PINHOLE_CAMERA)
    {
        if (!convertToPinhole(inputSfmData, outputSfmData, fakeFov))
        {
            ALICEVISION_LOG_ERROR("There was an error converting intrinsics");
            return EXIT_FAILURE;
        }
    }
    else if (cameraType == camera::EINTRINSIC::EQUIRECTANGULAR_CAMERA)
    {
        if (!convertToEquirectangular(inputSfmData, outputSfmData))
        {
            ALICEVISION_LOG_ERROR("There was an error converting intrinsics");
            return EXIT_FAILURE;
        }
    }
    else 
    {
        ALICEVISION_LOG_ERROR("Invalid camera model");
        return EXIT_FAILURE;
    }

    if (!convertObservations(inputSfmData, outputSfmData))
    {
        ALICEVISION_LOG_ERROR("There was an error converting observations");
        return EXIT_FAILURE;
    }

    if (!inputTracksFilename.empty())
    {
        // Load tracks
        ALICEVISION_LOG_INFO("Load tracks");
        track::TracksHandler tracksHandler;
        if (!tracksHandler.load(inputTracksFilename, inputSfmData.getViewsKeys()))
        {
            ALICEVISION_LOG_ERROR("The input tracks file '" + inputTracksFilename + "' cannot be read.");
            return EXIT_FAILURE;
        }

        if (!convertTracks(inputSfmData, outputSfmData, tracksHandler))
        {
            ALICEVISION_LOG_ERROR("There was an error converting tracks");
            return EXIT_FAILURE;
        }
    }

    ALICEVISION_LOG_INFO("Export SfM: " << outputSfmDataFilename);
    if (!sfmDataIO::save(outputSfmData, outputSfmDataFilename, flagsPart))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outputSfmDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
