// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/AlembicExporter.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>

#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;


int aliceVision_main(int argc, char** argv)
{
    // Command-line parameters
    std::string sfmDataFilename;
    std::string outputFilename;

    // User optional parameters
    double frameRate = 24.0;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file containing a complete SfM.")
        ("output,o", po::value<std::string>(&outputFilename)->required(),
         "Output Alembic.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("frameRate", po::value<double>(&frameRate)->default_value(frameRate),
         "Define the camera's Frames per seconds.");
    // clang-format on

    CmdLine cmdline("AliceVision exportAnimatedCamera");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

   
    // Load SfMData files
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (sfmData.getViews().empty())
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' is empty.");
        return EXIT_FAILURE;
    }


    std::map<std::string, std::map<std::size_t, IndexT>> videoViewPerFrame;
    std::map<std::string, std::vector<std::pair<std::size_t, IndexT>>> dslrViewPerKey;

    //Discriminate sequences and image sets
    for (const auto& [viewId, view] : sfmData.getViews().valueRange())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(view))
        {
            continue;
        }

        std::string cameraName = view.getImage().getMetadataMake() 
                                + "_" + view.getImage().getMetadataModel();

        IndexT frameN = 0;
        bool isSequence = false;

        if (view.isPartOfRig())
        {
            cameraName += std::string("_") + std::to_string(view.getSubPoseId());
        }

        std::string prefix;
        std::string suffix;
        const std::string imagePathStem = fs::path(view.getImage().getImagePath()).stem().string();
        if (sfmDataIO::extractNumberFromFileStem(imagePathStem, frameN, prefix, suffix))
        {
            if (prefix.empty() && suffix.empty())
            {
                cameraName = std::string("Undefined") + "_" + cameraName;
            }
            else
            {
                cameraName = prefix + "frame" + suffix + "_" + cameraName;
            }

            isSequence = true;
        }

        if (isSequence)  // Video
        {
            const std::size_t frame = frameN;
            videoViewPerFrame[cameraName][frame] = view.getViewId();
        }
        else if (view.getImage().hasMetadataDateTimeOriginal())  // Picture
        {
            const std::size_t key = view.getImage().getMetadataDateTimestamp();
            dslrViewPerKey[cameraName].push_back({key, view.getViewId()});
        }
        else  // No time or sequence information
        {
            dslrViewPerKey[cameraName].push_back({0, view.getViewId()});
        }
    }


    sfmDataIO::AlembicExporter exporter(outputFilename);

    for (const auto& [cameraName, frameToView] : videoViewPerFrame)
    {
        const std::size_t firstFrame = frameToView.begin()->first;
        const std::size_t lastFrame = frameToView.rbegin()->first;

        exporter.initAnimatedCamera(cameraName, firstFrame, frameRate);

        for (std::size_t frame = firstFrame; frame <= lastFrame; ++frame)
        {
            const auto findFrameIt = frameToView.find(frame);

            if (findFrameIt != frameToView.end())
            {
                const IndexT viewId = findFrameIt->second;

                const sfmData::View & view = sfmData.getView(viewId);
                const IndexT intrinsicId = view.getIntrinsicId();
                const camera::Pinhole * cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(intrinsicId));
                const sfmData::CameraPose pose = sfmData.getPose(view);
                const std::string& imagePath = view.getImage().getImagePath();

                exporter.addCameraKeyframe(pose.getTransform(), cam, imagePath, viewId, intrinsicId);
            }
            else
            {
                exporter.jumpKeyframe(std::to_string(frame));
            }
        }
    }

    for (auto& [cameraName, views] : dslrViewPerKey)
    {
        exporter.initAnimatedCamera(cameraName);

        //Sort views by timestamp
        std::sort(views.begin(), views.end());

        //Loop over images
        for (const auto& [key, id] : views)
        {
            const sfmData::View& view = sfmData.getView(id);
            const camera::Pinhole* cam = dynamic_cast<camera::Pinhole*>(sfmData.getIntrinsicPtr(view.getIntrinsicId()));
            const sfmData::CameraPose pose = sfmData.getPose(view);
            const std::string& imagePath = view.getImage().getImagePath();

            exporter.addCameraKeyframe(pose.getTransform(), cam, imagePath, view.getViewId(), view.getIntrinsicId());
        }
    }

    return EXIT_SUCCESS;
}
