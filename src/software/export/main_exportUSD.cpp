// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmDataIO/UsdExporter.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
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
         "Output USD.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("frameRate", po::value<double>(&frameRate)->default_value(frameRate),
         "Define the camera's frame rate in frames per second.");
    // clang-format on

    CmdLine cmdline("AliceVision exportUSD");
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

    using SequenceGroup = std::map<IndexT, std::vector<sfmData::View::sptr>>;
    using IntrinsicGroup = std::map<IndexT, SequenceGroup>;

    //Organize views among intrinsics
    IntrinsicGroup groups;
    for (const auto& [viewId, view] : sfmData.getViews())
    {
        if (!sfmData.isPoseAndIntrinsicDefined(viewId))
        {
            continue;
        }

        if (view->isPartOfRig())
        {
            ALICEVISION_LOG_ERROR("Rigs are not supported");
            return EXIT_FAILURE;
        }    

        groups[view->getIntrinsicId()][view->getFrameId()].push_back(view);
    }

    if (groups.empty())
    {
        ALICEVISION_LOG_ERROR("No views with both pose and supported (pinhole) intrinsics are available for export.");
        return EXIT_FAILURE;
    }
    sfmDataIO::UsdExporter exporter(outputFilename, frameRate);

    for (const auto & [idIntrinsic, group] : groups)
    {
        bool isSequence = true;
        const camera::IntrinsicBase & intrinsic = *(sfmData.getIntrinsics().at(idIntrinsic));

        if (!camera::isPinhole(intrinsic.getType()))
        {
            ALICEVISION_LOG_INFO("Ignoring non pinhole intrinsic " << idIntrinsic << " and all associated views");
            continue;
        }

        const camera::Pinhole & pinhole = dynamic_cast<const camera::Pinhole &>(intrinsic);

        //Check that we don't have multiple view per frame
        for (const auto & [frameId, vector] : group)
        {
            if (vector.size() > 1)
            {
                isSequence = false;
            }
        }

        if (!isSequence)
        {
            ALICEVISION_LOG_INFO("Exporting non sequence");
            for (const auto & [frameId, vector] : group)
            {
                for (const auto & view: vector)
                {
                    const std::string cameraName = "cam_" + std::to_string(view->getViewId());
                    const sfmData::CameraPose & cp = sfmData.getPose(*view);
                    exporter.createNewCamera(cameraName);

                    if (sfmData.existsPoseUncertainty(*view))
                    {
                        const auto & uncertainty = sfmData.getPosesUncertainty().at(view->getPoseId());
                        exporter.addFrameWithUncertainty(cameraName, cp, pinhole, uncertainty, 0);
                    }
                    else 
                    {
                        exporter.addFrame(cameraName, cp, pinhole, 0);
                    }
                }
            }
        }  
        else 
        {
            const std::string cameraName = "cam_" + std::to_string(idIntrinsic);
            ALICEVISION_LOG_INFO("Exporting sequence " << cameraName);
            exporter.createNewCamera(cameraName);

            for (const auto & [frameId, vector] : group)
            {
                for (const auto & view: vector)
                {
                    const sfmData::CameraPose & cp = sfmData.getPose(*view);
                    
                    if (sfmData.existsPoseUncertainty(*view))
                    {
                        const auto & uncertainty = sfmData.getPosesUncertainty().at(view->getPoseId());
                        exporter.addFrameWithUncertainty(cameraName, cp, pinhole, uncertainty, frameId);
                    }
                    else 
                    {
                        exporter.addFrame(cameraName, cp, pinhole, frameId);
                    }
                }
            }
        }      
    }

    exporter.terminate();

    return EXIT_SUCCESS;
}
