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
#include <aliceVision/sfmDataIO/UsdExporter.hpp>
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
         "Output USD.");

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


    std::map<IndexT, sfmData::View::sptr> viewsByFrame;

    //Discriminate sequences and image sets
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

        viewsByFrame[view->getFrameId()] == view;
    }

    long firstFrameId = viewsByFrame.begin()->first;
    long lastFrameId = viewsByFrame.rbegin()->first;

    sfmDataIO::UsdExporter exporter(outputFilename);

    exporter.createNewCamera(firstFrameId, lastFrameId, frameRate);

    for (long frameId = firstFrameId; frameId < lastFrameId; frameId++)
    {
        if (viewsByFrame.find(frameId) == viewsByFrame.end())
        {
            continue;
        }

        auto view = viewsByFrame.at(frameId);
        
        const camera::IntrinsicBase::sptr intrinsic = sfmData.getIntrinsics().at(view->getIntrinsicId());

        exporter.addFrame(ptr, intrinsic);
    }

    return EXIT_SUCCESS;
}
