// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/keyframe/KeyframeSelector.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision::keyframe;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // Command-line parameters
    std::string sensorDbPath;               // camera sensor width database
    std::string outputFolder;               // output folder for keyframes
    bool useRegularKeyframes = true;        // use regular keyframes method

    // Info for each "rig" view
    std::vector<std::string> mediaPaths;    // media file path list
    std::vector<std::string> brands;        // media brand list
    std::vector<std::string> models;        // media model list
    std::vector<float> mmFocals;            // media focal (mm) list

    // Algorithm variables
    unsigned int minFrameStep = 12;
    unsigned int maxNbOutFrame = 0;

    po::options_description inputParams("Required parameters");  
    inputParams.add_options()
        ("mediaPaths", po::value< std::vector<std::string> >(&mediaPaths)->required()->multitoken(),
            "Input video files or image sequence directories.")
        ("sensorDbPath", po::value<std::string>(&sensorDbPath)->required(),
            "Camera sensor width database path.")
        ("outputFolder", po::value<std::string>(&outputFolder)->required(),
            "Output keyframes folder for .jpg.")
        ("useRegularKeyframes", po::value<bool>(&useRegularKeyframes)->required(),
            "Use regular keyframe extraction instead of the smart method.");

    po::options_description metadataParams("Metadata parameters");  
    metadataParams.add_options()
        ("brands", po::value< std::vector<std::string> >(&brands)->default_value(brands)->multitoken(),
            "Camera brands.")
        ("models", po::value< std::vector<std::string> >(&models)->default_value(models)->multitoken(),
            "Camera models.")
        ("mmFocals", po::value< std::vector<float> >(&mmFocals)->default_value(mmFocals)->multitoken(),
            "Focals in mm (will be used if not 0).");
    
    po::options_description algorithmParams("Algorithm parameters");
    algorithmParams.add_options()
        ("minFrameStep", po::value<unsigned int>(&minFrameStep)->default_value(minFrameStep), 
            "Minimum number of frames between two keyframes.")
        ("maxNbOutFrame", po::value<unsigned int>(&maxNbOutFrame)->default_value(maxNbOutFrame), 
            "Maximum number of output frames (0 = no limit).");

    aliceVision::CmdLine cmdline("This program is used to extract keyframes from single camera or a camera rig");
    cmdline.add(inputParams);
    cmdline.add(metadataParams);
    cmdline.add(algorithmParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    const std::size_t nbCameras = mediaPaths.size();

    // Check output folder and update to its absolute path
    {
        const fs::path outDir = fs::absolute(outputFolder);
        outputFolder = outDir.string();
        if (!fs::is_directory(outDir))
        {
            ALICEVISION_LOG_ERROR("Cannot find folder: " << outputFolder);
            return EXIT_FAILURE;
        }
    }

    if (minFrameStep <= 0)
    {
        ALICEVISION_LOG_ERROR("Min frame step must be at least 1");
        return EXIT_FAILURE;
    }

    if (nbCameras < 1)
    {
        ALICEVISION_LOG_ERROR("Program need at least one media path.");
        return EXIT_FAILURE;
    }

    // Put default value is not filled
    brands.resize(nbCameras);
    models.resize(nbCameras);
    mmFocals.resize(nbCameras);

    // Initialize KeyframeSelector
    KeyframeSelector selector;

    // Set algorithm parameters
    selector.setMinFrameStep(minFrameStep);
    selector.setMaxOutFrame(maxNbOutFrame);

    // process
    if (useRegularKeyframes)
    {
        selector.processRegular(mediaPaths);
    }
    else
    {
        selector.processSmart(mediaPaths);
    }

    if (!selector.writeSelection(outputFolder, mediaPaths, brands, models, mmFocals))
    {
        ALICEVISION_LOG_ERROR("Impossible to write selection");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
