// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/io.hpp>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/types.hpp>

// Lighting calibration
#include <aliceVision/lightingEstimation/lightingCalibration.hpp>
#include <aliceVision/lightingEstimation/ellipseGeometry.hpp>
#include <aliceVision/lightingEstimation/sphereData.hpp>
#include <aliceVision/lightingEstimation/lightingData.hpp>

// Command line parameters
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <fstream>
#include <filesystem>

#include <Windows.h>
  
// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

// Namespaces
namespace po = boost::program_options;
namespace fs = std::filesystem;

using namespace aliceVision;

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;
 
    std::string inputPath;
    std::vector<std::string> inputJSONvec;
    aliceVision::IndexT viewId;
    std::string outputJSON;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputPath,i", po::value<std::string>(&inputPath)->required(),
         "Path to the SfMData input.")
        ("inputFile, j", po::value<std::vector<std::string>>(&inputJSONvec)->required()->multitoken(),
         "Path to JSON input lightings file.")
        ("viewId, c", po::value<aliceVision::IndexT>(&viewId)->required(),
         "Required viewId")
        ("outputFile, o", po::value<std::string>(&outputJSON)->required(),
         "Path to JSON output camera file.");

    CmdLine cmdline("AliceVision lightingConversionPerCamera");
    cmdline.add(requiredParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (fs::is_directory(inputPath))
    {
        ALICEVISION_LOG_ERROR("Directory input: WIP");
        ALICEVISION_THROW(std::invalid_argument, "Input directories are not yet supported");
    }
    else
    {
		ALICEVISION_LOG_INFO("Opening SFM file");
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, inputPath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input file '" + inputPath + "' cannot be read.");
            return EXIT_FAILURE;
        }

		ALICEVISION_LOG_INFO("Opening lightings file");
        lightingEstimation::Lightings lightings;
        for (unsigned int i = 0; i < inputJSONvec.size(); i++)
		{
            ALICEVISION_LOG_INFO("Loading " << inputJSONvec[i]);
			if(!lightingEstimation::LightingDataIO::loadJSON(inputJSONvec[i], lightings))
			{
				ALICEVISION_LOG_ERROR("Could not load lightings");
				return EXIT_FAILURE;
			}
        }

		std::cout << "viewId" << viewId << std::endl;
        auto& view = sfmData.getViews().at(viewId);
        auto& pose = sfmData.getPoses().at(view->getPoseId());

        // convert to view frame
        lightingEstimation::Lightings lightingsForView;
        for (auto itLight = lightings.begin(); itLight != lightings.end(); itLight++)
        {
            auto newLight = itLight->second->convertToFrame(pose);
            lightingsForView.emplace(itLight->first, newLight);
        }

        // save
        lightingEstimation::LightingDataIO::saveJSON(lightingsForView, outputJSON);
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
