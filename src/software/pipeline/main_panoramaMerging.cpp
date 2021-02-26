// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// System
#include <aliceVision/system/MemoryInfo.hpp>
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string compositingFolder;
    std::string outputPanoramaPath;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    // Program description
    po::options_description allParams(
        "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
        "AliceVision PanoramaCompositing");

    // Description of mandatory parameters
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(), "Input sfmData.")
        ("compositingFolder,w", po::value<std::string>(&compositingFolder)->required(), "Folder with composited images.")
        ("outputPanorama,o", po::value<std::string>(&outputPanoramaPath)->required(), "Path of the output panorama.");
    allParams.add(requiredParams);

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType), ("Storage data type: " + image::EStorageDataType_informations()).c_str());
    allParams.add(optionalParams);

    // Setup log level given command line
    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
    allParams.add(logParams);

    // Effectively parse command line given parse options
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // Set verbose level given command line
    system::Logger::get()->setLogLevel(verboseLevel);

    // load input scene
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }


    bool first = true;
    image::Image<image::RGBAfColor> panorama;

    for (auto viewItem : sfmData.getViews())
    {
        IndexT viewId = viewItem.first;
        if(!sfmData.isPoseAndIntrinsicDefined(viewId))
            continue;

        // Get composited image path
        const std::string imagePath = (fs::path(compositingFolder) / (std::to_string(viewId) + ".exr")).string();
        
        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
        const int offsetX = metadata.find("AliceVision:offsetX")->get_int();
        const int offsetY = metadata.find("AliceVision:offsetY")->get_int();
        const int panoramaWidth = metadata.find("AliceVision:panoramaWidth")->get_int();
        const int panoramaHeight = metadata.find("AliceVision:panoramaHeight")->get_int();

        if (first) 
        {
            panorama = image::Image<image::RGBAfColor>(panoramaWidth, panoramaHeight, true, image::RGBAfColor(0.0f, 0.0f, 0.f, 0.0f));
            first = false;
        }

        // Load image
        ALICEVISION_LOG_TRACE("Load image with path " << imagePath);
        image::Image<image::RGBAfColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::NO_CONVERSION);

        for (int i = 0; i < source.Height(); i++)
        {
            for (int j = 0; j < source.Width(); j++)
            {
                image::RGBAfColor pix = source(i, j);

                if (pix.a() > 0.9) {

                    int nx = offsetX + j;
                    if (nx < 0) nx += panoramaWidth;
                    if (nx >= panoramaWidth) nx -= panoramaWidth;

                    panorama(offsetY + i, nx) = pix;
                }
            }
        }
    }

    oiio::ParamValueList targetMetadata;
    targetMetadata.push_back(oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));
    image::writeImage(outputPanoramaPath, panorama, image::EImageColorSpace::AUTO, targetMetadata);

    return EXIT_SUCCESS;
}
