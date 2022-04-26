// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <aliceVision/image/all.hpp>
#include <OpenImageIO/imagebufalgo.h>

#include <aliceVision/calibration/checkerDetector.hpp>
#include <aliceVision/calibration/checkerDetector_io.hpp>




// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace aliceVision;




int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string outputFilePath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    int rangeStart = -1;
    int rangeSize = 1;
    bool exportDebugImages = false;

    // Command line parameters
    po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
    ("output,o", po::value<std::string>(&outputFilePath)->required(), "calibration boards json output directory.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), "Range size.")
        ("exportDebugImages", po::value<bool>(&exportDebugImages)->default_value(exportDebugImages), "Export Debug Images.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    // Parse command line
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

    system::Logger::get()->setLogLevel(verboseLevel);

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Order views by their image names for easier debugging
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for(auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }
    std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(),
              [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool
              {
                  if(a == nullptr || b == nullptr)
                      return true;
                  return (a->getImagePath() < b->getImagePath());
              });

    // Define range to compute
    if(rangeStart != -1)
    {
        if(rangeStart < 0 || rangeSize < 0 || std::size_t(rangeStart) > viewsOrderedByName.size())
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if(std::size_t(rangeStart + rangeSize) > viewsOrderedByName.size())
        {
            rangeSize = int(viewsOrderedByName.size()) - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = int(viewsOrderedByName.size());
    }

    ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);



    for (int itemidx = 0; itemidx < rangeSize; itemidx++)
    {
        std::shared_ptr<sfmData::View> view = viewsOrderedByName[rangeStart + itemidx];

        IndexT viewId = view->getViewId();

        //Load image and convert it to linear colorspace
        std::string imagePath = view->getImagePath();
        ALICEVISION_LOG_INFO("Load image with path " << imagePath);
        image::Image<image::RGBColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::SRGB);


        //Lookup checkerboard
        calibration::CheckerDetector detect;
        if(!detect.process(source, exportDebugImages))
        {
            continue;
        }
         
        
        // write the json file with the tree
        std::stringstream ss;
        ss << outputFilePath << "/" << "checkers_" << viewId << ".json";
        boost::json::value jv = boost::json::value_from(detect);
        std::ofstream of(ss.str());
        of << boost::json::serialize(jv);
        of.close();

        if(exportDebugImages)
        {
            for(auto pair : detect.getDebugImages())
            {
                std::stringstream ss;
                ss << outputFilePath << "/" << pair.first << "_" << viewId << ".png";
                image::writeImage(ss.str(), pair.second, image::EImageColorSpace::SRGB);
            }
        }
    }

    return EXIT_SUCCESS;
}
