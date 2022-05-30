// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

//This application is meant to detect a checkerboard or a set of checkerboards in a set of images.
//For application reasons, we are interested in detecting checkerboards which may be :
// - heavily distorted
// - with large blur 
// - partially occluded (Even in the center of the checkerboard)
// - Without any information on its size (with lower bounds)
//
//The checkerboard returned is a matrix of corners id or UndefinedIndexT.
//
//A corner describe :
// - Its center
// - Its two principal directions
// - a scale of detection : the smaller the scale, the "worst" the corner is.
//
//The checkerboard order is the same than the corners coordinates order:
// - if a checkerboard item is at the right of another checkerboard item, 
//   it means its associated corner is at the right of the other corner in the image
//
// - if a checkerboard item is at the bottom of another checkerboard item,
//   it means its associated corner is at the bottom of the other corner in the image

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
    bool doubleSize = false;
    bool useNestedGrids = false;

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
        ("exportDebugImages", po::value<bool>(&exportDebugImages)->default_value(exportDebugImages), "Export Debug Images.")
        ("doubleSize", po::value<bool>(&doubleSize)->default_value(doubleSize), "Double image size prior to processing.")
        ("useNestedGrids", po::value<bool>(&useNestedGrids)->default_value(useNestedGrids), "This image is a nested calibration grid (fully centered).");

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

        double pixelRatio = view->getDoubleMetadata({"PixelAspectRatio"});
        if (pixelRatio < 0.0)
        {
            pixelRatio = 1.0;
        }

        if (pixelRatio != 1.0 || doubleSize)
        {
            // if pixel are not squared, convert the image for easier lines extraction
            const double w = source.Width();
            const double h = source.Height();
            
            const double nw = w * ((doubleSize) ? 2.0 : 1.0);
            const double nh = h * ((doubleSize) ? 2.0 : 1.0) / pixelRatio;

            image::Image<image::RGBColor> resizedInput(nw, nh);

            const oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::UCHAR);
            const oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::UCHAR);

            const oiio::ImageBuf inBuf(imageSpecOrigin, source.data());
            oiio::ImageBuf outBuf(imageSpecResized, resizedInput.data());

            oiio::ImageBufAlgo::resize(outBuf, inBuf);
            source.swap(resizedInput);
        }

        //Lookup checkerboard
        calibration::CheckerDetector detect;
        if(!detect.process(source, useNestedGrids, exportDebugImages))
        {
            continue;
        }

        //Restore aspect ratio for corners coordinates
        if (pixelRatio != 1.0 || doubleSize)
        {
            std::vector<calibration::CheckerDetector::CheckerBoardCorner> & cs = detect.getCorners();
            for (auto &c : cs)
            {
                c.center(1) *= pixelRatio;

                if (doubleSize)
                {
                    c.center(0) /= 2.0;
                    c.center(1) /= 2.0;
                }
            }
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
