// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/calibration/checkerDetector.hpp>
#include <aliceVision/calibration/checkerDetector_io.hpp>

#include <OpenImageIO/imagebufalgo.h>

#include <boost/program_options.hpp>
#include <boost/property_tree/json_parser.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;

using namespace aliceVision;

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string outputFilePath;
    int rangeStart = -1;
    int rangeSize = 1;
    bool exportDebugImages = false;
    bool doubleSize = false;
    bool useNestedGrids = false;

    // Command line parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(),
         "SfMData file input.")
        ("output,o", po::value<std::string>(&outputFilePath)->required(),
         "Calibration boards JSON output directory.");

    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), 
         "Range start for processing views (ordered by image filepath). Set to -1 to process all images.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), 
         "Range size for processing views (ordered by image filepath).")
        ("exportDebugImages", po::value<bool>(&exportDebugImages)->default_value(exportDebugImages), 
         "Export debug images.")
        ("doubleSize", po::value<bool>(&doubleSize)->default_value(doubleSize), 
         "Double image size prior to processing.")
        ("useNestedGrids", po::value<bool>(&useNestedGrids)->default_value(useNestedGrids), 
         "Images contain nested calibration grids. These grids must be centered on image center.");
    // clang-format on

    CmdLine cmdline("AliceVision checkerboardDetection");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Order views by their image names
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for (auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }
    std::sort(viewsOrderedByName.begin(),
              viewsOrderedByName.end(),
              [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
                  if (a == nullptr || b == nullptr)
                      return true;
                  return (a->getImage().getImagePath() < b->getImage().getImagePath());
              });

    // Define range to compute
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0 || static_cast<std::size_t>(rangeStart) > viewsOrderedByName.size())
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (static_cast<std::size_t>(rangeStart + rangeSize) > viewsOrderedByName.size())
        {
            rangeSize = static_cast<int>(viewsOrderedByName.size()) - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = static_cast<int>(viewsOrderedByName.size());
    }

    ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);

    for (int itemidx = 0; itemidx < rangeSize; itemidx++)
    {
        std::shared_ptr<sfmData::View> view = viewsOrderedByName[rangeStart + itemidx];

        IndexT viewId = view->getViewId();

        // Load image and convert it to sRGB colorspace
        std::string imagePath = view->getImage().getImagePath();
        ALICEVISION_LOG_INFO("Load image with path " << imagePath);
        image::Image<image::RGBColor> source;
        image::readImage(imagePath, source, image::EImageColorSpace::SRGB);

        double pixelRatio = view->getImage().getDoubleMetadata({"PixelAspectRatio"});
        if (pixelRatio < 0.0)
        {
            pixelRatio = 1.0;
        }

        if (pixelRatio != 1.0 || doubleSize)
        {
            // if pixel are not squared, convert the image for easier lines extraction
            const double w = source.width();
            const double h = source.height();

            const double nw = w * ((doubleSize) ? 2.0 : 1.0);
            const double nh = h * ((doubleSize) ? 2.0 : 1.0) / pixelRatio;

            ALICEVISION_LOG_DEBUG("Resize image with dimensions " << nw << "x" << nh);

            image::Image<image::RGBColor> resizedInput;
            imageAlgo::resizeImage(nw, nh, source, resizedInput);
            source.swap(resizedInput);
        }

        // Lookup checkerboard
        calibration::CheckerDetector detect;
        ALICEVISION_LOG_INFO("Launching checkerboard detection");
        if (!detect.process(source, useNestedGrids, exportDebugImages))
        {
            ALICEVISION_LOG_ERROR("Detection failed");
            continue;
        }

        ALICEVISION_LOG_INFO("Detected " << detect.getBoards().size() << " boards and " << detect.getCorners().size() << " corners");

        // Restore aspect ratio for corners coordinates
        if (pixelRatio != 1.0 || doubleSize)
        {
            std::vector<calibration::CheckerDetector::CheckerBoardCorner>& cs = detect.getCorners();
            for (auto& c : cs)
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
        ALICEVISION_LOG_INFO("Writing detection output in "
                             << "checkers_" << viewId << ".json");
        std::stringstream ss;
        ss << outputFilePath << "/"
           << "checkers_" << viewId << ".json";
        boost::json::value jv = boost::json::value_from(detect);
        std::ofstream of(ss.str());
        of << boost::json::serialize(jv);
        of.close();

        if (exportDebugImages)
        {
            ALICEVISION_LOG_INFO("Writing debug image");
            std::stringstream ss;
            ss << outputFilePath << "/" << viewId << ".png";
            image::writeImage(ss.str(), detect.getDebugImage(), image::ImageWriteOptions());
        }
    }

    return EXIT_SUCCESS;
}
