// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/panorama/seams.hpp>

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmData/uid.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/imageAlgo.hpp>

// System
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = std::filesystem;

bool computeWTALabels(image::Image<IndexT>& labels,
                      const std::vector<std::shared_ptr<sfmData::View>>& views,
                      const std::string& inputPath,
                      const std::pair<int, int>& panoramaSize,
                      int downscale)
{
    ALICEVISION_LOG_INFO("Estimating initial labels for panorama");

    WTASeams seams(panoramaSize.first / downscale, panoramaSize.second / downscale);

    for (const auto& viewIt : views)
    {
        IndexT viewId = viewIt->getViewId();
        const std::string warpedPath = viewIt->getImage().getMetadata().at("AliceVision:warpedPath");

        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (warpedPath + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImageDirect(maskPath, mask);
        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, mask);
        }

        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int() / downscale;
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int() / downscale;

        // Load Weights
        const std::string weightsPath = (fs::path(inputPath) / (warpedPath + "_weight.exr")).string();
        ALICEVISION_LOG_TRACE("Load weights with path " << weightsPath);
        image::Image<float> weights;
        image::readImage(weightsPath, weights, image::EImageColorSpace::NO_CONVERSION);
        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, weights);
        }

        if (!seams.appendWithLoop(mask, weights, viewId, offsetX, offsetY))
        {
            return false;
        }
    }

    labels = seams.getLabels();

    return true;
}

bool computeGCLabels(image::Image<IndexT>& labels,
                     const std::vector<std::shared_ptr<sfmData::View>>& views,
                     const std::string& inputPath,
                     std::pair<int, int>& panoramaSize,
                     int smallestViewScale,
                     int downscale)
{
    ALICEVISION_LOG_INFO("Estimating smart seams for panorama");

    const int pyramidSize = 1 + std::max(0, smallestViewScale - 1);
    ALICEVISION_LOG_INFO("Graphcut pyramid size is " << pyramidSize);

    HierarchicalGraphcutSeams seams(panoramaSize.first / downscale, panoramaSize.second / downscale, pyramidSize);

    if (!seams.initialize(labels))
    {
        return false;
    }

    for (const auto& viewIt : views)
    {
        IndexT viewId = viewIt->getViewId();
        const std::string warpedPath = viewIt->getImage().getMetadata().at("AliceVision:warpedPath");

        // Load mask
        const std::string maskPath = (fs::path(inputPath) / (warpedPath + "_mask.exr")).string();
        ALICEVISION_LOG_TRACE("Load mask with path " << maskPath);
        image::Image<unsigned char> mask;
        image::readImageDirect(maskPath, mask);
        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, mask);
        }

        // Load Color
        const std::string colorsPath = (fs::path(inputPath) / (warpedPath + ".exr")).string();
        ALICEVISION_LOG_TRACE("Load colors with path " << colorsPath);
        image::Image<image::RGBfColor> colors;
        image::readImage(colorsPath, colors, image::EImageColorSpace::NO_CONVERSION);
        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, colors);
        }

        // Get offset
        oiio::ParamValueList metadata = image::readImageMetadata(maskPath);
        const std::size_t offsetX = metadata.find("AliceVision:offsetX")->get_int() / downscale;
        const std::size_t offsetY = metadata.find("AliceVision:offsetY")->get_int() / downscale;

        // Append to graph cut
        if (!seams.append(colors, mask, viewId, offsetX, offsetY))
        {
            return false;
        }
    }

    if (!seams.process())
    {
        return false;
    }

    labels = seams.getLabels();

    return true;
}

size_t getGraphcutOptimalScale(int width, int height)
{
    /*
    Look for the smallest scale such that the image is not smaller than the
    convolution window size.
    minsize / 2^x = 5
    minsize / 5 = 2^x
    x = log2(minsize/5)
    */

    const size_t minsize = std::min(width, height);
    const size_t gaussianFilterRadius = 2;

    const int gaussianFilterSize = 1 + 2 * gaussianFilterRadius;

    const size_t optimal_scale = size_t(floor(std::log2(double(minsize) / gaussianFilterSize)));

    return (optimal_scale - 1 /*Security*/);
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmDataFilepath;
    std::string sfmOutDataFilepath;
    std::string warpingFolder;
    std::string outputLabels;
    std::string temporaryCachePath;

    int maxPanoramaWidth = 3000;
    bool useGraphCut = true;
    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    // Description of mandatory parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "Input SfMData.")
        ("warpingFolder,w", po::value<std::string>(&warpingFolder)->required(),
         "Folder with warped images.")
        ("output,o", po::value<std::string>(&outputLabels)->required(),
         "Path of the output labels.")
        ("outputSfm,o", po::value<std::string>(&sfmOutDataFilepath)->required(),
         "Path of the output SfMData file.");
        
    // Description of optional parameters
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxWidth", po::value<int>(&maxPanoramaWidth)->required(),
         "Maximum panorama width.")
        ("useGraphCut,g", po::value<bool>(&useGraphCut)->default_value(useGraphCut),
         "Enable graphcut algorithm to improve seams.");
    // clang-format on

    CmdLine cmdline("Estimates the ideal path for the transition between images in order to minimize seams artifacts.\n"
                    "AliceVision panoramaSeams");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilepath + "' cannot be read");
        return EXIT_FAILURE;
    }

    /*List files*/
    const fs::path p = fs::path(warpingFolder);
    const std::regex pattern("([0-9]+)_([0-9]+).exr");

    std::map<IndexT, std::vector<std::string>> paths_per_view;
    for (auto& iter : fs::directory_iterator(p))
    {
        if (!fs::is_regular_file(iter))
        {
            continue;
        }

        std::smatch m;
        const std::string text = iter.path().string();
        if (!std::regex_search(text, m, pattern))
        {
            continue;
        }

        IndexT index = std::stol(m[1].str());
        paths_per_view[index].push_back(iter.path().stem().string());
    }

    auto copyviews = sfmData.getViews();
    sfmData.getViews().clear();

    for (auto pv : copyviews)
    {
        std::vector<std::string>& images = paths_per_view[pv.first];
        if (images.empty())
            continue;

        for (int idx = 0; idx < images.size(); idx++)
        {
            std::shared_ptr<sfmData::View> newView(pv.second->clone());

            newView->getImage().addMetadata("AliceVision:previousViewId", std::to_string(pv.first));
            newView->getImage().addMetadata("AliceVision:imageCounter", std::to_string(idx));
            newView->getImage().addMetadata("AliceVision:warpedPath", images[idx]);
            const IndexT newIndex = sfmData::computeViewUID(*newView);

            newView->setViewId(newIndex);
            sfmData.getViews().emplace(newIndex, newView);
        }
    }

    sfmDataIO::save(sfmData, sfmOutDataFilepath, sfmDataIO::ESfMData::ALL);

    int tileSize;
    std::pair<int, int> panoramaSize;
    int downscaleFactor = 1;
    {
        const IndexT viewId = *sfmData.getValidViews().begin();
        const std::string warpedPath = sfmData.getViews().at(viewId)->getImage().getMetadata().at("AliceVision:warpedPath");

        const std::string viewFilepath = (fs::path(warpingFolder) / (warpedPath + ".exr")).string();
        ALICEVISION_LOG_TRACE("Read panorama size from file: " << viewFilepath);

        oiio::ParamValueList metadata = image::readImageMetadata(viewFilepath);
        panoramaSize.first = metadata.find("AliceVision:panoramaWidth")->get_int();
        panoramaSize.second = metadata.find("AliceVision:panoramaHeight")->get_int();
        tileSize = metadata.find("AliceVision:tileSize")->get_int();

        if (panoramaSize.first == 0 || panoramaSize.second == 0)
        {
            ALICEVISION_LOG_ERROR("The output panorama size is empty.");
            return EXIT_FAILURE;
        }

        if (tileSize == 0)
        {
            ALICEVISION_LOG_ERROR("no information on tileSize");
            return EXIT_FAILURE;
        }

        if (maxPanoramaWidth > 0 && panoramaSize.first > maxPanoramaWidth)
        {
            downscaleFactor = divideRoundUp(panoramaSize.first, maxPanoramaWidth);
        }

        ALICEVISION_LOG_INFO("Input panorama size is " << panoramaSize.first << "x" << panoramaSize.second);
        ALICEVISION_LOG_INFO("Downscale factor set to " << downscaleFactor);
        ALICEVISION_LOG_INFO("Output labels size set to " << (panoramaSize.first / downscaleFactor) << "x"
                                                          << (panoramaSize.second / downscaleFactor));
    }

    // Get a list of views ordered by their image scale
    int smallestScale = 10000;
    std::vector<std::shared_ptr<sfmData::View>> views;
    for (auto it : sfmData.getViews())
    {
        auto view = it.second;
        IndexT viewId = view->getViewId();

        if (!sfmData.isPoseAndIntrinsicDefined(view.get()))
        {
            // skip unreconstructed views
            continue;
        }

        const std::string warpedPath = view->getImage().getMetadata().at("AliceVision:warpedPath");

        // Load mask
        const std::string maskPath = (fs::path(warpingFolder) / (warpedPath + "_mask.exr")).string();
        int width, height;
        image::readImageSize(maskPath, width, height);
        width /= downscaleFactor;
        height /= downscaleFactor;

        // Estimate scale
        int scale = getGraphcutOptimalScale(width, height);

        smallestScale = std::min(scale, smallestScale);
        views.push_back(view);
    }

    ALICEVISION_LOG_INFO(views.size() << " views to process");

    image::Image<IndexT> labels;
    if (!computeWTALabels(labels, views, warpingFolder, panoramaSize, downscaleFactor))
    {
        ALICEVISION_LOG_ERROR("Error computing initial labels");
        return EXIT_FAILURE;
    }

    if (useGraphCut)
    {
        if (!computeGCLabels(labels, views, warpingFolder, panoramaSize, smallestScale, downscaleFactor))
        {
            ALICEVISION_LOG_ERROR("Error computing graph cut labels");
            return EXIT_FAILURE;
        }
    }

    image::writeImage(outputLabels, labels, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));

    return EXIT_SUCCESS;
}
