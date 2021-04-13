// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <OpenImageIO/imagebufalgo.h>

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// HDR Related
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/brackets.hpp>

// Command line parameters
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

std::string getHdrImagePath(const std::string& outputPath, std::size_t g)
{
    // Output image file path
    std::stringstream sstream;
    sstream << "hdr_" << std::setfill('0') << std::setw(4) << g << ".exr";
    const std::string hdrImagePath = (fs::path(outputPath) / sstream.str()).string();
    return hdrImagePath;
}


int aliceVision_main(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename;
    std::string inputResponsePath;
    std::string sfmOutputDataFilepath;
    int nbBrackets = 3;
    bool byPass = false;
    int channelQuantizationPower = 10;
    int offsetRefBracketIndex = 0;

    hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
    float highlightCorrectionFactor = 0.0f;
    float highlightTargetLux = 120000.0f;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    int rangeStart = -1;
    int rangeSize = 1;

    // Command line parameters
    po::options_description allParams("Merge LDR images into HDR images.\n"
                                      "AliceVision LdrToHdrMerge");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("response,o", po::value<std::string>(&inputResponsePath)->required(),
        "Input path for the response file.")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets),
         "bracket count per HDR image (0 means automatic).")
        ("byPass", po::value<bool>(&byPass)->default_value(byPass),
         "bypass HDR creation and use medium bracket as input for next steps")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.")
        ("fusionWeight,W", po::value<hdr::EFunctionType>(&fusionWeightFunction)->default_value(fusionWeightFunction),
         "Weight function used to fuse all LDR images together (gaussian, triangle, plateau).")
        ("offsetRefBracketIndex", po::value<int>(&offsetRefBracketIndex)->default_value(offsetRefBracketIndex),
         "Zero to use the center bracket. +N to use a more exposed bracket or -N to use a less exposed backet.")
        ("highlightTargetLux", po::value<float>(&highlightTargetLux)->default_value(highlightTargetLux),
         "Highlights maximum luminance.")
        ("highlightCorrectionFactor", po::value<float>(&highlightCorrectionFactor)->default_value(highlightCorrectionFactor),
         "float value between 0 and 1 to correct clamped highlights in dynamic range: use 0 for no correction, 1 for "
         "full correction to maxLuminance.")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
          "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
          "Range size.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    // Analyze path
    boost::filesystem::path path(sfmOutputDataFilepath);
    std::string outputPath = path.parent_path().string();

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }
    // Check input compatibility with brackets
    const int countImages = sfmData.getViews().size();
    if(countImages == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData contains no image.");
        return EXIT_FAILURE;
    }
    if(nbBrackets > 0 && (countImages % nbBrackets) != 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData file (" << countImages << " images) is not compatible with the number of brackets (" << nbBrackets << " brackets).");
        return EXIT_FAILURE;
    }
    if(nbBrackets == 1 && !byPass)
    {
        ALICEVISION_LOG_WARNING("Enable bypass as there is only one input bracket.");
        byPass = true;
    }

    const std::size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, sfmData, nbBrackets))
    {
        return EXIT_FAILURE;
    }

    {
        std::set<std::size_t> sizeOfGroups;
        for(auto& group : groupedViews)
        {
            sizeOfGroups.insert(group.size());
        }
        if(sizeOfGroups.size() == 1)
        {
            std::size_t usedNbBrackets = *sizeOfGroups.begin();
            if(usedNbBrackets == 1)
            {
                ALICEVISION_LOG_INFO("No multi-bracketing.");
            }
            ALICEVISION_LOG_INFO("Number of brackets automatically detected: "
                                 << usedNbBrackets << ". It will generate " << groupedViews.size()
                                 << " hdr images.");
        }
        else
        {
            ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
            return EXIT_FAILURE;
        }
    }
    std::vector<std::shared_ptr<sfmData::View>> targetViews;
    hdr::selectTargetViews(targetViews, groupedViews, offsetRefBracketIndex);

    // Define range to compute
    if(rangeStart != -1)
    {
      if(rangeStart < 0 || rangeSize < 0 ||
         rangeStart > groupedViews.size())
      {
        ALICEVISION_LOG_ERROR("Range is incorrect");
        return EXIT_FAILURE;
      }

      if(rangeStart + rangeSize > groupedViews.size())
      {
        rangeSize = groupedViews.size() - rangeStart;
      }
    }
    else
    {
        rangeStart = 0;
        rangeSize = groupedViews.size();
    }
    ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);

    if(rangeStart == 0)
    {
        sfmData::SfMData outputSfm;
        outputSfm.getIntrinsics() = sfmData.getIntrinsics();

        // If we are on the first chunk, or we are computing all the dataset
        // Export a new sfmData with HDR images as new Views.
        for(std::size_t g = 0; g < groupedViews.size(); ++g)
        {
            std::shared_ptr<sfmData::View> hdrView = std::make_shared<sfmData::View>(*targetViews[g]);
            if(!byPass)
            {
                const std::string hdrImagePath = getHdrImagePath(outputPath, g);
                hdrView->setImagePath(hdrImagePath);
            }
            outputSfm.getViews()[hdrView->getViewId()] = hdrView;
        }

        // Export output sfmData
        if(!sfmDataIO::Save(outputSfm, sfmOutputDataFilepath, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilepath);
            return EXIT_FAILURE;
        }
    }
    if(byPass)
    {
        ALICEVISION_LOG_INFO("Bypass enabled, nothing to compute.");
        return EXIT_SUCCESS;
    }

    hdr::rgbCurve fusionWeight(channelQuantization);
    fusionWeight.setFunction(fusionWeightFunction);

    ALICEVISION_LOG_DEBUG("inputResponsePath: " << inputResponsePath);

    hdr::rgbCurve response(channelQuantization);
    response.read(inputResponsePath);

    for(std::size_t g = rangeStart; g < rangeStart + rangeSize; ++g)
    {
        const std::vector<std::shared_ptr<sfmData::View>>& group = groupedViews[g];

        std::vector<image::Image<image::RGBfColor>> images(group.size());
        std::shared_ptr<sfmData::View> targetView = targetViews[g];
        std::vector<float> exposures(group.size(), 0.0f);

        // Load all images of the group
        for(std::size_t i = 0; i < group.size(); ++i)
        {
            const std::string filepath = group[i]->getImagePath();
            ALICEVISION_LOG_INFO("Load " << filepath);

            image::ImageReadOptions options;
            options.outputColorSpace = image::EImageColorSpace::SRGB;
            options.applyWhiteBalance = group[i]->getApplyWhiteBalance();
            image::readImage(filepath, images[i], options);

            exposures[i] = group[i]->getCameraExposureSetting(/*targetView->getMetadataISO(), targetView->getMetadataFNumber()*/);
        }

        // Merge HDR images
        image::Image<image::RGBfColor> HDRimage;
        if(images.size() > 1)
        {
            hdr::hdrMerge merge;
            float targetCameraExposure = targetView->getCameraExposureSetting();
            ALICEVISION_LOG_INFO("[" << g - rangeStart << "/" << rangeSize << "] Merge " << group.size() << " LDR images " << g << "/" << groupedViews.size());
            merge.process(images, exposures, fusionWeight, response, HDRimage, targetCameraExposure);
            if(highlightCorrectionFactor > 0.0f)
            {
                merge.postProcessHighlight(images, exposures, fusionWeight, response, HDRimage, targetCameraExposure, highlightCorrectionFactor, highlightTargetLux);
            }
        }
        else if(images.size() == 1)
        {
            // Nothing to do
            HDRimage = images[0];
        }

        const std::string hdrImagePath = getHdrImagePath(outputPath, g);

        // Write an image with parameters from the target view
        oiio::ParamValueList targetMetadata = image::readImageMetadata(targetView->getImagePath());
        targetMetadata.push_back(oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));

        image::writeImage(hdrImagePath, HDRimage, image::EImageColorSpace::AUTO, targetMetadata);
    }

    return EXIT_SUCCESS;
}
