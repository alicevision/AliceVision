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
#include <iomanip>

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
    std::string sfmInputDataFilename;
    std::string inputResponsePath;
    std::string sfmOutputDataFilepath;
    int nbBrackets = 3;
    bool byPass = false;
    int channelQuantizationPower = 10;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::SRGB;
    int offsetRefBracketIndex = 0;

    hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
    float highlightCorrectionFactor = 0.0f;
    float highlightTargetLux = 120000.0f;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    int rangeStart = -1;
    int rangeSize = 1;

    // Command line parameters
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
        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())
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

    CmdLine cmdline("This program merges LDR images into HDR images.\n"
                    "AliceVision LdrToHdrMerge");
                  
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

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
    //hdr::selectTargetViews(targetViews, groupedViews, offsetRefBracketIndex);

    const std::string targetIndexFilename = (fs::path(inputResponsePath).parent_path() / (std::string("exposureRefIndexes") + std::string(".txt"))).string();

    std::ifstream file(targetIndexFilename);
    std::vector<int> targetIndexes;
    if (!file)
    {
        throw std::logic_error("Can't open target indexes file");
    }
    //create fileData
    while (file)
    {
        std::string line;
        if (!getline(file, line)) break;
        targetIndexes.push_back(atoi(line.c_str()));
    }
    file.close();

    for (int i = 0; i < groupedViews.size(); ++i)
    {
        targetViews.push_back(groupedViews[i][targetIndexes[i]]);
    }

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
        std::vector<sfmData::ExposureSetting> exposuresSetting(group.size());

        // Load all images of the group
        for(std::size_t i = 0; i < group.size(); ++i)
        {
            const std::string filepath = group[i]->getImagePath();
            ALICEVISION_LOG_INFO("Load " << filepath);

            image::ImageReadOptions options;
            options.workingColorSpace = workingColorSpace;
            options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(group[i]->getRawColorInterpretation());
            options.colorProfileFileName = group[i]->getColorProfileFileName();
            image::readImage(filepath, images[i], options);

            exposuresSetting[i] = group[i]->getCameraExposureSetting(/*targetView->getMetadataISO(), targetView->getMetadataFNumber()*/);
        }
        if(!sfmData::hasComparableExposures(exposuresSetting))
        {
            ALICEVISION_THROW_ERROR("Camera exposure settings are inconsistent.");
        }
        std::vector<double> exposures = getExposures(exposuresSetting);

        // Merge HDR images
        image::Image<image::RGBfColor> HDRimage;
        if(images.size() > 1)
        {
            hdr::hdrMerge merge;
            sfmData::ExposureSetting targetCameraSetting = targetView->getCameraExposureSetting();
            ALICEVISION_LOG_INFO("[" << g - rangeStart << "/" << rangeSize << "] Merge " << group.size() << " LDR images " << g << "/" << groupedViews.size());
            merge.process(images, exposures, fusionWeight, response, HDRimage, targetCameraSetting.getExposure());
            if(highlightCorrectionFactor > 0.0f)
            {
                merge.postProcessHighlight(images, exposures, fusionWeight, response, HDRimage, targetCameraSetting.getExposure(), highlightCorrectionFactor, highlightTargetLux);
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
        targetMetadata.add_or_replace(oiio::ParamValue("AliceVision:ColorSpace",
                                                       image::EImageColorSpace_enumToString(image::EImageColorSpace::LINEAR)));
        image::writeImage(hdrImagePath, HDRimage,
                          image::ImageWriteOptions().storageDataType(storageDataType), targetMetadata);
    }

    return EXIT_SUCCESS;
}
