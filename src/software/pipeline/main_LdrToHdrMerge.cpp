// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <OpenImageIO/imageio.h>
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

#include <filesystem>
#include <sstream>
#include <iomanip>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

std::string getHdrImagePath(const std::string& outputPath, std::size_t g, const std::string& rootname = "")
{
    // Output image file path
    std::stringstream sstream;
    if (rootname == "")
    {
        sstream << "hdr_" << std::setfill('0') << std::setw(4) << g << ".exr";
    }
    else
    {
        sstream << rootname << ".exr";
    }
    const std::string hdrImagePath = (fs::path(outputPath) / sstream.str()).string();
    return hdrImagePath;
}

std::string getHdrMaskPath(const std::string& outputPath, std::size_t g, const std::string& maskname, const std::string& rootname = "")
{
    // Output image file path
    std::stringstream sstream;
    if (rootname == "")
    {
        sstream << "hdrMask_" << maskname << "_" << std::setfill('0') << std::setw(4) << g << ".exr";
    }
    else
    {
        sstream << rootname << "_" << maskname << ".exr";
    }
    const std::string hdrImagePath = (fs::path(outputPath) / sstream.str()).string();
    return hdrImagePath;
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmInputDataFilename;
    std::string inputResponsePath;
    std::string sfmOutputDataFilepath;
    int nbBrackets = 0;
    bool byPass = false;
    bool keepSourceImageName = false;
    int channelQuantizationPower = 10;
    int offsetRefBracketIndex = 1000;  // By default, use the automatic selection
    double meanTargetedLumaForMerging = 0.4;
    double minSignificantValue = 0.05;
    double maxSignificantValue = 0.995;
    bool computeLightMasks = false;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::AUTO;

    hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
    float highlightCorrectionFactor = 0.0f;
    float highlightTargetLux = 120000.0f;

    image::EStorageDataType storageDataType = image::EStorageDataType::Float;

    int rangeStart = -1;
    int rangeSize = 1;

    // Command line parameters
    // clang-format off
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
         "Bracket count per HDR image (0 means automatic).")
        ("byPass", po::value<bool>(&byPass)->default_value(byPass),
         "Bypass HDR creation and use a single bracket as the input for the next steps.")
        ("keepSourceImageName", po::value<bool>(&keepSourceImageName)->default_value(keepSourceImageName),
         "Use the filename of the input image selected as the central image for the output image filename.")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.")
        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())
        ("fusionWeight,W", po::value<hdr::EFunctionType>(&fusionWeightFunction)->default_value(fusionWeightFunction),
         "Weight function used to fuse all the LDR images together (gaussian, triangle, plateau).")
        ("offsetRefBracketIndex", po::value<int>(&offsetRefBracketIndex)->default_value(offsetRefBracketIndex),
         "Zero to use the center bracket. +N to use a more exposed bracket or -N to use a less exposed bracket.")
        ("meanTargetedLumaForMerging", po::value<double>(&meanTargetedLumaForMerging)->default_value(meanTargetedLumaForMerging),
         "Mean expected luminance after merging step when the input LDR images are decoded in the sRGB color space. "
         "Must be in the range [0, 1].")
        ("minSignificantValue", po::value<double>(&minSignificantValue)->default_value(minSignificantValue),
         "Minimum channel input value to be considered in advanced pixelwise merging. "
         "Used in advanced pixelwise merging.")
        ("maxSignificantValue", po::value<double>(&maxSignificantValue)->default_value(maxSignificantValue),
         "Maximum channel input value to be considered in advanced pixelwise merging. "
         "Used in advanced pixelwise merging.")
        ("computeLightMasks", po::value<bool>(&computeLightMasks)->default_value(computeLightMasks),
         "Compute masks of dark and high lights and missing mid lights info.")
        ("highlightTargetLux", po::value<float>(&highlightTargetLux)->default_value(highlightTargetLux),
         "Highlights maximum luminance.")
        ("highlightCorrectionFactor", po::value<float>(&highlightCorrectionFactor)->default_value(highlightCorrectionFactor),
         "Float value between 0 and 1 to correct clamped highlights in dynamic range: use 0 for no correction, 1 for "
         "full correction to maxLuminance.")
        ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
         ("Storage data type: " + image::EStorageDataType_informations()).c_str())
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.");
    // clang-format on

    CmdLine cmdline("This program merges LDR images into HDR images.\n"
                    "AliceVision LdrToHdrMerge");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // Analyze path
    fs::path path(sfmOutputDataFilepath);
    std::string outputPath = path.parent_path().string();

    // Read SfMData
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }
    // Check input compatibility with brackets
    const int countImages = sfmData.getViews().size();
    if (countImages == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData contains no image.");
        return EXIT_FAILURE;
    }
    if (nbBrackets == 1 && !byPass)
    {
        ALICEVISION_LOG_WARNING("Enable bypass as there is only one input bracket.");
        byPass = true;
    }

    const std::size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Estimate groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, sfmData, nbBrackets))
    {
        ALICEVISION_LOG_ERROR("Failure to estimate brackets.");
        return EXIT_FAILURE;
    }

    // Check groups
    std::size_t usedNbBrackets;
    {
        std::set<std::size_t> sizeOfGroups;
        for (auto& group : groupedViews)
        {
            sizeOfGroups.insert(group.size());
        }
        if (sizeOfGroups.size() == 1)
        {
            usedNbBrackets = *sizeOfGroups.begin();
            if (usedNbBrackets == 1)
            {
                ALICEVISION_LOG_INFO("No multi-bracketing.");
            }
            ALICEVISION_LOG_INFO("Number of brackets automatically detected: " << usedNbBrackets << ". It will generate " << groupedViews.size()
                                                                               << " HDR images.");
        }
        else
        {
            ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
            return EXIT_FAILURE;
        }
    }

    // Group all groups sharing the same intrinsic
    std::map<IndexT, std::vector<std::vector<std::shared_ptr<sfmData::View>>>> groupedViewsPerIntrinsics;
    for (const auto& group : groupedViews)
    {
        IndexT intrinsicId = UndefinedIndexT;

        for (const auto& v : group)
        {
            IndexT lid = v->getIntrinsicId();
            if (intrinsicId == UndefinedIndexT)
            {
                intrinsicId = lid;
            }

            if (lid != intrinsicId)
            {
                ALICEVISION_LOG_INFO("One group shall not have multiple intrinsics.");
                return EXIT_FAILURE;
            }
        }

        if (intrinsicId == UndefinedIndexT)
        {
            ALICEVISION_LOG_INFO("One group has no intrinsics.");
            return EXIT_FAILURE;
        }

        groupedViewsPerIntrinsics[intrinsicId].push_back(group);
    }

    // Estimate working color space if set to AUTO
    if (workingColorSpace == image::EImageColorSpace::AUTO)
    {
        const bool isRAW = image::isRawFormat(groupedViews[0][0]->getImage().getImagePath());

        workingColorSpace = isRAW ? image::EImageColorSpace::LINEAR : image::EImageColorSpace::SRGB;
        ALICEVISION_LOG_INFO("Working color space automatically set to " << workingColorSpace << ".");
    }

    // Fusion always produces linear image. sRGB is the only non linear color space that must be changed to linear (sRGB
    // linear).
    image::EImageColorSpace mergedColorSpace =
      (workingColorSpace == image::EImageColorSpace::SRGB) ? image::EImageColorSpace::LINEAR : workingColorSpace;

    // Estimate target views for each group
    std::map<IndexT, std::vector<std::shared_ptr<sfmData::View>>> targetViewsPerIntrinsics;
    std::map<IndexT, int> targetIndexPerIntrinsics;
    if (!byPass)
    {
        for (const auto& intrinsicGroup : groupedViewsPerIntrinsics)
        {
            IndexT intrinsicId = intrinsicGroup.first;
            std::vector<std::vector<std::shared_ptr<sfmData::View>>> groups = intrinsicGroup.second;
            std::vector<std::shared_ptr<sfmData::View>> targetViews;

            const int middleIndex = usedNbBrackets / 2;
            const int targetIndex = middleIndex + offsetRefBracketIndex;
            const bool isOffsetRefBracketIndexValid = (targetIndex >= 0) && (targetIndex < usedNbBrackets);

            const fs::path lumaStatFilepath(fs::path(inputResponsePath).parent_path() /
                                            (std::string("luminanceStatistics") + "_" + std::to_string(intrinsicId) + ".txt"));

            if (!fs::is_regular_file(lumaStatFilepath) && !isOffsetRefBracketIndexValid)
            {
                ALICEVISION_LOG_ERROR("Unable to open the file '"
                                      << lumaStatFilepath.string()
                                      << "' with luminance "
                                         "statistics. This file is needed to select the optimal exposure for the creation "
                                         "of HDR images.");
                return EXIT_FAILURE;
            }

            // Adjust the targeted luminance level by removing the corresponding gamma if the working color space is not sRGB.
            if (workingColorSpace != image::EImageColorSpace::SRGB)
            {
                meanTargetedLumaForMerging = std::pow((meanTargetedLumaForMerging + 0.055) / 1.055, 2.2);
            }
            targetIndexPerIntrinsics[intrinsicId] =
              hdr::selectTargetViews(targetViews, groups, offsetRefBracketIndex, lumaStatFilepath.string(), meanTargetedLumaForMerging);

            if ((targetViews.empty() || targetViews.size() != groups.size()) && !isOffsetRefBracketIndexValid)
            {
                ALICEVISION_LOG_ERROR("File '" << lumaStatFilepath.string()
                                               << "' is not valid. This file is required "
                                                  "to select the optimal exposure for the creation of HDR images.");
                return EXIT_FAILURE;
            }

            targetViewsPerIntrinsics[intrinsicId] = targetViews;
        }
    }

    // Define range to compute
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0 || rangeStart > groupedViews.size())
        {
            ALICEVISION_LOG_ERROR("Range is incorrect.");
            return EXIT_FAILURE;
        }

        if (rangeStart + rangeSize > groupedViews.size())
        {
            rangeSize = groupedViews.size() - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = groupedViews.size();
    }
    ALICEVISION_LOG_DEBUG("Range to compute: rangeStart = " << rangeStart << ", rangeSize = " << rangeSize);

    if (rangeStart == 0)
    {
        int pos = 0;
        sfmData::SfMData outputSfm;
        outputSfm.getIntrinsics() = sfmData.getIntrinsics();

        // If we are on the first chunk, or we are computing all the dataset
        // Export a new sfmData with HDR images as new Views and LDR images as ancestors
        // ancestorIds are similar to viewIds because viewIds are computed only from image infos
        for (const auto& groupedViews : groupedViewsPerIntrinsics)
        {
            IndexT intrinsicId = groupedViews.first;

            const auto& groups = groupedViews.second;
            const auto& targetViews = targetViewsPerIntrinsics[intrinsicId];

            for (int g = 0; g < groups.size(); g++, pos++)
            {
                std::shared_ptr<sfmData::View> hdrView;

                const auto& group = groups[g];

                if (group.size() == 1)
                {
                    hdrView.reset(group.at(0)->clone());
                }
                else if (targetViews.empty())
                {
                    ALICEVISION_LOG_ERROR("Target view for HDR merging has not been computed.");
                    return EXIT_FAILURE;
                }
                else
                {
                    hdrView.reset(targetViews.at(g)->clone());
                }
                if (!byPass)
                {
                    fs::path p(targetViews[g]->getImage().getImagePath());
                    const std::string hdrImagePath = getHdrImagePath(outputPath, pos, keepSourceImageName ? p.stem().string() : "");
                    hdrView->getImage().setImagePath(hdrImagePath);
                }
                hdrView->getImage().addMetadata("AliceVision:ColorSpace", image::EImageColorSpace_enumToString(mergedColorSpace));
                outputSfm.getViews().emplace(hdrView->getViewId(), hdrView);

                for (const auto& v : group)
                {
                    outputSfm.addAncestor(v->getViewId(), v->getImageInfo());
                }
            }
        }

        // Export output SfMData
        if (!sfmDataIO::save(outputSfm, sfmOutputDataFilepath, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("Cannot save output SfMData file at '" << sfmOutputDataFilepath << "'.");
            return EXIT_FAILURE;
        }
    }

    if (byPass)
    {
        ALICEVISION_LOG_INFO("Bypass enabled, nothing to compute.");
        return EXIT_SUCCESS;
    }

    int rangeEnd = rangeStart + rangeSize;

    int pos = 0;
    for (const auto& pGroupedViews : groupedViewsPerIntrinsics)
    {
        IndexT intrinsicId = pGroupedViews.first;

        const auto& groupedViews = pGroupedViews.second;
        const auto& targetViews = targetViewsPerIntrinsics.at(intrinsicId);

        hdr::rgbCurve fusionWeight(channelQuantization);
        fusionWeight.setFunction(fusionWeightFunction);
        hdr::rgbCurve response(channelQuantization);

        const std::string baseName = (fs::path(inputResponsePath).parent_path() / std::string("response_")).string();
        const std::string intrinsicName = baseName + std::to_string(intrinsicId);
        const std::string intrinsicInputResponsePath = intrinsicName + ".csv";

        ALICEVISION_LOG_DEBUG("inputResponsePath: " << intrinsicInputResponsePath);
        response.read(intrinsicInputResponsePath);

        for (std::size_t g = 0; g < groupedViews.size(); ++g, ++pos)
        {
            if (pos < rangeStart || pos >= rangeEnd)
            {
                continue;
            }

            const std::vector<std::shared_ptr<sfmData::View>>& group = groupedViews[g];

            std::vector<image::Image<image::RGBfColor>> images(group.size());
            std::shared_ptr<sfmData::View> targetView = targetViews[g];
            std::vector<sfmData::ExposureSetting> exposuresSetting(group.size());

            // Load all images of the group
            for (std::size_t i = 0; i < group.size(); ++i)
            {
                const std::string filepath = group[i]->getImage().getImagePath();
                ALICEVISION_LOG_INFO("Load " << filepath);

                image::ImageReadOptions options;
                options.workingColorSpace = workingColorSpace;
                options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(group[i]->getImage().getRawColorInterpretation());
                options.colorProfileFileName = group[i]->getImage().getColorProfileFileName();

                // Whatever the raw color interpretation mode, the default read processing for raw images is to apply
                // white balancing in libRaw, before demosaicing.
                // The DcpMetadata mode allows to not apply color management after demosaicing.
                // Because if requested after demosaicing, white balancing is done at color management stage, we can
                // set this option to true to get real raw data, without any white balancing, when the DcpMetadata mode
                // is selected.
                if (options.rawColorInterpretation == image::ERawColorInterpretation::DcpMetadata)
                {
                    options.doWBAfterDemosaicing = true;
                }

                image::readImage(filepath, images[i], options);

                exposuresSetting[i] = group[i]->getImage().getCameraExposureSetting();
            }

            if (!sfmData::hasComparableExposures(exposuresSetting))
            {
                ALICEVISION_THROW_ERROR("Camera exposure settings are inconsistent.");
            }

            std::vector<double> exposures = getExposures(exposuresSetting);

            // Merge HDR images
            image::Image<image::RGBfColor> HDRimage;
            image::Image<image::RGBfColor> lowLightMask;
            image::Image<image::RGBfColor> highLightMask;
            image::Image<image::RGBfColor> noMidLightMask;
            if (images.size() > 1)
            {
                hdr::hdrMerge merge;
                sfmData::ExposureSetting targetCameraSetting = targetView->getImage().getCameraExposureSetting();
                hdr::MergingParams mergingParams;
                mergingParams.targetCameraExposure = targetCameraSetting.getExposure();
                mergingParams.refImageIndex = targetIndexPerIntrinsics[intrinsicId];
                mergingParams.minSignificantValue = minSignificantValue;
                mergingParams.maxSignificantValue = maxSignificantValue;
                mergingParams.computeLightMasks = computeLightMasks;

                merge.process(images, exposures, fusionWeight, response, HDRimage, lowLightMask, highLightMask, noMidLightMask, mergingParams);
                if (highlightCorrectionFactor > 0.0f)
                {
                    merge.postProcessHighlight(images,
                                               exposures,
                                               fusionWeight,
                                               response,
                                               HDRimage,
                                               targetCameraSetting.getExposure(),
                                               highlightCorrectionFactor,
                                               highlightTargetLux);
                }
            }
            else if (images.size() == 1)
            {
                // Nothing to do
                HDRimage = images[0];
            }

            fs::path p(targetView->getImage().getImagePath());
            const std::string hdrImagePath = getHdrImagePath(outputPath, pos, keepSourceImageName ? p.stem().string() : "");

            // Write an image with parameters from the target view
            std::map<std::string, std::string> viewMetadata = targetView->getImage().getMetadata();

            oiio::ParamValueList targetMetadata;
            for (const auto& meta : viewMetadata)
            {
                if (meta.first.compare(0, 3, "raw") == 0)
                {
                    targetMetadata.add_or_replace(oiio::ParamValue("AliceVision:" + meta.first, meta.second));
                }
                else
                {
                    targetMetadata.add_or_replace(oiio::ParamValue(meta.first, meta.second));
                }
            }

            targetMetadata.add_or_replace(oiio::ParamValue("AliceVision:ColorSpace", image::EImageColorSpace_enumToString(mergedColorSpace)));

            image::ImageWriteOptions writeOptions;
            writeOptions.fromColorSpace(mergedColorSpace);
            writeOptions.toColorSpace(mergedColorSpace);
            writeOptions.storageDataType(storageDataType);

            image::writeImage(hdrImagePath, HDRimage, writeOptions, targetMetadata);

            if (computeLightMasks)
            {
                const std::string hdrMaskLowLightPath = getHdrMaskPath(outputPath, pos, "lowLight", keepSourceImageName ? p.stem().string() : "");
                const std::string hdrMaskHighLightPath = getHdrMaskPath(outputPath, pos, "highLight", keepSourceImageName ? p.stem().string() : "");
                const std::string hdrMaskNoMidLightPath = getHdrMaskPath(outputPath, pos, "noMidLight", keepSourceImageName ? p.stem().string() : "");

                image::ImageWriteOptions maskWriteOptions;
                maskWriteOptions.exrCompressionMethod(image::EImageExrCompression::None);

                image::writeImage(hdrMaskLowLightPath, lowLightMask, maskWriteOptions);
                image::writeImage(hdrMaskHighLightPath, highLightMask, maskWriteOptions);
                image::writeImage(hdrMaskNoMidLightPath, noMidLightMask, maskWriteOptions);
            }
        }
    }

    return EXIT_SUCCESS;
}