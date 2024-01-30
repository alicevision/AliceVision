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

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/image/jetColorMap.hpp>

#include <aliceVision/utils/Histogram.hpp>

// HDR Related
#include <aliceVision/hdr/sampling.hpp>
#include <aliceVision/hdr/brackets.hpp>

// Image Processing
#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebufalgo.h>

// Command line parameters
#include <boost/program_options.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <sstream>
#include <filesystem>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
using namespace aliceVision::hdr;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string sfmInputDataFilename;
    std::string outputFolder;
    int nbBrackets = 0;
    int channelQuantizationPower = 10;
    ECalibrationMethod calibrationMethod = ECalibrationMethod::AUTO;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::AUTO;
    hdr::Sampling::Params params;
    bool byPass = false;
    bool debug = false;

    int rangeStart = -1;
    int rangeSize = 1;

    // Command line parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Output path for the samples files.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets),
         "Bracket count per HDR image (0 means automatic).")
        ("byPass", po::value<bool>(&byPass)->default_value(byPass),
         "Bypass HDR creation and use a single bracket as the input for the next steps.")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.")
        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())
        ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod)->default_value(calibrationMethod),
         "Name of the method used for the camera calibration: auto, linear, debevec, grossberg, laguerre."
         "If 'auto' is selected, the linear method will be used if there are RAW images; otherwise, the Debevec method "
         "will be used.")
        ("blockSize", po::value<int>(&params.blockSize)->default_value(params.blockSize),
         "Size of the image tile to extract a sample.")
        ("radius", po::value<int>(&params.radius)->default_value(params.radius),
         "Radius of the patch used to analyze the sample statistics.")
        ("maxCountSample", po::value<std::size_t>(&params.maxCountSample)->default_value(params.maxCountSample),
         "Maximum number of samples per image group.")
        ("debug", po::value<bool>(&debug)->default_value(debug),
         "Export debug files.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
          "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
          "Range size.");
    // clang-format on

    CmdLine cmdline("This program extracts stable samples from multiple LDR images with different bracketing.\n"
                    "AliceVision LdrToHdrSampling");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    const std::size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Read SfMData
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, sfmData, nbBrackets))
    {
        ALICEVISION_LOG_ERROR("Failure to estimate brackets.");
        return EXIT_FAILURE;
    }

    std::size_t usedNbBrackets = 0;
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
                return EXIT_SUCCESS;
            }
            ALICEVISION_LOG_INFO("Number of brackets: " << usedNbBrackets << ". It will generate " << groupedViews.size() << " HDR images.");
        }
        else
        {
            ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
            for (auto& group : groupedViews)
            {
                ALICEVISION_LOG_ERROR(" * " << group.size());
            }
            return EXIT_FAILURE;
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

    std::map<IndexT, std::vector<std::vector<std::shared_ptr<sfmData::View>>>> groupedViewsPerIntrinsics;
    for (std::size_t groupIdx = rangeStart; groupIdx < rangeStart + rangeSize; ++groupIdx)
    {
        auto& group = groupedViews[groupIdx];

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

    for (const auto& pGroups : groupedViewsPerIntrinsics)
    {
        IndexT intrinsicId = pGroups.first;

        const auto& intrinsic = sfmData.getIntrinsics().at(intrinsicId);
        const std::size_t width = intrinsic->w();
        const std::size_t height = intrinsic->h();

        for (const auto& group : pGroups.second)
        {
            std::vector<std::string> paths;
            std::vector<sfmData::ExposureSetting> exposuresSetting;
            std::vector<IndexT> viewIds;

            image::ERawColorInterpretation rawColorInterpretation = image::ERawColorInterpretation::LibRawWhiteBalancing;
            std::string colorProfileFileName = "";

            bool first = true;
            IndexT firstViewId = UndefinedIndexT;

            for (auto& v : group)
            {
                // Retrieve first ViewId to get a unique name for files as one view is only in one group
                if (first)
                {
                    firstViewId = v->getViewId();
                    first = false;

                    const bool isRAW = image::isRawFormat(v->getImage().getImagePath());

                    if (calibrationMethod == ECalibrationMethod::AUTO)
                    {
                        calibrationMethod = isRAW ? ECalibrationMethod::LINEAR : ECalibrationMethod::DEBEVEC;
                        ALICEVISION_LOG_INFO("Calibration method automatically set to " << calibrationMethod);
                    }
                    if (workingColorSpace == image::EImageColorSpace::AUTO)
                    {
                        workingColorSpace = isRAW ? image::EImageColorSpace::LINEAR : image::EImageColorSpace::SRGB;
                        ALICEVISION_LOG_INFO("Working color space automatically set to " << workingColorSpace);
                    }
                }

                paths.push_back(v->getImage().getImagePath());
                exposuresSetting.push_back(v->getImage().getCameraExposureSetting());
                viewIds.push_back(v->getViewId());

                const std::string rawColorInterpretation_str = v->getImage().getRawColorInterpretation();
                rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(rawColorInterpretation_str);
                colorProfileFileName = v->getImage().getColorProfileFileName();

                ALICEVISION_LOG_INFO("Image: " << paths.back() << ", exposure: " << exposuresSetting.back()
                                               << ", raw color interpretation: " << ERawColorInterpretation_enumToString(rawColorInterpretation));
            }
            if (!sfmData::hasComparableExposures(exposuresSetting))
            {
                ALICEVISION_THROW_ERROR("Camera exposure settings are inconsistent.");
            }
            std::vector<double> exposures = getExposures(exposuresSetting);

            image::ImageReadOptions imgReadOptions;
            imgReadOptions.workingColorSpace = workingColorSpace;
            imgReadOptions.rawColorInterpretation = rawColorInterpretation;
            imgReadOptions.colorProfileFileName = colorProfileFileName;

            const bool simplifiedSampling = byPass || (calibrationMethod == ECalibrationMethod::LINEAR);

            std::vector<hdr::ImageSample> out_samples;
            const bool res = hdr::Sampling::extractSamplesFromImages(
              out_samples, paths, viewIds, exposures, width, height, channelQuantization, imgReadOptions, params, simplifiedSampling);
            if (!res)
            {
                ALICEVISION_LOG_ERROR("Error while extracting samples from group.");
            }

            using namespace boost::accumulators;
            using Accumulator = accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean>>;
            Accumulator acc_nbUsedBrackets;
            {
                utils::Histogram<int> histogram(1, usedNbBrackets, usedNbBrackets - 1);
                for (const hdr::ImageSample& sample : out_samples)
                {
                    acc_nbUsedBrackets(sample.descriptions.size());
                    histogram.Add(sample.descriptions.size());
                }
                ALICEVISION_LOG_INFO("Number of used brackets in selected samples: "
                                     << " min: " << extract::min(acc_nbUsedBrackets) << " max: " << extract::max(acc_nbUsedBrackets)
                                     << " mean: " << extract::mean(acc_nbUsedBrackets) << " median: " << extract::median(acc_nbUsedBrackets) << ".");

                ALICEVISION_LOG_INFO("Histogram of the number of brackets per sample: " << histogram.ToString("", 2) << ".");
            }
            if (debug)
            {
                image::Image<image::RGBfColor> selectedPixels(width, height, true);

                for (const hdr::ImageSample& sample : out_samples)
                {
                    const float score = float(sample.descriptions.size()) / float(usedNbBrackets);
                    const image::RGBfColor color = getColorFromJetColorMap(score);
                    selectedPixels(sample.y, sample.x) = image::RGBfColor(color.r(), color.g(), color.b());
                }
                oiio::ParamValueList metadata;
                metadata.push_back(oiio::ParamValue("AliceVision:nbSelectedPixels", int(selectedPixels.size())));
                metadata.push_back(oiio::ParamValue("AliceVision:minNbUsedBrackets", extract::min(acc_nbUsedBrackets)));
                metadata.push_back(oiio::ParamValue("AliceVision:maxNbUsedBrackets", extract::max(acc_nbUsedBrackets)));
                metadata.push_back(oiio::ParamValue("AliceVision:meanNbUsedBrackets", extract::mean(acc_nbUsedBrackets)));
                metadata.push_back(oiio::ParamValue("AliceVision:medianNbUsedBrackets", extract::median(acc_nbUsedBrackets)));

                image::writeImage((fs::path(outputFolder) / (std::to_string(firstViewId) + "_selectedPixels.png")).string(),
                                  selectedPixels,
                                  image::ImageWriteOptions(),
                                  metadata);
            }

            // Store to file
            const std::string samplesFilepath = (fs::path(outputFolder) / (std::to_string(firstViewId) + "_samples.dat")).string();
            std::ofstream fileSamples(samplesFilepath, std::ios::binary);
            if (!fileSamples.is_open())
            {
                ALICEVISION_LOG_ERROR("Cannot write samples.");
                return EXIT_FAILURE;
            }

            const std::size_t size = out_samples.size();
            fileSamples.write((const char*)&size, sizeof(size));

            for (std::size_t i = 0; i < out_samples.size(); ++i)
            {
                fileSamples << out_samples[i];
            }
        }
    }

    return EXIT_SUCCESS;
}
