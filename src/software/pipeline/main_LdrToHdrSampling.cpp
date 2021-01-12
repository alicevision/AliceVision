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

// SFMData
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/mvsData/jetColorMap.hpp>

#include <aliceVision/utils/Histogram.hpp>


// HDR Related
#include <aliceVision/hdr/sampling.hpp>
#include <aliceVision/hdr/brackets.hpp>

// Image Processing
#include <OpenImageIO/imagebufalgo.h>

// Command line parameters
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

#include <sstream>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int aliceVision_main(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename;
    std::string outputFolder;
    int nbBrackets = 0;
    int channelQuantizationPower = 10;
    hdr::Sampling::Params params;
    bool debug = false;

    int rangeStart = -1;
    int rangeSize = 1;

    // Command line parameters
    po::options_description allParams("Extract stable samples from multiple LDR images with different bracketing.\n"
                                      "AliceVision LdrToHdrSampling");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
        "Output path for the samples files.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets),
         "bracket count per HDR image (0 means automatic).")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.")
        ("blockSize", po::value<int>(&params.blockSize)->default_value(params.blockSize),
         "Size of the image tile to extract a sample.")
        ("radius", po::value<int>(&params.radius)->default_value(params.radius),
         "Radius of the patch used to analyze the sample statistics.")
        ("maxCountSample", po::value<size_t>(&params.maxCountSample)->default_value(params.maxCountSample),
         "Max number of samples per image group.")
        ("debug", po::value<bool>(&debug)->default_value(debug),
         "Export debug files.")
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

    const std::size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Make sure there is only one kind of image in dataset
    if(sfmData.getIntrinsics().size() > 1)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
        return EXIT_FAILURE;
    }

    const std::size_t width = sfmData.getIntrinsics().begin()->second->w();
    const std::size_t height = sfmData.getIntrinsics().begin()->second->h();

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, sfmData, nbBrackets))
    {
        return EXIT_FAILURE;
    }

    std::size_t usedNbBrackets = 0;
    {
        std::set<std::size_t> sizeOfGroups;
        for(auto& group : groupedViews)
        {
            sizeOfGroups.insert(group.size());
        }
        if(sizeOfGroups.size() == 1)
        {
            usedNbBrackets = *sizeOfGroups.begin();
            if(usedNbBrackets == 1)
            {
                ALICEVISION_LOG_INFO("No multi-bracketing.");
                return EXIT_SUCCESS;
            }
            ALICEVISION_LOG_INFO("Number of brackets: " << usedNbBrackets << ". It will generate " << groupedViews.size() << " hdr images.");
        }
        else
        {
            ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
            return EXIT_FAILURE;
        }
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

    for(std::size_t groupIdx = rangeStart; groupIdx < rangeStart + rangeSize; ++groupIdx)
    {
        auto & group = groupedViews[groupIdx];

        std::vector<std::string> paths;
        std::vector<float> exposures;

        bool applyWhiteBalance = true;

        for (auto & v : group)
        {
            paths.push_back(v->getImagePath());
            exposures.push_back(v->getCameraExposureSetting());

            applyWhiteBalance = applyWhiteBalance && v->getApplyWhiteBalance();
        }

        ALICEVISION_LOG_INFO("Extracting samples from group " << groupIdx);
        std::vector<hdr::ImageSample> out_samples;
        const bool res = hdr::Sampling::extractSamplesFromImages(out_samples, paths, exposures, width, height, channelQuantization, image::EImageColorSpace::SRGB, applyWhiteBalance, params);
        if (!res)
        {
            ALICEVISION_LOG_ERROR("Error while extracting samples from group " << groupIdx);
        }

        using namespace boost::accumulators;
        using Accumulator = accumulator_set<float, stats<tag::min, tag::max, tag::median, tag::mean>>;
        Accumulator acc_nbUsedBrackets;
        {
            utils::Histogram<int> histogram(1, usedNbBrackets, usedNbBrackets-1);
            for(const hdr::ImageSample& sample : out_samples)
            {
                acc_nbUsedBrackets(sample.descriptions.size());
                histogram.Add(sample.descriptions.size());
            }
            ALICEVISION_LOG_INFO("Number of used brackets in selected samples: "
                                 << " min: " << extract::min(acc_nbUsedBrackets) << " max: "
                                 << extract::max(acc_nbUsedBrackets) << " mean: " << extract::mean(acc_nbUsedBrackets)
                                 << " median: " << extract::median(acc_nbUsedBrackets));

            ALICEVISION_LOG_INFO("Histogram of the number of brackets per sample: " << histogram.ToString("", 2));
        }
        if(debug)
        {
            image::Image<image::RGBfColor> selectedPixels(width, height, true);

            for(const hdr::ImageSample& sample: out_samples)
            {
                const float score = float(sample.descriptions.size()) / float(usedNbBrackets);
                const ColorRGBf color = getColorFromJetColorMap(score);
                selectedPixels(sample.y, sample.x) = image::RGBfColor(color.r, color.g, color.b);
            }
            oiio::ParamValueList metadata;
            metadata.push_back(oiio::ParamValue("AliceVision:nbSelectedPixels", int(selectedPixels.size())));
            metadata.push_back(oiio::ParamValue("AliceVision:minNbUsedBrackets", extract::min(acc_nbUsedBrackets)));
            metadata.push_back(oiio::ParamValue("AliceVision:maxNbUsedBrackets", extract::max(acc_nbUsedBrackets)));
            metadata.push_back(oiio::ParamValue("AliceVision:meanNbUsedBrackets", extract::mean(acc_nbUsedBrackets)));
            metadata.push_back(oiio::ParamValue("AliceVision:medianNbUsedBrackets", extract::median(acc_nbUsedBrackets)));

            image::writeImage((fs::path(outputFolder) / (std::to_string(groupIdx) + "_selectedPixels.png")).string(),
                              selectedPixels, image::EImageColorSpace::AUTO, metadata);

        }

        // Store to file
        const std::string samplesFilepath = (fs::path(outputFolder) / (std::to_string(groupIdx) + "_samples.dat")).string();
        std::ofstream fileSamples(samplesFilepath, std::ios::binary);
        if (!fileSamples.is_open())
        {
            ALICEVISION_LOG_ERROR("Impossible to write samples");
            return EXIT_FAILURE;
        }

        const std::size_t size = out_samples.size();
        fileSamples.write((const char *)&size, sizeof(size));

        for(std::size_t i = 0; i < out_samples.size(); ++i)
        {
            fileSamples << out_samples[i];
        }
    }

    return EXIT_SUCCESS;
}
