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
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/GrossbergCalibrate.hpp>
#include <aliceVision/hdr/emorCurve.hpp>
#include <aliceVision/hdr/LaguerreBACalibration.hpp>
#include <aliceVision/hdr/sampling.hpp>
#include <aliceVision/hdr/brackets.hpp>

// Command line parameters
#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <sstream>

#include <fstream>
#include <map>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
using namespace aliceVision::hdr;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

struct luminanceInfo
{
    aliceVision::IndexT srcId;
    double meanLum;
    double minLum;
    double maxLum;
    int itemNb;
};

void computeLuminanceStatFromSamples(const std::vector<double>& groupedExposure, const std::vector<hdr::ImageSample>& samples, std::map<int, luminanceInfo>& luminanceInfos)
{
    for (int i = 0; i < groupedExposure.size(); i++)
    {
        luminanceInfos[(int)(groupedExposure[i] * 1000)].itemNb = 0;
        luminanceInfos[(int)(groupedExposure[i] * 1000)].meanLum = 0.0;
        luminanceInfos[(int)(groupedExposure[i] * 1000)].minLum = 1000.0;
        luminanceInfos[(int)(groupedExposure[i] * 1000)].maxLum = 0.0;
    }
    for (int i = 0; i < samples.size(); i++)
    {
        for (int j = 0; j < samples[i].descriptions.size(); j++)
        {
            double lum = image::Rgb2GrayLinear(samples[i].descriptions[j].mean[0], samples[i].descriptions[j].mean[1], samples[i].descriptions[j].mean[2]);
            luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].meanLum += lum;
            luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].itemNb++;
            if (lum < luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].minLum)
            {
                luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].minLum = lum;
            }
            if (lum > luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].maxLum)
            {
                luminanceInfos[(int)(samples[i].descriptions[j].exposure * 1000)].maxLum = lum;
            }
        }
    }
}

void computeLuminanceInfoFromImage(image::Image<image::RGBfColor>& image, luminanceInfo& lumaInfo)
{
    // Luminance statistics are calculated from a subsampled square, centered and rotated by 45°.
    // 2 vertices of this square are the centers of the longest sides of the image.
    // Such a shape is suitable for both fisheye and classic images.

    double meanLuminance = 0.0;
    double maxLuminance = 0.0;
    double minLuminance = 1000.0;
    int sampleNb = 0;

    const int imgH = image.Height();
    const int imgW = image.Width();

    const int a1 = (imgH <= imgW) ? imgW / 2 : imgH / 2;
    const int a2 = (imgH <= imgW) ? imgW / 2 : imgW - (imgH / 2);
    const int a3 = (imgH <= imgW) ? imgH - (imgW / 2) : imgH / 2;
    const int a4 = (imgH <= imgW) ? (imgW / 2) + imgH : imgW + (imgH / 2);

    // All rows must be considered if image orientation is landscape
    // Only imgW rows centered on imgH/2 must be considered if image orientation is portrait
    const int rmin = (imgH <= imgW) ? 0 : (imgH - imgW) / 2;
    const int rmax = (imgH <= imgW) ? imgH : (imgH + imgW) / 2;

    const int sampling = 16;

    for (int r = rmin; r < rmax; r = r + sampling)
    {
        const int cmin = (r < imgH / 2) ? a1 - r : r - a3;
        const int cmax = (r < imgH / 2) ? a2 + r : a4 - r;

        for (int c = cmin; c < cmax; c = c + sampling)
        {
            double luma = image::Rgb2GrayLinear(image(r, c)[0], image(r, c)[1], image(r, c)[2]);
            meanLuminance += luma;
            minLuminance = (luma < minLuminance) ? luma : minLuminance;
            maxLuminance = (luma > maxLuminance) ? luma : maxLuminance;
            sampleNb++;
        }
    }

    lumaInfo.itemNb = sampleNb;
    lumaInfo.minLum = minLuminance;
    lumaInfo.maxLum = maxLuminance;
    lumaInfo.meanLum = meanLuminance;
}

int aliceVision_main(int argc, char** argv)
{
    std::string sfmInputDataFilename;
    std::string samplesFolder;
    std::string outputResponsePath;
    ECalibrationMethod calibrationMethod = ECalibrationMethod::DEBEVEC;
    std::string calibrationWeightFunction = "default";
    int nbBrackets = 0;
    int channelQuantizationPower = 10;
    image::EImageColorSpace workingColorSpace = image::EImageColorSpace::SRGB;
    size_t maxTotalPoints = 1000000;
    bool byPass = false;

    // Command line parameters

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("response,o", po::value<std::string>(&outputResponsePath)->required(),
        "Output path for the response file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("samplesFolders,f", po::value<std::string>(&samplesFolder)->default_value(samplesFolder),
         "Path to folder containing the extracted samples (Required if the calibration is not linear).")
        ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod)->default_value(calibrationMethod),
         "Name of method used for camera calibration: linear, debevec, grossberg, laguerre.")
        ("calibrationWeight,w", po::value<std::string>(&calibrationWeightFunction)->default_value(calibrationWeightFunction),
         "Weight function used to calibrate camera response (default depends on the calibration method, gaussian, "
         "triangle, plateau).")
        ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets),
         "bracket count per HDR image (0 means automatic).")
        ("byPass", po::value<bool>(&byPass)->default_value(byPass),
         "bypass HDR creation and use a single bracket as input for next steps")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.")
        ("workingColorSpace", po::value<image::EImageColorSpace>(&workingColorSpace)->default_value(workingColorSpace),
         ("Working color space: " + image::EImageColorSpace_informations()).c_str())
        ("maxTotalPoints", po::value<size_t>(&maxTotalPoints)->default_value(maxTotalPoints),
            "Max number of points used from the sampling. This ensures that the number of pixels values extracted by the sampling "
            "can be managed by the calibration step (in term of computation time and memory usage).")
        ;

    CmdLine cmdline("This program recovers the Camera Response Function (CRF) from samples extracted from LDR images with multi-bracketing.\n"
                    "AliceVision LdrToHdrCalibration");
                  
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }
    if (channelQuantizationPower <= 0)
    {
        ALICEVISION_LOG_ERROR("Invalid channelQuantizationPower config");
        return EXIT_FAILURE;
    }
    if (nbBrackets < 0)
    {
        ALICEVISION_LOG_ERROR("Invalid nbBrackets config");
        return EXIT_FAILURE;
    }

    const size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, sfmData, nbBrackets))
    {
        ALICEVISION_LOG_ERROR("Error on brackets information");
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
                // Nothing to calibrate, export a linear CRF.
                calibrationMethod = ECalibrationMethod::LINEAR;
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

    std::vector<std::map<int, luminanceInfo>> v_luminanceInfos;
    std::vector<std::vector<hdr::ImageSample>> calibrationSamples;
    hdr::rgbCurve calibrationWeight(channelQuantization);
    std::vector<std::vector<double>> groupedExposures;

    if (samplesFolder.empty())
    {
        ALICEVISION_LOG_ERROR("A folder with selected samples is required to calibrate the Camera Response Function (CRF) and/or estimate the hdr output exposure level.");
        return EXIT_FAILURE;
    }
    else
    {
        // Build camera exposure table
        for (int i = 0; i < groupedViews.size(); ++i)
        {
            const std::vector<std::shared_ptr<sfmData::View>>& group = groupedViews[i];
            std::vector<sfmData::ExposureSetting> exposuresSetting;

            for (int j = 0; j < group.size(); ++j)
            {
                const sfmData::ExposureSetting exp = group[j]->getCameraExposureSetting();
                exposuresSetting.push_back(exp);
            }
            if (!sfmData::hasComparableExposures(exposuresSetting))
            {
                ALICEVISION_THROW_ERROR("Camera exposure settings are inconsistent.");
            }
            groupedExposures.push_back(getExposures(exposuresSetting));
        }

        size_t group_pos = 0;
        hdr::Sampling sampling;

        ALICEVISION_LOG_INFO("Analyzing samples for each group");
        for (auto& group : groupedViews)
        {
            // Read from file
            const std::string samplesFilepath = (fs::path(samplesFolder) / (std::to_string(group_pos) + "_samples.dat")).string();
            std::ifstream fileSamples(samplesFilepath, std::ios::binary);
            if (!fileSamples.is_open())
            {
                ALICEVISION_LOG_ERROR("Impossible to read samples from file " << samplesFilepath);
                return EXIT_FAILURE;
            }

            std::size_t size;
            fileSamples.read((char*)&size, sizeof(size));

            std::vector<hdr::ImageSample> samples(size);
            for (std::size_t i = 0; i < size; ++i)
            {
                fileSamples >> samples[i];
            }

            sampling.analyzeSource(samples, channelQuantization, group_pos);

            std::map<int, luminanceInfo> luminanceInfos;
            computeLuminanceStatFromSamples(groupedExposures[group_pos], samples, luminanceInfos);

            int v = 0;
            for (auto it = luminanceInfos.begin(); it != luminanceInfos.end(); it++)
            {
                if (v < group.size())
                {
                    (it->second).srcId = group[v]->getViewId();
                    v++;
                }
                else
                {
                    (it->second).srcId = 0;
                }
            }
            v_luminanceInfos.push_back(luminanceInfos);

            ++group_pos;
        }

        if (!byPass)
        {
            // We need to trim samples list
            sampling.filter(maxTotalPoints);

            ALICEVISION_LOG_INFO("Extracting samples for each group");
            group_pos = 0;

            std::size_t total = 0;
            for (auto& group : groupedViews)
            {
                // Read from file
                const std::string samplesFilepath = (fs::path(samplesFolder) / (std::to_string(group_pos) + "_samples.dat")).string();
                std::ifstream fileSamples(samplesFilepath, std::ios::binary);
                if (!fileSamples.is_open())
                {
                    ALICEVISION_LOG_ERROR("Impossible to read samples from file " << samplesFilepath);
                    return EXIT_FAILURE;
                }

                std::size_t size = 0;
                fileSamples.read((char*)&size, sizeof(size));

                std::vector<hdr::ImageSample> samples(size);
                for (int i = 0; i < size; ++i)
                {
                    fileSamples >> samples[i];
                }

                std::vector<hdr::ImageSample> out_samples;
                sampling.extractUsefulSamples(out_samples, samples, group_pos);

                calibrationSamples.push_back(out_samples);

                ++group_pos;
            }

            // Define calibration weighting curve from name
            boost::algorithm::to_lower(calibrationWeightFunction);
            if (calibrationWeightFunction == "default")
            {
                switch (calibrationMethod)
                {
                case ECalibrationMethod::DEBEVEC:
                    calibrationWeightFunction = hdr::EFunctionType_enumToString(hdr::EFunctionType::TRIANGLE);
                    break;
                case ECalibrationMethod::LINEAR:
                case ECalibrationMethod::GROSSBERG:
                case ECalibrationMethod::LAGUERRE:
                default:
                    calibrationWeightFunction = hdr::EFunctionType_enumToString(hdr::EFunctionType::GAUSSIAN);
                    break;
                }
            }
            calibrationWeight.setFunction(hdr::EFunctionType_stringToEnum(calibrationWeightFunction));
        }
    }

    if (!byPass)
    {
        ALICEVISION_LOG_INFO("Start calibration");
        hdr::rgbCurve response(channelQuantization);

        switch (calibrationMethod)
        {
        case ECalibrationMethod::LINEAR:
        {
            // set the response function to linear
            response.setLinear();
            break;
        }
        case ECalibrationMethod::DEBEVEC:
        {
            hdr::DebevecCalibrate debevec;
            const float lambda = channelQuantization;
            bool res = debevec.process(calibrationSamples, groupedExposures, channelQuantization, calibrationWeight, lambda, response);
            if (!res) {
                ALICEVISION_LOG_ERROR("Calibration failed");
                return EXIT_FAILURE;
            }

            response.exponential();
            response.scale();
            break;
        }
        case ECalibrationMethod::GROSSBERG:
        {
            hdr::GrossbergCalibrate calibration(5);
            calibration.process(calibrationSamples, groupedExposures, channelQuantization, response);
            break;
        }
        case ECalibrationMethod::LAGUERRE:
        {
            hdr::LaguerreBACalibration calibration;
            calibration.process(calibrationSamples, groupedExposures, channelQuantization, false, response);
            break;
        }
        }

        const std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
        const std::string htmlOutput = (fs::path(outputResponsePath).parent_path() / (std::string("response_") + methodName + std::string(".html"))).string();

        response.write(outputResponsePath);
        response.writeHtml(htmlOutput, "response");
    }

    const std::string lumastatFilename = (fs::path(outputResponsePath).parent_path() / "luminanceStatistics.txt").string();
    std::ofstream file(lumastatFilename);
    if (!file)
    {
        ALICEVISION_LOG_ERROR("Unable to create file " << lumastatFilename << " for storing luminance statistics");
        return EXIT_FAILURE;
    }

    file << v_luminanceInfos.size() << std::endl;
    if (!v_luminanceInfos.empty())
    {
        file << v_luminanceInfos[0].size() << std::endl;
        file << "# viewId ; exposure ; sampleNumber ; meanLuminance ; minLuminance ; maxLuminance" << std::endl;

        for (int i = 0; i < v_luminanceInfos.size(); ++i)
        {
            for (auto it = v_luminanceInfos[i].begin(); it != v_luminanceInfos[i].end(); it++)
            {
                file << (it->second).srcId << " ";
                file << it->first << " " << (it->second).itemNb << " " << (it->second).meanLum / (it->second).itemNb << " ";
                file << (it->second).minLum << " " << (it->second).maxLum << std::endl;
            }
        }
    }

    return EXIT_SUCCESS;
}
