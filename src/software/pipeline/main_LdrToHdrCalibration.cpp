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

enum class ECalibrationMethod
{
    LINEAR,
    DEBEVEC,
    GROSSBERG,
    LAGUERRE,
};

/**
 * @brief convert an enum ECalibrationMethod to its corresponding string
 * @param ECalibrationMethod
 * @return String
 */
inline std::string ECalibrationMethod_enumToString(const ECalibrationMethod calibrationMethod)
{
    switch(calibrationMethod)
    {
        case ECalibrationMethod::LINEAR:
            return "linear";
        case ECalibrationMethod::DEBEVEC:
            return "debevec";
        case ECalibrationMethod::GROSSBERG:
            return "grossberg";
        case ECalibrationMethod::LAGUERRE:
            return "laguerre";
    }
    throw std::out_of_range("Invalid method name enum");
}

/**
 * @brief convert a string calibration method name to its corresponding enum ECalibrationMethod
 * @param ECalibrationMethod
 * @return String
 */
inline ECalibrationMethod ECalibrationMethod_stringToEnum(const std::string& calibrationMethodName)
{
    std::string methodName = calibrationMethodName;
    std::transform(methodName.begin(), methodName.end(), methodName.begin(), ::tolower);

    if(methodName == "linear")
        return ECalibrationMethod::LINEAR;
    if(methodName == "debevec")
        return ECalibrationMethod::DEBEVEC;
    if(methodName == "grossberg")
        return ECalibrationMethod::GROSSBERG;
    if(methodName == "laguerre")
        return ECalibrationMethod::LAGUERRE;

    throw std::out_of_range("Invalid method name : '" + calibrationMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ECalibrationMethod calibrationMethodName)
{
    os << ECalibrationMethod_enumToString(calibrationMethodName);
    return os;
}

inline std::istream& operator>>(std::istream& in, ECalibrationMethod& calibrationMethod)
{
    std::string token;
    in >> token;
    calibrationMethod = ECalibrationMethod_stringToEnum(token);
    return in;
}

int aliceVision_main(int argc, char** argv)
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename;
    std::string samplesFolder;
    std::string outputResponsePath;
    ECalibrationMethod calibrationMethod = ECalibrationMethod::LINEAR;
    std::string calibrationWeightFunction = "default";
    int nbBrackets = 0;
    int channelQuantizationPower = 10;
    bool byPass = false;

    // Command line parameters
    po::options_description allParams("Recover the Camera Response Function (CRF) from samples extracted from LDR images with multi-bracking.\n"
                                      "AliceVision LdrToHdrCalibration");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("samplesFolders,f", po::value<std::string>(&samplesFolder)->required(),
        "Path to folder containing the extracted samples.")
        ("response,o", po::value<std::string>(&outputResponsePath)->required(),
        "Output path for the response file.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod)->default_value(calibrationMethod),
         "Name of method used for camera calibration: linear, debevec, grossberg, laguerre.")
        ("calibrationWeight,w", po::value<std::string>(&calibrationWeightFunction)->default_value(calibrationWeightFunction),
         "Weight function used to calibrate camera response (default depends on the calibration method, gaussian, "
        "triangle, plateau).")
        ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets),
         "bracket count per HDR image (0 means automatic).")
        ("byPass", po::value<bool>(&byPass)->default_value(byPass),
         "bypass HDR creation and use medium bracket as input for next steps")
        ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
         "Quantization level like 8 bits or 10 bits.");

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
    std::vector<std::shared_ptr<sfmData::View>> targetViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, targetViews, sfmData, nbBrackets))
    {
        ALICEVISION_LOG_ERROR("Error on brackets information");
        return EXIT_FAILURE;
    }

    // Build camera exposure table
    std::vector<std::vector<float>> groupedExposures;
    for(int i = 0; i < groupedViews.size(); i++)
    {
        const std::vector<std::shared_ptr<sfmData::View>>& group = groupedViews[i];
        std::vector<float> exposures;

        for(int j = 0; j < group.size(); j++)
        {
            float etime = group[j]->getCameraExposureSetting();
            exposures.push_back(etime);
        }
        groupedExposures.push_back(exposures);
    }

    size_t group_pos = 0;
    hdr::Sampling sampling;

    ALICEVISION_LOG_INFO("Analyzing samples for each group");
    for(auto & group : groupedViews)
    {
        // Read from file
        std::stringstream ss;
        ss << samplesFolder << "/samples_" << group_pos << ".dat";
        std::ifstream file_samples(ss.str(), std::ios::binary);
        if (!file_samples.is_open())
        {
            ALICEVISION_LOG_ERROR("Impossible to read samples from file " << ss.str());
            return EXIT_FAILURE;
        }

        std::size_t size;
        file_samples.read((char *)&size, sizeof(size));

        std::vector<hdr::ImageSample> samples(size);
        for (std::size_t i = 0; i < size; ++i)
        {
            file_samples >> samples[i];
        }

        sampling.analyzeSource(samples, channelQuantization, group_pos);

        ++group_pos;
    }

    // We need to trim samples list
    sampling.filter(1000000);
    
    ALICEVISION_LOG_INFO("Extracting samples for each group");
    group_pos = 0;

    size_t total = 0;
    std::vector<std::vector<hdr::ImageSample>> calibration_samples;
    for(auto & group : groupedViews)
    {
        // Read from file
        std::stringstream ss;
        ss << samplesFolder << "/samples_" << group_pos << ".dat";
        std::ifstream file_samples(ss.str(), std::ios::binary);
        if (!file_samples.is_open())
        {
            ALICEVISION_LOG_ERROR("Impossible to read samples from file " << ss.str());
            return EXIT_FAILURE;
        }

        size_t size; 
        file_samples.read((char *)&size, sizeof(size));

        std::vector<hdr::ImageSample> samples(size);
        for (int i = 0; i < size; i++)
        {
            file_samples >> samples[i];
        }

        std::vector<hdr::ImageSample> out_samples;
        sampling.extractUsefulSamples(out_samples, samples, group_pos); 

        calibration_samples.push_back(out_samples);

        ++group_pos;
    }

    // Define calibration weighting curve from name
    hdr::rgbCurve calibrationWeight(channelQuantization);
    std::transform(calibrationWeightFunction.begin(), calibrationWeightFunction.end(), calibrationWeightFunction.begin(), ::tolower);
    std::string calibrationWeightFunctionV = calibrationWeightFunction;
    if(calibrationWeightFunction == "default")
    {
        switch(calibrationMethod)
        {
            case ECalibrationMethod::LINEAR:
                break;
            case ECalibrationMethod::DEBEVEC:
                calibrationWeight.setTriangular();
                calibrationWeightFunctionV = "triangular";
                break;
            case ECalibrationMethod::GROSSBERG:
                break;
            case ECalibrationMethod::LAGUERRE:
                break;
        }
    }
    else
    {
        calibrationWeight.setFunction(hdr::EFunctionType_stringToEnum(calibrationWeightFunction));
    }

    ALICEVISION_LOG_INFO("Start calibration");
    hdr::rgbCurve response(channelQuantization);

    switch(calibrationMethod)
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
            bool res = debevec.process(calibration_samples, groupedExposures, channelQuantization, calibrationWeight, lambda, response);
            if (!res) {
                ALICEVISION_LOG_ERROR("Calibration failed");
                return EXIT_FAILURE;
            }

            response.exponential();
            break;
        }
        case ECalibrationMethod::GROSSBERG:
        {
            hdr::GrossbergCalibrate calibration(5);
            calibration.process(calibration_samples, groupedExposures, channelQuantization, response);
            break;
        }
        case ECalibrationMethod::LAGUERRE:
        {
            hdr::LaguerreBACalibration calibration;
            calibration.process(calibration_samples, groupedExposures, channelQuantization, false, response);
            break;
        }
    }

    const std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
    const std::string htmlOutput = (fs::path(outputResponsePath).parent_path() / (std::string("response_") + methodName + std::string(".html"))).string();

    response.write(outputResponsePath);
    response.writeHtml(htmlOutput, "response");

    return EXIT_SUCCESS;
}
