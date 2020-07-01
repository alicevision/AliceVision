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

/*SFMData*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/*HDR Related*/
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/brackets.hpp>

/*Command line parameters*/
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

int aliceVision_main(int argc, char* argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename = "";
    std::string inputResponsePath = "";
    std::string sfmOutputDataFilename;
    int nbBrackets = 3;
    bool byPass = false;
    int channelQuantizationPower = 10;

    hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
    float highlightCorrectionFactor = 1.0f;
    float highlightTargetLux = 120000.0f;

    // Command line parameters
    po::options_description allParams("Merge LDR images into HDR images.\n"
                                      "AliceVision LdrToHdrMerge");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("response,o", po::value<std::string>(&inputResponsePath)->required(),
        "Input path for the response file.")
        ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(),
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
        ("highlightTargetLux", po::value<float>(&highlightTargetLux)->default_value(highlightTargetLux),
         "Highlights maximum luminance.")
        ("highlightCorrectionFactor", po::value<float>(&highlightCorrectionFactor)->default_value(highlightCorrectionFactor),
         "float value between 0 and 1 to correct clamped highlights in dynamic range: use 0 for no correction, 1 for "
         "full correction to maxLuminance.");

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
    boost::filesystem::path path(sfmOutputDataFilename);
    std::string outputPath = path.parent_path().string();

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    /** Check input compatibility with brackets */
    size_t countImages = sfmData.getViews().size();
    if(countImages == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData contains no input !");
        return EXIT_FAILURE;
    }
    if(nbBrackets > 0 && countImages % nbBrackets != 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData file is not compatible with the number of brackets.");
        return EXIT_FAILURE;
    }

    size_t channelQuantization = std::pow(2, channelQuantizationPower);

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    std::vector<std::shared_ptr<sfmData::View>> targetViews;
    if (!hdr::estimateBracketsFromSfmData(groupedViews, targetViews, sfmData, nbBrackets)) {
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

    // Build table of file names
    std::vector<std::vector<std::string>> groupedFilenames;
    for(int i = 0; i < groupedViews.size(); i++)
    {
        const std::vector<std::shared_ptr<sfmData::View>>& group = groupedViews[i];

        std::vector<std::string> filenames;

        for(int j = 0; j < group.size(); j++)
        {
            filenames.push_back(group[j]->getImagePath());
        }

        groupedFilenames.push_back(filenames);
    }    

    sfmData::SfMData outputSfm;
    sfmData::Views& vs = outputSfm.getViews();
    outputSfm.getIntrinsics() = sfmData.getIntrinsics();

    hdr::rgbCurve fusionWeight(channelQuantization);
    fusionWeight.setFunction(fusionWeightFunction);

    hdr::rgbCurve response(channelQuantization);
    std::cout << inputResponsePath << std::endl;
    response.read(inputResponsePath);


    for(int g = 0; g < groupedFilenames.size(); ++g)
    {
        std::vector<image::Image<image::RGBfColor>> images(groupedViews[g].size());
        std::shared_ptr<sfmData::View> targetView = targetViews[g];

        // Load all images of the group
        for(int i = 0; i < images.size(); i++)
        {
            ALICEVISION_LOG_INFO("Load " << groupedFilenames[g][i]);
            image::readImage(groupedFilenames[g][i], images[i], image::EImageColorSpace::SRGB);
        }

        // Merge HDR images
        hdr::hdrMerge merge;
        float targetCameraExposure = targetView->getCameraExposureSetting();
        image::Image<image::RGBfColor> HDRimage;
        merge.process(images, groupedExposures[g], fusionWeight, response, HDRimage, targetCameraExposure);
        if(highlightCorrectionFactor > 0.0)
        {
            merge.postProcessHighlight(images, groupedExposures[g], fusionWeight, response, HDRimage, targetCameraExposure, highlightCorrectionFactor, highlightTargetLux);
        }

        // Output image file path
        std::stringstream sstream;
        sstream << "hdr_" << std::setfill('0') << std::setw(4) << g << ".exr";
        std::string hdrImagePath = (fs::path(outputPath) / sstream.str()).string();

        // Write an image with parameters from the target view
        oiio::ParamValueList targetMetadata = image::readImageMetadata(targetView->getImagePath());
        image::writeImage(hdrImagePath, HDRimage, image::EImageColorSpace::AUTO, targetMetadata);

        targetViews[g]->setImagePath(hdrImagePath);
        vs[targetViews[g]->getViewId()] = targetViews[g];
    }

    // Export output sfmData
    if(!sfmDataIO::Save(outputSfm, sfmOutputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
