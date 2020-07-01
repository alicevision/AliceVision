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
#include <aliceVision/hdr/sampling.hpp>
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
    std::string outputFolder = "";
    int nbBrackets = 0;
    int channelQuantizationPower = 10;
    bool byPass = false;

    // Command line parameters
    po::options_description allParams("Parse external information about cameras used in a panorama.\n"
                                      "AliceVision PanoramaExternalInfo");

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

    
    size_t channelQuantization = std::pow(2, channelQuantizationPower);
    
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

    size_t width = sfmData.getIntrinsics().begin()->second->w();
    size_t height = sfmData.getIntrinsics().begin()->second->h();

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


    
    
    size_t group_pos = 0;
    for(auto & group : groupedViews) {

        
        std::vector<std::string> paths;
        std::vector<float> exposures;

        for (auto & v : group) {

            paths.push_back(v->getImagePath());
            exposures.push_back(v->getCameraExposureSetting());
        }

        ALICEVISION_LOG_INFO("Extracting sample from group " << group_pos);
        std::vector<hdr::ImageSample> out_samples;
        bool res = hdr::Sampling::extractSamplesFromImages(out_samples, paths, exposures, width, height, channelQuantization, image::EImageColorSpace::SRGB);
        if (!res) {
            ALICEVISION_LOG_ERROR("Error while extracting samples from group " << group_pos);
        }

        /*Store to file*/
        std::stringstream ss;
        ss << outputFolder << "/samples_" << group_pos << ".dat";
        std::ofstream file_samples(ss.str(), std::ios::binary);
        if (!file_samples.is_open()) {
            ALICEVISION_LOG_ERROR("Impossible to write samples");
            return EXIT_FAILURE;
        }
        
        size_t size = out_samples.size();
        file_samples.write((const char *)&size, sizeof(size));

        for (int i = 0; i < out_samples.size(); i++) {
            file_samples << out_samples[i];
        }

        group_pos++;
    }

    return EXIT_SUCCESS;
}
