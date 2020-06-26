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

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

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

    // Make sure there is only one kind of image in dataset
    if(sfmData.getIntrinsics().size() > 1)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
        return EXIT_FAILURE;
    }

    sfmData::Views& views = sfmData.getViews();

    // Order views by their image names (without path and extension to make sure we handle rotated images)
    std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
    for(auto& viewIt : sfmData.getViews())
    {
        viewsOrderedByName.push_back(viewIt.second);
    }
    std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(),
              [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
                  if(a == nullptr || b == nullptr)
                      return true;

                  boost::filesystem::path path_a(a->getImagePath());
                  boost::filesystem::path path_b(b->getImagePath());

                  return (path_a.stem().string() < path_b.stem().string());
              });

    {
        // Print a warning if the aperture changes.
        std::set<float> fnumbers;
        for(auto& view : viewsOrderedByName)
        {
            fnumbers.insert(view->getMetadataFNumber());
        }
        if(fnumbers.size() != 1)
        {
            ALICEVISION_LOG_WARNING("Different apertures amongst the dataset. For correct HDR, you should only change "
                                    "the shutter speed (and eventually the ISO).");
            ALICEVISION_LOG_WARNING("Used f-numbers:");
            for(auto f : fnumbers)
            {
                ALICEVISION_LOG_WARNING(" * " << f);
            }
        }
    }

    // Make groups
    std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
    {
        std::vector<std::shared_ptr<sfmData::View>> group;
        std::vector<float> exposures;
        for(auto& view : viewsOrderedByName)
        {
            if(nbBrackets > 0)
            {
                group.push_back(view);
                if(group.size() == nbBrackets)
                {
                    groupedViews.push_back(group);
                    group.clear();
                }
            }
            else
            {
                // Automatically determines the number of brackets
                float exp = view->getCameraExposureSetting();
                if(!exposures.empty() && exp != exposures.back() && exp == exposures.front())
                {
                    groupedViews.push_back(group);
                    group.clear();
                    exposures.clear();
                }
                exposures.push_back(exp);
                group.push_back(view);
            }
        }
        if(!group.empty())
            groupedViews.push_back(group);
    }

    if(nbBrackets <= 0)
    {
        std::set<std::size_t> sizeOfGroups;
        for(auto& group : groupedViews)
        {
            sizeOfGroups.insert(group.size());
        }
        if(sizeOfGroups.size() == 1)
        {
            ALICEVISION_LOG_INFO("Number of brackets automatically detected: "
                                 << *sizeOfGroups.begin() << ". It will generate " << groupedViews.size()
                                 << " hdr images.");
        }
        else
        {
            ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
        }
    }
    else if(nbBrackets == 1)
    {
        byPass = true;
    }

    std::vector<std::shared_ptr<sfmData::View>> targetViews;
    for(auto& group : groupedViews)
    {
        // Sort all images by exposure time
        std::sort(group.begin(), group.end(),
                  [](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
                      if(a == nullptr || b == nullptr)
                          return true;
                      return (a->getCameraExposureSetting() < b->getCameraExposureSetting());
                  });

        // Target views are the middle exposed views
        // For add number, there is no ambiguity on the middle image.
        // For even number, we arbitrarily choose the more exposed view.
        const int middleIndex = group.size() / 2;

        targetViews.push_back(group[middleIndex]);
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

    // If bypass, simply use central bracket
    if(byPass)
    {
        return EXIT_SUCCESS;
    }

    size_t channelQuantization = std::pow(2, channelQuantizationPower);
    

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
        bool res = hdr::extractSamples(out_samples, paths, exposures, channelQuantization);
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

    /*Store to file*/
    std::stringstream ss;
    ss << outputFolder << "/config.dat";
    std::ofstream file_config(ss.str(), std::ios::binary);
    if (!file_config.is_open()) {
        ALICEVISION_LOG_ERROR("Impossible to write config");
        return EXIT_FAILURE;
    }
    file_config.write((const char *)&nbBrackets, sizeof(nbBrackets));
    file_config.write((const char *)&byPass, sizeof(byPass));
    file_config.write((const char *)&channelQuantizationPower, sizeof(channelQuantizationPower));

    return EXIT_SUCCESS;
}
