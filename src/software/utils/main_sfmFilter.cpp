// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/stl/regex.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    system::Timer timer;

    // command-line parameters
    std::string sfmDataFilename;
    std::string outputSfMFilenameSelected;
    std::string outputSfMFilenameUnselected;
    std::string fileMatchingPattern;
    bool prunePoses;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputFile,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Path to the input SfMData file.\n")
        ("fileMatchingPattern,m", po::value<std::string>(&fileMatchingPattern)->required(),
         "Matching pattern for the from_filepath method.\n")
        ("prunePoses,p", po::value<bool>(&prunePoses)->required(), "Prune unselected poses.\n")
        ("outputSfMData_selected,o", po::value<std::string>(&outputSfMFilenameSelected)->required(),
         "Path to the output SfMData file.\n")
         ("outputSfMData_unselected,o", po::value<std::string>(&outputSfMFilenameUnselected)->required(),
         "Path to the output SfMData file.\n");
    // clang-format on
    
    CmdLine cmdline("AliceVision sfmFilter");
    cmdline.add(requiredParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    std::regex re(fileMatchingPattern);
    std::vector<IndexT> selectedViews;
    std::vector<IndexT> selectedPoses;

    for (auto& viewIt : sfmData.getViews())
    {
        const std::string& imagePath = viewIt.second->getImage().getImagePath();
        std::smatch matches;
        if (std::regex_search(imagePath, matches, re))
        {
            selectedViews.push_back(viewIt.first);
            if (prunePoses)
                selectedPoses.push_back(viewIt.second->getPoseId());
        }
    }

    // Split views into the selected and unselected SfmData

    sfmData::SfMData outputSfMData_selected = sfmData;
    sfmData::SfMData outputSfMData_unselected = sfmData;
    std::set<IndexT> viewIdsToRemove;
    std::set<IndexT> viewIdsToKeep;

    for (auto& viewIt : outputSfMData_selected.getViews())
    {
        const IndexT viewId = viewIt.first;
        auto it = std::find(selectedViews.begin(), selectedViews.end(), viewId);
        if (it == selectedViews.end())
        {
            viewIdsToRemove.insert(viewId);
        }
        else
        {
            viewIdsToKeep.insert(viewId);
        }
    }

    for (auto r : viewIdsToRemove)
    {
        outputSfMData_selected.getViews().erase(r);
    }

    for (auto r : viewIdsToKeep)
    {
        outputSfMData_unselected.getViews().erase(r);
    }

    // Split poses into the selected and unselected SfmData

    if (prunePoses)
    {
        std::set<IndexT> poseIdsToRemove;
        std::set<IndexT> poseIdsToKeep;

        for (auto& viewIt : outputSfMData_selected.getPoses())
        {
            const IndexT viewId = viewIt.first;
            auto it = std::find(selectedPoses.begin(), selectedPoses.end(), viewId);
            if (it == selectedPoses.end())
            {
                poseIdsToRemove.insert(viewId);
            }
            else
            {
                poseIdsToKeep.insert(viewId);
            }
        }

        for (auto r : poseIdsToRemove)
        {
            outputSfMData_selected.getPoses().erase(r);
        }

        for (auto r : poseIdsToKeep)
        {
            outputSfMData_unselected.getPoses().erase(r);
        }
    }


    ALICEVISION_LOG_INFO("Save into '" << outputSfMFilenameSelected << "'");
    // Export the SfMData scene in the expected format
    if (!sfmDataIO::save(outputSfMData_selected, outputSfMFilenameSelected, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outputSfMFilenameSelected << "'");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Save into '" << outputSfMFilenameUnselected << "'");
    // Export the SfMData scene in the expected format
    if (!sfmDataIO::save(outputSfMData_unselected, outputSfMFilenameUnselected, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outputSfMFilenameUnselected << "'");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
