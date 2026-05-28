// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <set>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string sfmDataFilenameA;
    std::string sfmDataFilenameB;
    std::string outSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Path to the main SfMData file (content source: views, poses, intrinsics, landmarks).")
        ("inputA,a", po::value<std::string>(&sfmDataFilenameA)->required(),
         "Path to SfMData file A. Only its view IDs are used to define camera group A.")
        ("inputB,b", po::value<std::string>(&sfmDataFilenameB)->required(),
         "Path to SfMData file B. Only its view IDs are used to define camera group B.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision sfmDataIntersection");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load the main input scene (content source)
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Load inputA and inputB — only their view IDs are needed
    sfmData::SfMData sfmDataA;
    if (!sfmDataIO::load(sfmDataA, sfmDataFilenameA, sfmDataIO::ESfMData::VIEWS))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file A '" << sfmDataFilenameA << "' cannot be read");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmDataB;
    if (!sfmDataIO::load(sfmDataB, sfmDataFilenameB, sfmDataIO::ESfMData::VIEWS))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file B '" << sfmDataFilenameB << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Build sets of view IDs from inputA and inputB
    std::set<IndexT> viewsSetA;
    for (const auto& [viewId, _] : sfmDataA.getViews())
    {
        viewsSetA.insert(viewId);
    }

    std::set<IndexT> viewsSetB;
    for (const auto& [viewId, _] : sfmDataB.getViews())
    {
        viewsSetB.insert(viewId);
    }

    ALICEVISION_LOG_INFO("Input SfMData: " << sfmData.getViews().size() << " views, "
                                           << sfmData.getLandmarks().size() << " landmarks");
    ALICEVISION_LOG_INFO("InputA: " << viewsSetA.size() << " views");
    ALICEVISION_LOG_INFO("InputB: " << viewsSetB.size() << " views");

    // The output is the main input SfMData with landmarks filtered
    sfmData::SfMData outputSfmData = sfmData;

    ALICEVISION_LOG_INFO("Total landmarks before filtering: " << outputSfmData.getLandmarks().size());

    // Filter landmarks: keep only those with observations from BOTH camera groups
    auto& landmarks = outputSfmData.getLandmarks();
    for (auto it = landmarks.begin(); it != landmarks.end();)
    {
        bool hasObsFromSetA = false;
        bool hasObsFromSetB = false;

        for (const auto& [viewId, obs] : it->second.getObservations())
        {
            if (viewsSetA.count(viewId))
            {
                hasObsFromSetA = true;
            }
            if (viewsSetB.count(viewId))
            {
                hasObsFromSetB = true;
            }

            if (hasObsFromSetA && hasObsFromSetB)
            {
                break;
            }
        }

        if (hasObsFromSetA && hasObsFromSetB)
        {
            ++it;
        }
        else
        {
            it = landmarks.erase(it);
        }
    }

    ALICEVISION_LOG_INFO("Total landmarks after filtering: " << outputSfmData.getLandmarks().size());

    if (!sfmDataIO::save(outputSfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
