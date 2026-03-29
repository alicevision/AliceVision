// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> inputFiles;
    std::string outputFile;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputs,i", po::value<std::vector<std::string>>(&inputFiles)->multitoken()->required(),
         "List of input image pairs files.")
        ("output,o", po::value<std::string>(&outputFile)->required(),
         "Output image pairs file.");
    // clang-format on

    CmdLine cmdline("AliceVision imagePairsMerging");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (inputFiles.empty())
    {
        ALICEVISION_LOG_ERROR("At least one input file should be given.");
        return EXIT_FAILURE;
    }

    PairSet mergedPairs;

    for (const auto& filename : inputFiles)
    {
        PairSet pairs;
        ALICEVISION_LOG_INFO("Loading image pairs from: " << filename);
        if (!matchingImageCollection::loadPairsFromFile(filename, pairs))
        {
            ALICEVISION_LOG_WARNING("Failed to load image pairs from: " << filename);
            continue;
        }
        ALICEVISION_LOG_INFO("Loaded " << pairs.size() << " pairs.");
        mergedPairs.insert(pairs.begin(), pairs.end());
    }

    ALICEVISION_LOG_INFO("Total merged pairs: " << mergedPairs.size());
    ALICEVISION_LOG_INFO("Saving merged image pairs to: " << outputFile);

    if (!matchingImageCollection::savePairsToFile(outputFile, mergedPairs))
    {
        ALICEVISION_LOG_ERROR("Failed to save image pairs to: " << outputFile);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
