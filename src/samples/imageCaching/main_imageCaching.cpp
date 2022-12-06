// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/caching.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


int aliceVision_main(int argc, char **argv)
{
    // command-line arguments
    int capacity = 256;
    int maxSize = 1024;
    std::vector<std::string> filenames;
    std::vector<int> halfSampleLevels = {1};

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("filenames", po::value<std::vector<std::string>>(&filenames)->multitoken()->default_value(filenames), 
        "Filenames")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("capacity", po::value<int>(&capacity)->default_value(capacity), 
        "Cache capacity")
        ("maxSize", po::value<int>(&maxSize)->default_value(maxSize), 
        "Cache max size")
        ("halfSampleLevels", po::value<std::vector<int>>(&halfSampleLevels)->multitoken()->default_value(halfSampleLevels), 
        "halfSampleLevels")
        ;

    CmdLine cmdline("AliceVision imageCaching");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Setup cache
    ALICEVISION_LOG_INFO("Creating image cache with capacity " << capacity << " and max size " << maxSize);
    image::ImageReadOptions readOptions(image::EImageColorSpace::LINEAR);
    image::ImageCache cache(capacity, maxSize, readOptions);

    // Load images
    for (const std::string& filename: filenames)
    {
        for (int level: halfSampleLevels)
        {
            ALICEVISION_LOG_INFO("Reading " << filename << " with half sample level " << level);
            auto img = cache.get(filename, level);
            ALICEVISION_LOG_INFO(cache.toString());
        }
    }

    return EXIT_SUCCESS;
}
