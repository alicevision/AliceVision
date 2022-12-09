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

#include <thread>
#include <functional>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


int aliceVision_main(int argc, char **argv)
{
    // command-line arguments
    float capacity = 256;
    float maxSize = 1024;
    std::vector<std::string> filenames;
    std::vector<int> halfSampleLevels;
    int defaultHalfSampleLevel = 0;
    std::vector<int> pixelTypes;
    int defaultPixelType = 5;
    int nbThreads = 1;

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("filenames", po::value<std::vector<std::string>>(&filenames)->multitoken()->default_value(filenames), 
        "Filenames")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("capacity", po::value<float>(&capacity)->default_value(capacity), 
        "Cache capacity")
        ("maxSize", po::value<float>(&maxSize)->default_value(maxSize), 
        "Cache max size")
        ("halfSampleLevels", po::value<std::vector<int>>(&halfSampleLevels)->multitoken()->default_value(halfSampleLevels), 
        "Half-sampling levels")
        ("defaultHalfSampleLevel", po::value<int>(&defaultHalfSampleLevel)->default_value(defaultHalfSampleLevel), 
        "Default half-sampling level")
        ("pixelTypes", po::value<std::vector<int>>(&pixelTypes)->multitoken()->default_value(pixelTypes), 
        "Pixel types:"
        "\n * 0: unsigned char"
        "\n * 1: float"
        "\n * 2: RGB"
        "\n * 3: RGBf"
        "\n * 4: RGBA"
        "\n * 5: RGBAf")
        ("defaultPixelType", po::value<int>(&defaultPixelType)->default_value(defaultPixelType), 
        "Default pixel type")
        ("nbThreads", po::value<int>(&nbThreads)->default_value(nbThreads), 
        "Number of threads")
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

    // Start threads
    std::vector<std::thread> threads;
    for (int i = 0; i < nbThreads; i++)
    {
        threads.emplace_back([&](){
            // Load images
            for (int j = 0; j < filenames.size(); j++)
            {
                const std::string& filename = filenames[j];
                const int level = (j < halfSampleLevels.size()) ? halfSampleLevels[j] : defaultHalfSampleLevel;
                const int pixelType = (j < pixelTypes.size()) ? pixelTypes[j] : defaultPixelType;
                switch (pixelType)
                {
                    case 0:
                        { auto imgUChar = cache.get<unsigned char>(filename, level); }
                        break;
                    case 1:
                        { auto imgFloat = cache.get<float>(filename, level); }
                        break;
                    case 2:
                        { auto imgRGB = cache.get<image::RGBColor>(filename, level); }
                        break;
                    case 3:
                        { auto imgRGBf = cache.get<image::RGBfColor>(filename, level); }
                        break;
                    case 4:
                        { auto imgRGBA = cache.get<image::RGBAColor>(filename, level); }
                        break;
                    case 5:
                        { auto imgRGBAf = cache.get<image::RGBAfColor>(filename, level); }
                        break;
                    default:
                        break;
                }
            }
        });
    }

    // Join threads
    for (int i = 0; i < nbThreads; i++)
    {
        threads[i].join();
    }

    return EXIT_SUCCESS;
}
