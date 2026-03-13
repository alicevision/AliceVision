// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Parallelization.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <random>
#include <filesystem>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

/**
@brief Each pixel (x,y) in integralCount is the count of non-zero pixel 
in the input mask's sub rectangle (0->x; 0->y).
@param integralCount the output integral image
@param mask the input mask
*/
void computeIntegralMask(image::Image<unsigned long> & integralCount, const image::Image<unsigned char> & mask)
{
    integralCount.resize(mask.width(), mask.height());

    //Bootstrap first row
    int countFirstRow = 0;
    for (int j = 0; j < mask.width(); j++)
    {
        if (mask(0, j))
        {
            countFirstRow++;
            integralCount(0, j) = countFirstRow;
        }
    }

    //For all next rows
    for (int i = 1; i < mask.height(); i++)
    {
        int countRow = 0;
        for (int j = 0; j < mask.width(); j++)
        {
            if (mask(i, j))
            {
                countRow++;
            }

            integralCount(i, j) = integralCount(i - 1, j) + countRow;
        }
    }
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string directoryName;
    std::string outDirectory;
    int rangeIteration = 0;
    int rangeBlocksCount = 1;
    int radius = 5;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&directoryName)->required(),
         "Path to directories to process.")
        ("output,o", po::value<std::string>(&outDirectory)->required(),
         "Output directory.");
        
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("radius", po::value<int>(&radius)->default_value(radius), "")
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Chunk id.")
        ("rangeBlocksCount", po::value<int>(&rangeBlocksCount)->default_value(rangeBlocksCount), "Chunk count.");
    // clang-format on

    CmdLine cmdline("AliceVision maskEroding");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    std::vector<std::filesystem::path> paths;
    for (auto &p : std::filesystem::recursive_directory_iterator(directoryName))
    {
        const std::filesystem::path path = p.path();
        if (path.extension() != ".exr")
        {
            continue;
        }

        paths.push_back(path);
    }

    int rangeStart, rangeEnd;
    if (!rangeComputation(rangeStart, rangeEnd, rangeIteration, rangeBlocksCount, paths.size()))
    {
        ALICEVISION_LOG_ERROR("Problem computing chunks.");
        return EXIT_FAILURE;
    }

    #pragma omp parallel for schedule(dynamic)
    for (int pid = rangeStart; pid < rangeEnd; pid++)
    {
        const auto & path = paths[pid];

        ALICEVISION_LOG_INFO("Processing " << path.string());

        std::filesystem::path outputDirectoryPath(outDirectory);
        std::filesystem::path outputPath = outputDirectoryPath / path.filename();


        image::Image<unsigned char> img;
        aliceVision::image::readImage(path.string(), img, image::EImageColorSpace::NO_CONVERSION);

        image::Image<unsigned long> integral;
        computeIntegralMask(integral, img);

        image::Image<unsigned char> out(img.width(), img.height());
        for (int i = 0; i < img.height(); i++)
        {
            int mini = std::max(i - radius, 0);
            int maxi = std::min(i + radius, img.height() - 1);
            int si = maxi - mini + 1;

            for (int j = 0; j < img.width(); j++)
            {
                int minj = std::max(j - radius, 0);
                int maxj = std::min(j + radius, img.width() - 1);
                int sj = maxj - minj + 1;

                //A B
                //C D
                //"CD" = D - V)
                long A = 0;
                long B = 0;
                long C = 0;
                long D = integral(maxi, maxj);

                if (minj > 0)
                {
                    C = integral(maxi, minj - 1);
                }
                if (mini > 0)
                {
                    B = integral(mini - 1, maxj);

                    if (minj > 0)
                    {
                        A = integral(mini - 1, minj - 1);
                    }
                }

                //Are all the pixels valid ?
                long sub = D - C - B + A;
                out(i, j) = (sub == (si * sj))?255:0;
            }
        }
        
        aliceVision::image::ImageWriteOptions wopt;
        aliceVision::image::writeImage(outputPath.string(), out, wopt);
    }    
    
    return EXIT_SUCCESS;
}

