// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/utils/filesIO.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/alicevision_omp.hpp>

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


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string inputSfmDataFilename;
    std::string outputSfmDataFilename;
    std::string outputDirectory;
    std::string masks;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputSfmDataFilename)->required(),
         "Input SfmData.")
        ("outputSfmData", po::value<std::string>(&outputSfmDataFilename)->required(),
         "Output SfmData.")
        ("masks", po::value<std::string>(&masks)->required(),
         "Input Masks.")
        ("outputDirectory", po::value<std::string>(&outputDirectory)->required(),
         "Output directory with modified images.");
    // clang-format on

    CmdLine cmdline("AliceVision maskRandomApplying");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    if (!utils::exists(outputDirectory))
    {
        ALICEVISION_LOG_ERROR("Output Directory does not exists.");
        return EXIT_FAILURE;
    }

    if (!utils::exists(masks))
    {
        ALICEVISION_LOG_ERROR("Masks Directory does not exists.");
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, inputSfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }   
    
    // List input images
    std::vector<IndexT> viewIds;
    std::vector<std::filesystem::path> imagePaths;
    for (const auto [idview, view]: sfmData.getViews().valueRange())
    {
        std::filesystem::path p(view.getImage().getImagePath());
        imagePaths.push_back(p);
        viewIds.push_back(idview);
    }

    std::mt19937 gen;
    std::uniform_real_distribution<float> dis(0.0f, 1.0f);
    
    #pragma omp parallel for schedule(dynamic)
    for (int chunkId = 0; chunkId < imagePaths.size(); chunkId++)
    {
        IndexT viewId = viewIds[chunkId];
        std::filesystem::path path = imagePaths[chunkId];
        

        std::string maskFileName = masks + "/" + path.filename().string();
        if (!utils::exists(maskFileName))
        {
            ALICEVISION_LOG_INFO(maskFileName << " not found.");
            continue;
        }

        std::string outFileName = outputDirectory + "/" + path.filename().string();
        

        ALICEVISION_LOG_INFO("Processing " << path.string());

        image::Image<image::RGBfColor> img;
        aliceVision::image::readImage(path.string(), img, image::EImageColorSpace::NO_CONVERSION);

        image::Image<unsigned char> mask;
        aliceVision::image::readImage(maskFileName, mask, image::EImageColorSpace::NO_CONVERSION);

        if (img.height() != mask.height())
        {
            ALICEVISION_LOG_INFO("Incompatible sizes.");
            continue;
        }

        if (img.width() != mask.width())
        {
            ALICEVISION_LOG_INFO("Incompatible sizes.");
            continue;
        }

        for (int i = 0; i < mask.height(); i++)
        {
            for (int j = 0; j < mask.width(); j++)
            {
                if (!mask(i, j))
                {
                    auto & p = img(i, j);
                    p.r() = dis(gen);
                    p.g() = dis(gen);
                    p.b() = dis(gen);
                }
            }
        }

        aliceVision::image::ImageWriteOptions wopt;
        aliceVision::image::writeImage(outFileName, img, wopt);

        sfmData.getView(viewId).getImage().setImagePath(outFileName);
    }

    if (!sfmDataIO::save(sfmData, outputSfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outputSfmDataFilename << "'");
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

