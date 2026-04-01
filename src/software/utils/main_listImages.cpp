// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Parallelization.hpp>
#include <aliceVision/sfmDataIO/viewIO.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmData/uid.hpp>
#include <aliceVision/sfmData/ImageGroup.hpp>
#include <boost/program_options.hpp>

#include <filesystem>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> inputPaths;
    std::string outSfMDataFilepath;
    bool isSequence = false;
    
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input", po::value<std::vector<std::string>>(&inputPaths)->multitoken()->required(), "Path to folder(s) or images")
    ("output,o", po::value<std::string>(&outSfMDataFilepath)->required(), "Path to the output SfMData file.");
        
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
    ("isSequence", po::value<bool>(&isSequence)->default_value(isSequence), "The images provided as input are part of a sequence with temporal coherency.");
    // clang-format on

    CmdLine cmdline("AliceVision listImages");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());


    // Loop over all inputs
    std::unordered_set<std::string> allImages;
    for (const auto path : inputPaths)
    {
        // Find all images
        std::vector<std::string> images;
        if (!image::listImages(images, path))
        {
            continue;
        }

        // Add found images, removing duplicates
        allImages.insert(images.begin(), images.end());
    }

    // Copy to vector for OpenMP compatibility
    std::vector<std::string> allImagesItems(allImages.begin(), allImages.end());

    sfmData::SfMData sfmData;
    sfmData::Views& views = sfmData.getViews();

    // Create one view per path
    #pragma omp parallel for
    for (int i = 0; i < allImagesItems.size(); i++)
    {
        const std::string & path = allImagesItems[i];
        auto view = std::make_shared<sfmData::View>(path);
        
        // Fill the view
        // Mostly generate unique ids
        sfmDataIO::bootstrapView(*view, sfmDataIO::EViewIdMethod::METADATA, "");

        // try to detect image sequence
        IndexT frameId;
        std::string prefix;
        std::string suffix;
        if (sfmDataIO::extractNumberFromFileStem(std::filesystem::path(path).stem().string(), frameId, prefix, suffix))
        {
            view->setFrameId(frameId);
        }
        
        #pragma omp critical
        {
            views.emplace(view->getViewId(), view);
        }
    }

    // Set the unique image group id for all the views of the sfmData
    IndexT imageGroupId = computeGlobalId(sfmData.getViews());
    for (auto & [_, view]: sfmData.getViews().valueRange())
    {
        view.setImageGroupId(imageGroupId);
    }

    if (sfmData.getViews().size() > 0)
    {
        // Create the image group
        auto ptrImageGroup = sfmData::ImageGroup::create(isSequence ? sfmData::ImageGroup::Type::ImageSequence : sfmData::ImageGroup::Type::ImageSet);
        sfmData.getImageGroups().emplace(imageGroupId, ptrImageGroup);
    }

    // store SfMData views & intrinsic data
    if (!sfmDataIO::save(sfmData, outSfMDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ESfMData::VIEWS)))
    {
        return EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

