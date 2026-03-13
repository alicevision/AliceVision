// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Image.hpp>

#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <filesystem>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string tracksInputFilename;
    std::string tracksOutputFilename;
    std::string depthSource;
    
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("tracksFilename", po::value<std::string>(&tracksInputFilename)->required(), "Tracks input file.")
    ("output,o", po::value<std::string>(&tracksOutputFilename)->required(), "Tracks output file.")
    ("depthSource", po::value<std::string>(&depthSource)->required(), "Depth Source directory.");

    CmdLine cmdline("AliceVision DepthMapTracksInjecting");

    cmdline.add(requiredParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::VIEWS))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksInputFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksInputFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    const auto & tracksPerView = tracksHandler.getTracksPerView();
    auto & tracks = tracksHandler.getAllTracksMutable();

    for (const auto & [idView, view]: sfmData.getViews())
    {
        if (tracksPerView.find(idView) == tracksPerView.end())
        {
            continue;
        }

        fs::path path(view->getImage().getImagePath());
        std::string depthPath = depthSource + "/depth_" + path.stem().string() + ".exr";

        image::Image<float> depthImg;

        try
        {
            image::readImage(depthPath, depthImg, image::EImageColorSpace::NO_CONVERSION);
        }
        catch(...)
        {
            continue;
        }

        int swidth = view->getImage().getWidth();
        int sheight = view->getImage().getHeight();
        
        double scaleW = 1.0;
        double scaleH = 1.0;
        if (depthImg.width() != swidth || depthImg.height() != sheight)
        {
            ALICEVISION_LOG_INFO("Depth image has a different size than the input image");
            scaleW = double(depthImg.width()) / double(swidth);
            scaleH = double(depthImg.height()) / double(sheight);
            ALICEVISION_LOG_INFO("Relative scale : (" << scaleW << ","<< scaleH <<")");
        }

        ALICEVISION_LOG_INFO("Injecting from " << depthPath);
        for (const auto & trackId: tracksPerView.at(idView))
        {
            track::Track & track = tracks[trackId];

            auto & feature = track.featPerView.at(idView);
            
            int ix = int(scaleW * feature.coords.x());
            int iy = int(scaleH * feature.coords.y());

            if (ix < 0 || ix >= depthImg.width()) continue;
            if (iy < 0 || iy >= depthImg.height()) continue;

            double Z = depthImg(iy, ix);
            if (!std::isfinite(Z)) continue;
	
            feature.depth = Z;
        }
    }

    // Save tracks
    ALICEVISION_LOG_INFO("Saving to " << tracksOutputFilename);
    ALICEVISION_LOG_INFO("File has " << tracksHandler.getAllTracks().size() << " tracks.");
    if (!track::saveTracks(tracksHandler.getAllTracks(), tracksOutputFilename))
    {
        ALICEVISION_LOG_ERROR("Failed to save file");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
