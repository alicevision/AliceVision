// This file is part of the AliceVision project. 
// Copyright (c) 2016 AliceVision contributors. 
// This Source Code Form is subject to the terms of the Mozilla Public License, 
// v. 2.0. If a copy of the MPL was not distributed with this file, 
// You can obtain one at https://mozilla.org/MPL/2.0/. 
 
#include <aliceVision/sfmData/SfMData.hpp> 
#include <aliceVision/system/Logger.hpp> 
#include <aliceVision/cmdline/cmdline.hpp> 
#include <aliceVision/image/io.hpp> 
#include <aliceVision/image/pixelTypes.hpp> 
 
#include <boost/program_options.hpp> 
#include <boost/filesystem.hpp> 
 
#include <string> 
#include <sstream> 
 
#include <OpenImageIO/paramlist.h> 
 
// These constants define the current software version. 
// They must be updated when the command line is changed. 
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1 
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0 
 
using namespace aliceVision; 
 
namespace po = boost::program_options; 
namespace fs = boost::filesystem; 
 
namespace oiio = OIIO; 
 
int main(int argc, char **argv) 
{ 
    // command-line parameters 
 
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel()); 
    std::string inputFolderPath; 
    std::string outputFilePath; 
 
    po::options_description requiredParams("Required parameters"); 
    requiredParams.add_options() 
        ("input,i", po::value<std::string>(&inputFolderPath)->required(), 
         "Input file path.") 
        ("output,o", po::value<std::string>(&outputFilePath)->required(), 
         "Path to save files in."); 

    aliceVision::CmdLine cmdline("AliceVision Sample evCorrection");
    cmdline.add(requiredParams);
    if(!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfm_data; 
 
    int c = 0; 
    fs::path folderPath(inputFolderPath); 
    for(auto& file : fs::directory_iterator(folderPath)) 
    { 
        fs::path filePath = file.path(); 
        if(fs::is_regular_file(filePath)) 
        { 
            int w, h; 
            const IndexT id_view = c, id_pose = c, id_intrinsic = 0, rigId = 0, subPoseId = 0; 
 
            const auto metadata = image::readImageMetadata(filePath.string(), w, h);
 
            sfm_data.views[c] = std::make_shared<sfmData::View>(filePath.string(), id_view, id_intrinsic,
                                                                id_pose, w, h, rigId, subPoseId,
                                                                image::getMapFromMetadata(metadata));
 
            ++c; 
        } 
 
    } 
 
    const double cameraExposureMedian = sfm_data.getMedianCameraExposureSetting().getExposure();
    ALICEVISION_LOG_INFO("  EV Median :" << cameraExposureMedian); 
 
    for(int i = 0; i < sfm_data.views.size(); ++i) 
    { 
        const sfmData::View& view = *(sfm_data.views[i]);
        const float evComp = float(cameraExposureMedian / view.getImage().getCameraExposureSetting().getExposure());
 
        image::Image<image::RGBfColor> img; 
        image::readImage(view.getImage().getImagePath(), img, image::EImageColorSpace::LINEAR); 
 
        for(int pix = 0; pix < view.getImage().getWidth() * view.getImage().getHeight(); ++pix) 
            img(pix) *= evComp; 
 
        ALICEVISION_LOG_INFO(fs::path(view.getImage().getImagePath()).stem()); 
        ALICEVISION_LOG_INFO("  EV: " << view.getImage().getEv()); 
        ALICEVISION_LOG_INFO("  EV Compensation: " << evComp);

        std::string outputPath = outputFilePath + fs::path(view.getImage().getImagePath()).stem().string() + ".EXR"; 
        oiio::ParamValueList metadata = image::getMetadataFromMap(view.getImage().getMetadata()); 
        image::writeImage(outputPath, img,
                          image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::LINEAR),
                          metadata);
    } 

 
 
    return EXIT_SUCCESS; 
} 
 
