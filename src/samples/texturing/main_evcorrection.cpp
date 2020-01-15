// This file is part of the AliceVision project. 
// Copyright (c) 2016 AliceVision contributors. 
// This Source Code Form is subject to the terms of the Mozilla Public License, 
// v. 2.0. If a copy of the MPL was not distributed with this file, 
// You can obtain one at https://mozilla.org/MPL/2.0/. 
 
#include <aliceVision/sfmData/SfMData.hpp> 
#include <aliceVision/system/Logger.hpp> 
#include <aliceVision/system/cmdline.hpp> 
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
 
    po::options_description allParams("AliceVision evCorrection"); 
 
    po::options_description requiredParams("Required parameters"); 
    requiredParams.add_options() 
            ("input,i", po::value<std::string>(&inputFolderPath)->required(), 
             "Input file path.") 
            ("output,o", po::value<std::string>(&outputFilePath)->required(), 
             "Path to save files in."); 
 
    po::options_description logParams("Log parameters"); 
    logParams.add_options() 
            ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel), 
             "verbosity level (fatal,  error, warning, info, debug, trace)."); 
 
 
    allParams.add(requiredParams).add(logParams); 
 
    po::variables_map vm; 
    try 
    { 
        po::store(po::parse_command_line(argc, argv, allParams), vm); 
 
        if(vm.count("help") || (argc == 1)) 
        { 
            ALICEVISION_COUT(allParams); 
            return EXIT_SUCCESS; 
        } 
        po::notify(vm); 
    } 
    catch(boost::program_options::required_option& e) 
    { 
        ALICEVISION_CERR("ERROR: " << e.what()); 
        ALICEVISION_COUT("Usage:\n\n" << allParams); 
        return EXIT_FAILURE; 
    } 
    catch(boost::program_options::error& e) 
    { 
        ALICEVISION_CERR("ERROR: " << e.what()); 
        ALICEVISION_COUT("Usage:\n\n" << allParams); 
        return EXIT_FAILURE; 
    } 
 
    ALICEVISION_COUT("Program called with the following parameters:"); 
    ALICEVISION_COUT(vm); 
 
    // set verbose level 
    system::Logger::get()->setLogLevel(verboseLevel); 
 
    sfmData::SfMData sfm_data; 
 
    int c = 0; 
    fs::path folderPath(inputFolderPath); 
    for(auto& file : fs::directory_iterator(folderPath)) 
    { 
        fs::path filePath = file.path(); 
        if(fs::is_regular_file(filePath)) 
        { 
            int w, h; 
            std::map<std::string, std::string> metadata; 
            const IndexT id_view = c, id_pose = c, id_intrinsic = 0, rigId = 0, subPoseId = 0; 
 
            image::readImageMetadata(filePath.string(), w, h, metadata); 
 
            sfm_data.views[c] = std::make_shared<sfmData::View>(filePath.string(), id_view, id_intrinsic, id_pose, w, h, rigId, subPoseId, metadata); 
 
            ++c; 
        } 
 
    } 
 
    float cameraExposureMedian = sfm_data.getMedianCameraExposureSetting(); 
    ALICEVISION_LOG_INFO("  EV Median :" << cameraExposureMedian); 
 
    for(int i = 0; i < sfm_data.views.size(); ++i) 
    { 
        sfmData::View view = *(sfm_data.views[i]);
        float evComp = cameraExposureMedian / view.getCameraExposureSetting();
 
        image::Image<image::RGBfColor> img; 
        image::readImage(view.getImagePath(), img, image::EImageColorSpace::LINEAR); 
 
        for(int pix = 0; pix < view.getWidth() * view.getHeight(); ++pix) 
            img(pix) *= evComp; 
 
        ALICEVISION_LOG_INFO(fs::path(view.getImagePath()).stem()); 
        ALICEVISION_LOG_INFO("  EV: " << view.getEv()); 
        ALICEVISION_LOG_INFO("  EV Compensation: " << evComp);

        std::string outputPath = outputFilePath + fs::path(view.getImagePath()).stem().string() + ".EXR"; 
        oiio::ParamValueList metadata = image::getMetadataFromMap(view.getMetadata()); 
        image::writeImage(outputPath, img, image::EImageColorSpace::LINEAR, metadata); 
    } 
 
/* 
    // calculate EV for all images in the input folder 
    fs::path folderPath(inputFolderPath); 
    for(auto& filePath : fs::directory_iterator(folderPath)) 
    { 
        Image img; 
        int w, h; 
        float ev; 
        oiio::ParamValueList metadata; 
 
        imageIO::readImage(filePath.path().string(), w, h, img.data(), imageIO::EImageColorSpace::LINEAR); 
        img.setSize(w, h); 
        img.setName(filePath.path().stem().string()); 
        imageIO::readImageMetadata(filePath.path().string(), metadata); 
 
        ALICEVISION_LOG_INFO(img.getName()); 
 
        const double shutter = metadata.get_float("ExposureTime"); 
        const double aperture = metadata.get_float("FNumber"); 
        const float iso = metadata.get_float("Exif:PhotographicSensitivity"); 
        ev = log2f(std::pow(aperture, 2.0f) / shutter) - log2f(iso/100.f); 
 
        img.setEv(ev); 
        evList.push_back(ev); 
 
        ALICEVISION_LOG_INFO("  Shutter: " << shutter); 
        ALICEVISION_LOG_INFO("  Aperture: " << aperture); 
        ALICEVISION_LOG_INFO("  ISO: " << iso); 
 
        ALICEVISION_LOG_INFO("  EV: " << img.getEv()); 
 
        imgList.push_back(img); 
    } 
 
    // calculate median EV if necessary 
    std::sort(evList.begin(), evList.end()); 
    cameraExposureMedian = evList[evList.size()/2]; 
    ALICEVISION_LOG_INFO("Median EV: " << cameraExposureMedian); 
 
    // write corrected images, not over exposed images 
    for(int i = 0; i < imgList.size(); ++i) 
    { 
        Image& img = imgList[i]; 
        ALICEVISION_LOG_INFO(img.getName()); 
 
        float evComp = std::pow(2.0f, img.getEv() - cameraExposureMedian); 
        ALICEVISION_LOG_INFO("  EV Compensation: " << evComp); 
 
        for(int pix = 0; pix < img.width() * img.height(); ++pix) 
        { 
            img[pix] *= evComp; 
        } 
 
        std::string outputPath = outputFilePath + imgList[i].getName() + ".EXR"; 
        imageIO::writeImage(outputPath, imgList[i].width(), imgList[i].height(), imgList[i].data(), imageIO::EImageQuality::LOSSLESS, imageIO::EImageColorSpace::LINEAR); 
 
    } 
*/ 
 
 
    return EXIT_SUCCESS; 
} 
 
