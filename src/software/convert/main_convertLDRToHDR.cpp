// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/RobertsonCalibrate.hpp>
#include <aliceVision/hdr/RobertsonMerge.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string imageFolder;
  std::string outputHDRImagePath;
  std::string outputResponsePath;
  bool calibration;
  std::string weightFunctionName = hdr::EFunctionType_enumToString(hdr::EFunctionType::GAUSSIAN);

  po::options_description allParams("AliceVision convertLDRToHDR");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("inputFolder,i", po::value<std::string>(&imageFolder)->required(),
      "LDR images folder.")
    ("output,o", po::value<std::string>(&outputHDRImagePath)->required(),
      "output HDR image path.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).")
    ("response,r", po::value<std::string>(&outputResponsePath),
       "output response function path.")
    ("calibration,c", po::value<bool>(&calibration)->default_value(false),
        "calibration of the camera response function (true, false).")
    ("weight,w", po::value<std::string>(&weightFunctionName)->default_value(weightFunctionName),
       "weight function type (gaussian, triangle, plateau).");

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

  const std::size_t channelQuantization = std::pow(2, 12); //RAW 12 bit precision, 2^12 values between black and white point

  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups_cropped(1);
  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups(1);
  std::vector< std::vector<float> > times(1);
  hdr::rgbCurve weight(channelQuantization);
  hdr::rgbCurve response(channelQuantization);

  std::vector<image::Image<image::RGBfColor>> &ldrImages_cropped = ldrImageGroups_cropped.at(0);
  std::vector<image::Image<image::RGBfColor>> &ldrImages = ldrImageGroups.at(0);
  std::vector<float> &ldrTimes = times.at(0);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // set the correct weight function corresponding to the string parameter
  const hdr::EFunctionType weightFunction = hdr::EFunctionType_stringToEnum(weightFunctionName);
  weight.setFunction(weightFunction);

  // get all valid images paths from input folder
  std::vector<std::string> inputFilesNames;
  const std::vector<std::string> validExtensions = {".jpg", ".jpeg", ".png", ".cr2", ".tiff", ".tif"};
  for(auto& entry: fs::directory_iterator(fs::path(imageFolder)))
  {
      if(fs::is_regular_file(entry.status()))
      {
          std::string lowerExtension=entry.path().extension().string();
          std::transform(lowerExtension.begin(), lowerExtension.end(), lowerExtension.begin(), ::tolower);
          if(std::find(validExtensions.begin(), validExtensions.end(), lowerExtension) != validExtensions.end())
          {
              inputFilesNames.push_back(entry.path().string());
          }
      }
  }

  int nbImages = inputFilesNames.size();
  ldrImages_cropped.resize(nbImages);
  ldrImages.resize(nbImages);
  for(int i=0; i<nbImages; ++i)
  {
    std::string imagePath = inputFilesNames.at(i);
    int w, h;
    std::map<std::string, std::string> metadata;

    image::readImage(imagePath, ldrImages.at(i));
    image::readImageMetadata(imagePath, w, h, metadata);

    // we use a cropped image for calibration (fisheye images)
    ldrImages_cropped.at(i) = image::Image<image::RGBfColor>(ldrImages.at(i).block<2000,2000>(h/2-1000, w/2-1000));

    // Debevec and Robertson algorithms use shutter speed as ev value
    float ev;
    try
    {
      // const float aperture = std::stof(metadata.at("FNumber"));
      // const float iso = std::stof(metadata.at("Exif:PhotographicSensitivity"));
      // ev = std::log2(pow(aperture, 2) / shutter) + std::log2(iso/100);

      const float shutter = std::stof(metadata.at("ExposureTime"));
      ev = shutter;
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR("no metadata in images : " << imagePath);
      return EXIT_FAILURE;
    }

    ldrTimes.push_back(ev);
  }

  float targetTime;
  {
    std::vector<float> ldrTimes_sorted = ldrTimes;
    std::sort(ldrTimes_sorted.begin(), ldrTimes_sorted.end());
    targetTime = ldrTimes_sorted.at(ldrTimes_sorted.size()/2);
  }

  image::Image<image::RGBfColor> image(ldrImages.at(0).Width(), ldrImages.at(0).Height(), false);
  if(calibration == true)
  {
    // Robertson calibrate
    hdr::RobertsonCalibrate calibration(40);
    calibration.process(ldrImageGroups_cropped, times, weight, response, targetTime);

    hdr::RobertsonMerge merge;
    merge.process(ldrImages, ldrTimes, weight, response, image, targetTime);
  }
  else
  {
    // Robertson merge only
    response.setLinear();
    hdr::RobertsonMerge merge;
    merge.process(ldrImages, ldrTimes, weight, response, image, targetTime);
  }

  image::writeImage(outputHDRImagePath, image);
  if(!outputResponsePath.empty())    response.write(outputResponsePath);

  return EXIT_SUCCESS;
}


