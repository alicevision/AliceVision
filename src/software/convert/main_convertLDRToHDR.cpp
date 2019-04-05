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
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/DebevecMerge.hpp>

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


enum class ECalibrationMethod
{
      ROBERTSON,
      DEBEVEC
};

/**
 * @brief convert an enum ECalibrationMethod to its corresponding string
 * @param ECalibrationMethod
 * @return String
 */
inline std::string ECalibrationMethod_enumToString(const ECalibrationMethod calibrationMethod)
{
  switch(calibrationMethod)
  {
    case ECalibrationMethod::ROBERTSON:   return "robertson";
    case ECalibrationMethod::DEBEVEC:     return "debevec";
  }
  throw std::out_of_range("Invalid method name enum");
}

/**
 * @brief convert a string calibration method name to its corresponding enum ECalibrationMethod
 * @param ECalibrationMethod
 * @return String
 */
inline ECalibrationMethod ECalibrationMethod_stringToEnum(const std::string& calibrationMethodName)
{
  std::string methodName = calibrationMethodName;
  std::transform(methodName.begin(), methodName.end(), methodName.begin(), ::tolower);

  if(methodName == "robertson")   return ECalibrationMethod::ROBERTSON;
  if(methodName == "debevec")     return ECalibrationMethod::DEBEVEC;

  throw std::out_of_range("Invalid method name : '" + calibrationMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, const ECalibrationMethod calibrationMethodName)
{
  os << ECalibrationMethod_enumToString(calibrationMethodName);
  return os;
}

inline std::istream& operator>>(std::istream& in, ECalibrationMethod calibrationMethod)
{
  std::string token;
  in >> token;
  calibrationMethod = ECalibrationMethod_stringToEnum(token);
  return in;
}


int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string imageFolder;
  std::string outputHDRImagePath;
  std::string outputResponsePath;
  bool calibration;
  std::string calibrationMethodName = ECalibrationMethod_enumToString(ECalibrationMethod::DEBEVEC);
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
    ("calibrationMethod,m", po::value<std::string>(&calibrationMethodName)->default_value(calibrationMethodName),
        "method used for camera calibration.")
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

  ALICEVISION_COUT("response file: " << outputResponsePath);

  const std::size_t channelQuantization = std::pow(2, 12); //RAW 12 bit precision, 2^12 values between black and white point
  //const std::size_t channelQuantization = std::pow(2, 8); //JPG 8 bit precision, 256 values between black and white point

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

  // set the correct calibration method corresponding to the string parameter
  const ECalibrationMethod calibrationMethod = ECalibrationMethod_stringToEnum(calibrationMethodName);

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

    image::readImage(imagePath, ldrImages.at(i), image::EImageColorSpace::NO_CONVERSION);
    image::readImageMetadata(imagePath, w, h, metadata);

    // we use a cropped image for calibration (fisheye images)
    ldrImages_cropped.at(i) = image::Image<image::RGBfColor>(ldrImages.at(i).block<4000,4000>(h/2-2000, w/2-2000));
    image::writeImage("/s/prods/mvg/_source_global/samples/HDR_selection/terrasse_2/Mikros/Debevec/image_cropped_2.jpg", ldrImages_cropped.at(i));

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
  ALICEVISION_COUT("target time = " << targetTime);

  image::Image<image::RGBfColor> image(ldrImages.at(0).Width(), ldrImages.at(0).Height(), false);

  if(calibration == true)
  {
    // calculate the response function according to the method given in argument and merge the HDR
    switch(calibrationMethod)
    {
      case ECalibrationMethod::DEBEVEC:
      {
        ALICEVISION_COUT("Debevec calibration");
        const float lambda = 1000.f;
        const int nbPoints = 1000;
        hdr::DebevecCalibrate calibration(channelQuantization);
        calibration.process(ldrImageGroups_cropped, times, nbPoints, weight, lambda, response);

//        response.write(outputResponsePath);

        hdr::DebevecMerge merge;
        merge.process(ldrImages, ldrTimes, weight, response, image, targetTime);

//        hdr::rgbCurve expResponse(channelQuantization);
//        for(unsigned int channel=0; channel<3; ++channel)
//        {
//          for(unsigned int k=0; k<channelQuantization; ++k)
//            expResponse.setValue(k, channel, std::exp(response.getValue(k, channel)));
//        }
//        ALICEVISION_COUT("response(middle) = " << expResponse.getValue(2048, 0));
//        expResponse.write("/s/prods/mvg/_source_global/samples/HDR_selection/terrasse_2/Mikros/Debevec/response_test_exp.csv");
      }
      break;

      case ECalibrationMethod::ROBERTSON:
      {
        ALICEVISION_COUT("Robertson calibration");
        hdr::RobertsonCalibrate calibration(40);
        calibration.process(ldrImageGroups_cropped, times, weight, response, targetTime);

//        response.write(outputResponsePath);

        hdr::RobertsonMerge merge;
        merge.process(ldrImages, ldrTimes, weight, response, image, targetTime);
      }
      break;
    }
  }
  else
  {
    // set the response function to linear
    response.setLinear();
//    response.write(outputResponsePath);
    hdr::RobertsonMerge merge;
    merge.process(ldrImages, ldrTimes, weight, response, image, targetTime);
  }


  image::writeImage(outputHDRImagePath, image, image::EImageColorSpace::NO_CONVERSION);
  if(!outputResponsePath.empty())   response.write(outputResponsePath);





  return EXIT_SUCCESS;
}


