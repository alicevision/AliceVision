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
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/GrossbergCalibrate.hpp>
#include <aliceVision/hdr/emorCurve.hpp>

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
      DEBEVEC,
      GROSSBERG
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
    case ECalibrationMethod::GROSSBERG:   return "grossberg";
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
  if(methodName == "grossberg")   return ECalibrationMethod::GROSSBERG;

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

/**
 * @brief recreate the source image at the target exposure by applying the inverse camera response function to HDR image
 * @param hdrImage
 * @param response
 * @param channelQuantization
 * @param path to write the output image
 */
void recoverSourceImage(const image::Image<image::RGBfColor>& hdrImage, hdr::rgbCurve& response, float channelQuantization, std::string path, float meanVal[])
{
    image::Image<image::RGBfColor> targetRecover(hdrImage.Width(), hdrImage.Height(), false);
    float meanRecovered[3] = {0.f, 0.f, 0.f};
    for(std::size_t channel = 0; channel < 3; ++channel)
    {
      std::vector<float>::iterator first = response.getCurve(channel).begin();
      std::vector<float>::iterator last = response.getCurve(channel).end();
      for(std::size_t y = 0; y < hdrImage.Height(); ++y)
      {
        for(std::size_t x = 0; x < hdrImage.Width(); ++x)
        {
          const float &pixelValue = hdrImage(y, x)(channel);
          std::vector<float>::iterator it = std::lower_bound(first, last, pixelValue);
          float value = float(std::distance(response.getCurve(channel).begin(), it)) / (channelQuantization - 1.f);
          targetRecover(y, x)(channel) =  value;

          meanRecovered[channel] += value;

        }
      }
      meanRecovered[channel] /= hdrImage.size();
    }
    ALICEVISION_COUT("mean values of recovered image = " << meanRecovered);
    float offset[3];
    for(int i=0; i<3; ++i)
        offset[i] = std::abs(meanRecovered[i] - meanVal[i]);
    ALICEVISION_COUT("offset between target source image and recovered from hdr = " << offset);

    image::writeImage(path, targetRecover, image::EImageColorSpace::NO_CONVERSION);
}


int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string imageFolder;
  std::string outputHDRImagePath;
  std::string outputResponsePath;
  bool calibration;
  std::string calibrationMethodName = ECalibrationMethod_enumToString(ECalibrationMethod::GROSSBERG);
  std::string weightFunctionName = hdr::EFunctionType_enumToString(hdr::EFunctionType::GAUSSIAN);
  std::string target;
  float threshold = 1.f;

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
       "weight function type (gaussian, triangle, plateau).")
    ("targetExposureTime,e", po::value<std::string>(&target),
      "target exposure time for the output HDR image to be centered")
    ("threshold,t", po::value<float>(&threshold)->default_value(threshold),
      "threshold for clamped value (0 for no correction, 0.5 for inside lights and 1 for outside lights");

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
//  const std::size_t channelQuantization = std::pow(2, 8); //JPG 8 bit precision, 256 values between black and white point
//  const std::size_t channelQuantization = std::pow(2, 10); //RAW 10 bit precision, 2^10 values between black and white point

  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups(1);
  std::vector< std::vector<float> > times(1);
//  std::vector< std::vector<float> > ev(1);
  hdr::rgbCurve weight(channelQuantization);
  hdr::rgbCurve response(channelQuantization);

  std::vector<image::Image<image::RGBfColor>> &ldrImages = ldrImageGroups.at(0);
  std::vector<float> &ldrTimes = times.at(0);
//  std::vector<float> &ldrEv = ev.at(0);

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // set the correct calibration method corresponding to the string parameter
  const ECalibrationMethod calibrationMethod = ECalibrationMethod_stringToEnum(calibrationMethodName);

  // set the correct weight function corresponding to the string parameter
  const hdr::EFunctionType weightFunction = hdr::EFunctionType_stringToEnum(weightFunctionName);
  weight.setFunction(weightFunction);

  // get all valid images paths from input folder
  std::vector<std::string> inputFilesNames;
  std::vector<std::string> stemImages;
  std::vector<std::string> nameImages;
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
              stemImages.push_back(entry.path().stem().string());
              nameImages.push_back(entry.path().filename().string());
          }
      }
  }



  int nbImages = inputFilesNames.size();
  ldrImages.resize(nbImages);
  for(int i=0; i<nbImages; ++i)
  {
    std::string imagePath = inputFilesNames.at(i);
    int w, h;
    std::map<std::string, std::string> metadata;

    image::readImage(imagePath, ldrImages.at(i), image::EImageColorSpace::SRGB);

    // test image conversion to sRGB
//    image::writeImage(std::string("/s/prods/mvg/_source_global/samples/HDR_selection/terrasse_2/Mikros/sources/JPG/" + nameImages.at(i) + ".jpg"), ldrImages.at(i), image::EImageColorSpace::NO_CONVERSION);
    image::readImageMetadata(imagePath, w, h, metadata);

    // Debevec and Robertson algorithms use shutter speed as ev value
    float shutter;
    try
    {
//      const float aperture = std::stof(metadata.at("FNumber"));
//      const float iso = std::stof(metadata.at("Exif:PhotographicSensitivity"));
//      const float iso = std::stof(metadata.at("Exif:ISOSpeedRatings"));
      shutter = std::stof(metadata.at("ExposureTime"));
//      ldrEv.push_back(std::log2(pow(aperture, 2) / shutter) + std::log2(iso/100));
    }
    catch(std::exception& e)
    {
      ALICEVISION_LOG_ERROR("no metadata in images : " << imagePath);
      return EXIT_FAILURE;
    }

    ldrTimes.push_back(shutter);
  }

  std::vector< std::vector<float> > times_sorted = times;
  std::sort(times_sorted.at(0).begin(), times_sorted.at(0).end());
  std::vector<float> ldrTimes_sorted = times_sorted.at(0);

  float targetTime;
  if(!target.empty())
  {
      nameImages.insert(nameImages.end(), stemImages.begin(), stemImages.end());    //search target for filenames only and filenames + extensions
      std::size_t targetIndex = std::distance(nameImages.begin(), std::find(nameImages.begin(), nameImages.end(), target));
      try
      {
        targetTime = ldrTimes.at(targetIndex);
      }
      catch(std::exception& e)
      {
          try
          {
              targetTime = ldrTimes.at(targetIndex - ldrTimes.size());
          }
          catch(std::exception& e)
          {
              ALICEVISION_CERR("Invalid name of target");
              return EXIT_FAILURE;
          }
      }
  }
  else
      targetTime = ldrTimes_sorted.at(ldrTimes_sorted.size()/2);

  // calcul of mean value of target image
  std::size_t targetIndex = std::distance(ldrTimes.begin(), std::find(ldrTimes.begin(), ldrTimes.end(), targetTime));
  float meanVal[3] = {0.f, 0.f, 0.f};
  for(std::size_t channel=0; channel<3; ++channel)
  {
      for(std::size_t y = 0; y < ldrImages.at(0).Height(); ++y)
      {
        for(std::size_t x = 0; x < ldrImages.at(0).Width(); ++x)
        {
            meanVal[channel] += ldrImages.at(targetIndex)(y, x)(channel);
        }
      }
      meanVal[channel] /= ldrImages.at(targetIndex).size();
  }
//  ALICEVISION_COUT("mean values of target image = " << meanVal);

  // we sort the images according to their exposure time
  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups_sorted(1);
  ldrImageGroups_sorted.at(0).resize(nbImages);
//  std::vector<std::string> nameImages_sorted = nameImages;
  for(int i=0; i<nbImages; ++i)
  {
      std::vector<float>::iterator it = std::find(ldrTimes.begin(), ldrTimes.end(), ldrTimes_sorted.at(i));
      if(it != ldrTimes.end())
      {
          ldrImageGroups_sorted.at(0).at(i) = ldrImages.at(std::distance(ldrTimes.begin(), it));
//          nameImages_sorted.at(i) = nameImages.at(std::distance(ldrTimes.begin(), it));
      }
      else
          ALICEVISION_LOG_ERROR("sorting failed");
  }

//  ALICEVISION_COUT("sorted images : " << nameImages_sorted);

  image::Image<image::RGBfColor> image(ldrImages.at(0).Width(), ldrImages.at(0).Height(), false);

  if(calibration == true)
  {
    // calculate the response function according to the method given in argument and merge the HDR
    switch(calibrationMethod)
    {
      case ECalibrationMethod::DEBEVEC:
      {
        ALICEVISION_COUT("Debevec calibration");
        const float lambda = channelQuantization * 1.f;
        const int nbPoints = 1000;
        hdr::DebevecCalibrate calibration;
        calibration.process(ldrImageGroups, channelQuantization, times, nbPoints, weight, lambda, response);

        response.exponential();
//                response.scale();
      }
      break;

      case ECalibrationMethod::ROBERTSON:
      {
        ALICEVISION_COUT("Robertson calibration");
        hdr::RobertsonCalibrate calibration(40);
        const int nbPoints = 1000000;
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, weight, response, targetTime, threshold);
        response.scale();
      }
      break;

      case ECalibrationMethod::GROSSBERG:
      {
        ALICEVISION_COUT("Grossberg calibration");
        const int nbPoints = 1000000;
        hdr::GrossbergCalibrate calibration(5);
        calibration.process(ldrImageGroups_sorted, times_sorted, nbPoints, weight, response);
//        response.scale();
      }
      break;
    }
  }
  else
  {
    // set the response function to linear
    response.setLinear();
  }

  hdr::hdrMerge merge;
  merge.process(ldrImageGroups_sorted.at(0), ldrTimes_sorted, weight, response, image, targetTime, threshold);


  image::writeImage(outputHDRImagePath, image, image::EImageColorSpace::NO_CONVERSION);
  if(!outputResponsePath.empty())   response.write(outputResponsePath);


  // test of recovery of source target image from HDR
  recoverSourceImage(image, response, channelQuantization, "/s/prods/mvg/_source_global/samples/HDR_selection/terrasse_2/Mikros/Centrage_expos/recovered_from_Gros_bonsCoeff_meanCurve.exr", meanVal);


  return EXIT_SUCCESS;
}


