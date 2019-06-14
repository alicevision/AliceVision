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
#include <regex>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


enum class ECalibrationMethod
{
      LINEAR,
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
    case ECalibrationMethod::LINEAR:      return "linear";
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

  if(methodName == "linear")      return ECalibrationMethod::LINEAR;
  if(methodName == "robertson")   return ECalibrationMethod::ROBERTSON;
  if(methodName == "debevec")     return ECalibrationMethod::DEBEVEC;
  if(methodName == "grossberg")   return ECalibrationMethod::GROSSBERG;

  throw std::out_of_range("Invalid method name : '" + calibrationMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ECalibrationMethod calibrationMethodName)
{
  os << ECalibrationMethod_enumToString(calibrationMethodName);
  return os;
}

inline std::istream& operator>>(std::istream& in, ECalibrationMethod& calibrationMethod)
{
  std::string token;
  in >> token;
  calibrationMethod = ECalibrationMethod_stringToEnum(token);
  return in;
}

/**
 * @brief recreate the source image at the target exposure by applying the inverse camera response function to HDR image
 * and calculate the offset between the mean value of target source image and the recovered image
 * @param[in] hdrImage
 * @param[in] response
 * @param[in] channelQuantization
 * @param[in] path to write the output image
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
  std::vector<std::string> imagesFolder;
  std::string outputHDRImagePath;
  std::string outputResponsePath;
  ECalibrationMethod calibrationMethod = ECalibrationMethod::LINEAR;
  std::string inputResponsePath;
  std::string calibrationWeightFunction = "default";
  hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
  std::string target;
  float clampedValueCorrection = 1.f;
  std::string recoverSourcePath;

  po::options_description allParams("AliceVision convertLDRToHDR");

  po::options_description requiredParams("Required Parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&imagesFolder)->required()->multitoken(),
      "List of LDR images or a folder containing them (accepted formats are: .jpg .jpeg .png .tif .tiff .cr2 .rw2)")
    ("output,o", po::value<std::string>(&outputHDRImagePath)->required(),
      "output HDR image path.");

  po::options_description optionalParams("Optional Parameters");
  optionalParams.add_options()
    ("outputResponse", po::value<std::string>(&outputResponsePath),
       "output response function path.")
    ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod )->default_value(calibrationMethod ),
        "method used for camera calibration (linear, robertson, debevec).")
    ("inputResponse,r", po::value<std::string>(&inputResponsePath ),
        "external camera response file path to fuse all LDR images together.")
    ("calibrationWeight,w", po::value<std::string>(&calibrationWeightFunction)->default_value(calibrationWeightFunction),
       "weight function type (default, gaussian, triangle, plateau).")
    ("fusionWeight,W", po::value<hdr::EFunctionType>(&fusionWeightFunction)->default_value(fusionWeightFunction),
       "weight function used to fuse all LDR images together (gaussian, triangle, plateau).")
    ("targetExposureImage,e", po::value<std::string>(&target),
      "LDR image at the target exposure for the output HDR image to be centered.")
    ("oversaturatedCorrection,s", po::value<float>(&clampedValueCorrection)->default_value(clampedValueCorrection),
      "oversaturated correction for pixels oversaturated in all images: use 0 for no correction, 0.5 for interior lighting and 1 for outdoor lighting.")
    ("recoverPath", po::value<std::string>(&recoverSourcePath)->default_value(recoverSourcePath),
      "path to write recovered LDR image at the target exposure by applying inverse response on HDR image.");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

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

  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups(1);
  std::vector< std::vector<float> > times(1);
//  std::vector< std::vector<float> > ev(1);
  hdr::rgbCurve response(0);

  std::vector<image::Image<image::RGBfColor>> &ldrImages = ldrImageGroups.at(0);
  std::vector<float> &ldrTimes = times.at(0);
//  std::vector<float> &ldrEv = ev.at(0);

  // read teh input response fiel or set the correct channel quantization according to the calibration method used
  std::size_t channelQuantization;
  if(!inputResponsePath.empty())
  {
    response = hdr::rgbCurve(inputResponsePath);    // use the camera response function set in "responseCalibration", calibrationMethod is set to "none"
    channelQuantization = response.getSize();
  }
  else
  {
    if(calibrationMethod == ECalibrationMethod::GROSSBERG)
      channelQuantization = std::pow(2, 10);  //RAW 10 bit precision, 2^10 values between black and white point
    else
      channelQuantization = std::pow(2, 12); //RAW 12 bit precision, 2^12 values between black and white point
    response.resize(channelQuantization);
  }

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // set the correct weight functions corresponding to the string parameter
  hdr::rgbCurve fusionWeight(channelQuantization);
  fusionWeight.setFunction(fusionWeightFunction);

  hdr::rgbCurve calibrationWeight(channelQuantization);
  std::transform(calibrationWeightFunction.begin(), calibrationWeightFunction.end(), calibrationWeightFunction.begin(), ::tolower);
  if(calibrationWeightFunction == "default")
  {
    switch(calibrationMethod)
    {
    case ECalibrationMethod::LINEAR:      break;
    case ECalibrationMethod::DEBEVEC:     calibrationWeight.setTriangular();
    case ECalibrationMethod::ROBERTSON:   calibrationWeight.setRobertsonWeight();
    case ECalibrationMethod::GROSSBERG:   calibrationWeight.setGaussian();
    }
  }
  else
    calibrationWeight.setFunction(hdr::EFunctionType_stringToEnum(calibrationWeightFunction));

  // get all valid images from input
  std::vector<std::string> inputImagesNames;
  std::vector<std::string> stemImages;
  std::vector<std::string> nameImages;

  const std::string validExtensions(
  // Basic image file extensions
  "jpg|jpeg|png|tiff|tif|"
  // RAW image file extensions:
  "3fr|" // Hasselblad
  "arw|" // Sony
  "crw|cr2|cr3|" // Canon
  "dng|" // Adobe
  "kdc|" // Kodak
  "mrw|" // Minolta
  "nef|nrw|" // Nikon
  "orf|" // Olympus
  "ptx|pef|" // Pentax
  "raf|" // Fuji
  "R3D|" // RED
  "rw2|" // Panasonic
  "srw|" // Samsung
  "x3f" // Sigma
  );
  const std::regex validExtensionsExp("\\.(?:" + validExtensions + ")");

  for(const std::string& entry: imagesFolder)
  {
    if(fs::is_directory(fs::path(entry)))
    {
      for(const auto& file: fs::directory_iterator(fs::path(entry)))
      {
        std::string extension = file.path().extension().string();
        std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
        if(fs::is_regular_file(file.status()) && std::regex_match(extension, validExtensionsExp))
        {
            inputImagesNames.push_back(file.path().string());
            stemImages.push_back(file.path().stem().string());
            nameImages.push_back(file.path().filename().string());
        }
      }
    }
    else
    {
      std::string extension = fs::path(entry).extension().string();
      std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
      if(fs::is_regular_file(fs::path(entry)) && std::regex_match(extension, validExtensionsExp))
      {
        inputImagesNames.push_back(entry);
        stemImages.push_back(fs::path(entry).stem().string());
        nameImages.push_back(fs::path(entry).filename().string());
      }
    }
  }
  if(inputImagesNames.empty())
  {
    ALICEVISION_LOG_ERROR("No valid input images. Please give a list of LDR images or a folder path containing them (accepted formats are: " << validExtensions << ")");
    return EXIT_FAILURE;
  }


  int nbImages = inputImagesNames.size();
  ldrImages.resize(nbImages);
  for(int i=0; i<nbImages; ++i)
  {
    std::string imagePath = inputImagesNames.at(i);
    int w, h;

    std::map<std::string, std::string> metadata;

    image::readImage(imagePath, ldrImages.at(i), image::EImageColorSpace::SRGB);

    image::readImageMetadata(imagePath, w, h, metadata);

    // Debevec and Robertson algorithms use shutter speed as ev value
    // TODO: in the future, we should use EVs instead of just shutter speed.
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

  // we sort the images according to their exposure time
  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups_sorted(1);
  ldrImageGroups_sorted.at(0).resize(nbImages);
  for(int i=0; i<nbImages; ++i)
  {
      std::vector<float>::iterator it = std::find(ldrTimes.begin(), ldrTimes.end(), ldrTimes_sorted.at(i));
      if(it != ldrTimes.end())
      {
          ldrImageGroups_sorted.at(0).at(i) = ldrImages.at(std::distance(ldrTimes.begin(), it));
      }
      else
          ALICEVISION_LOG_ERROR("sorting failed");
  }


  image::Image<image::RGBfColor> image(ldrImages.at(0).Width(), ldrImages.at(0).Height(), false);

  // calculate the response function according to the method given in argument or take the response provided by the user
  if(inputResponsePath.empty())
  {
    switch(calibrationMethod)
    {
      case ECalibrationMethod::LINEAR:
      {
        // set the response function to linear
        response.setLinear();
      }
      break;

      case ECalibrationMethod::DEBEVEC:
      {
        ALICEVISION_LOG_INFO("Debevec calibration");
        const float lambda = channelQuantization * 1.f;
        const int nbPoints = 1000;
        hdr::DebevecCalibrate calibration;
        calibration.process(ldrImageGroups, channelQuantization, times, nbPoints, calibrationWeight, lambda, response);

        response.exponential();
        response.scale();
      }
      break;

      case ECalibrationMethod::ROBERTSON:
      {
        ALICEVISION_LOG_INFO("Robertson calibration");
        hdr::RobertsonCalibrate calibration(40);
        const int nbPoints = 1000000;
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, calibrationWeight, targetTime, response);
        response.scale();
      }
      break;

      case ECalibrationMethod::GROSSBERG:
      {
        ALICEVISION_LOG_INFO("Grossberg calibration");
        const int nbPoints = 1000000;
        hdr::GrossbergCalibrate calibration(5);
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, calibrationWeight, response);
      }
      break;
    }
  }

  ALICEVISION_LOG_INFO("hdr fusion");
  hdr::hdrMerge merge;
  merge.process(ldrImageGroups_sorted.at(0), ldrTimes_sorted, fusionWeight, response, image, targetTime, false, clampedValueCorrection);


  image::writeImage(outputHDRImagePath, image, image::EImageColorSpace::NO_CONVERSION);
  if(!outputResponsePath.empty())   response.write(outputResponsePath);


  // test of recovery of source target image from HDR
  if(!recoverSourcePath.empty())
  {
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
    recoverSourceImage(image, response, channelQuantization, recoverSourcePath, meanVal);
  }

  ALICEVISION_LOG_INFO("Successfull HDR fusion of " << nbImages << " LDR images.");

  return EXIT_SUCCESS;
}


