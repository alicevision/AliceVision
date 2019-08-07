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
 * @brief check if given file is regular and matches with authorized extensions
 * @param[in] filepath
 * @return true if valid file
 */
bool isValidImageFile(const fs::path& filepath)
{
  if(!fs::is_regular_file(filepath))
    false;

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

  std::string extension = filepath.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  return std::regex_match(extension, validExtensionsExp);
}


/**
 * @brief get image file paths in folder and add it to output vector if file is valid
 * @param[in] folder - input folder path
 * @param[out] out_imageFiles - vector of images paths
 */
void getImagesGroup(const fs::path& folder, std::vector<std::string>& out_imageFiles)
{

  for(const fs::directory_entry& file: fs::directory_iterator(folder))
  {
    if(isValidImageFile(file.path()))
      out_imageFiles.push_back(file.path().string());
  }
}


/**
 * @brief get all input images paths from input provided by user
 * @param[in] imagesFolder - input folder or list provided by user
 * @param[out] ldrImagesPaths - vector of vector of images paths sorted by group
 */
void getInputPaths(const std::vector<std::string>& imagesFolder, std::vector<std::vector<std::string>>& ldrImagesPaths)
{
    std::vector<std::string> singleFiles;
    bool sub_folders = false;

    for(const std::string& entry: imagesFolder)
    {
      fs::path entryPath = fs::path(entry);
      if(fs::is_directory(entryPath))
      {
        std::vector<std::string> group;
        getImagesGroup(entryPath, group);

        if(group.empty())
        {
          // folder without image, assume it is a folder with sub-folders
          sub_folders = true;
          for(const fs::directory_entry& subEntry: fs::directory_iterator(entryPath))
          {
            fs::path subEntryPath = fs::path(subEntry);
            if(fs::is_directory(subEntryPath));
            {
              std::vector<std::string> subGroup;
              getImagesGroup(subEntryPath, subGroup);
              ldrImagesPaths.push_back(subGroup);
            }
          }
        }
        else
        {
          // folder with images
          ldrImagesPaths.push_back(group);
        }
      }
      // list of images
      else if(fs::is_regular_file(entryPath))
      {
        if(isValidImageFile(entryPath))
          singleFiles.push_back(entry);
      }
      else // is an expression
      {
        // extract folder / expression
        std::string imagesExp = entryPath.stem().string();
        std::string extension = entryPath.extension().string();
        std::size_t pos = imagesExp.find("*");
        if(pos != std::string::npos)
        {
          imagesExp.insert(pos, std::string("."));
        }
        else
        {
          throw std::runtime_error("[ldrToHdr] Invalid expression of input files.");
        }
        std::regex exp("\\.(?:" + imagesExp + "\\" + extension + ")");


        for(const fs::directory_entry& file: fs::directory_iterator(entryPath.parent_path()))
        {
          if(fs::is_regular_file(file) && std::regex_match(file.path().string(), exp))
            if(isValidImageFile(file.path()))
              singleFiles.push_back(file.path().string());
        }

        if(singleFiles.empty())
          throw std::runtime_error("[ldrToHdr] Invalid expression of input files.");
      }
    }

    if(!singleFiles.empty())
    {
      if(!ldrImagesPaths.empty())
      {
        throw std::runtime_error("[ldrToHdr] Cannot mix files and folders in input.");
      }
      ldrImagesPaths.push_back(singleFiles);
    }

    // if folder with sub-folders we need to sort sub-folders in alphabetic order
    if(sub_folders)
    {
      std::vector<std::vector<std::string>> ldrImagesPaths_copy = ldrImagesPaths;
      std::vector<std::string> subFoldersNames;

      for(const std::vector<std::string>& group: ldrImagesPaths)
      {
        fs::path firstPath = fs::path(group.front());
        subFoldersNames.push_back(firstPath.parent_path().string());
      }

      std::vector<std::string> subFoldersNames_sorted = subFoldersNames;
      std::sort(subFoldersNames_sorted.begin(), subFoldersNames_sorted.end());

      for(std::size_t g = 0; g < ldrImagesPaths.size(); ++g)
      {
        std::vector<std::string>::iterator it = std::find(subFoldersNames.begin(), subFoldersNames.end(), subFoldersNames_sorted[g]);
        if(it == subFoldersNames.end())
          ALICEVISION_LOG_ERROR("Cannot sort folers, please store them in alphabetic order.");

        ldrImagesPaths[g] = ldrImagesPaths_copy.at(std::distance(subFoldersNames.begin(), it));
      }
    }
}


/**
 * @brief recreate the source image at the target exposure by applying the inverse camera response function to HDR image
 * and calculate the offset between the mean value of target source image and the recovered image
 * @param[in] hdrImage
 * @param[in] response
 * @param[in] channelQuantization
 * @param[in] path to write the output image
 */
void recoverSourceImage(const image::Image<image::RGBfColor>& hdrImage, hdr::rgbCurve& response, float channelQuantization, float meanVal[], image::Image<image::RGBfColor> &targetRecover)
{
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
    ALICEVISION_LOG_INFO("Offset between target source image and recovered from hdr = " << offset);
}


int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::vector<std::string> imagesFolder;
  std::vector<std::string> outputHDRImagesPath;
  std::string outputResponsePath;
  ECalibrationMethod calibrationMethod = ECalibrationMethod::LINEAR;
  std::string inputResponsePath;
  std::string calibrationWeightFunction = "default";
  hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;
  std::vector<std::string> targets;
  float clampedValueCorrection = 1.f;
  bool fisheye = true;
  std::string recoverSourcePath;

  po::options_description allParams("AliceVision convertLDRToHDR");

  po::options_description requiredParams("Required Parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&imagesFolder)->required()->multitoken(),
      "List of LDR images or a folder containing them (accepted formats are: .jpg .jpeg .png .tif .tiff or RAW image file extensions: .3fr .arw .crw .cr2 .cr3 .dng .kdc .mrw .nef .nrw .orf .ptx .pef .raf .R3D .rw2 .srw .x3f")
    ("output,o", po::value<std::vector<std::string>>(&outputHDRImagesPath)->required()->multitoken(),
      "Output HDR image folders or complete paths.");

  po::options_description optionalParams("Optional Parameters");
  optionalParams.add_options()
    ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod )->default_value(calibrationMethod),
      "Name of method used for camera calibration (linear, robertson -> slow !, debevec, grossberg).")
    ("expandDynamicRange,e", po::value<float>(&clampedValueCorrection)->default_value(clampedValueCorrection),
      "float value between 0 and 1 to correct clamped high values in dynamic range: use 0 for no correction, 0.5 for interior lighting and 1 for outdoor lighting.")
    ("targetExposureImage,t", po::value<std::vector<std::string>>(&targets)->multitoken(),
      "Name of LDR image to center your HDR exposure.")
    ("fisheyeLens,f", po::value<bool>(&fisheye)->default_value(fisheye),
     "Set to 1 if images are taken with a fisheye lens and to 0 if not. Default value is set to 1.")
    ("inputResponse,r", po::value<std::string>(&inputResponsePath),
      "External camera response file to fuse all LDR images together.")
    ("calibrationWeight,w", po::value<std::string>(&calibrationWeightFunction)->default_value(calibrationWeightFunction),
       "Weight function used to calibrate camera response (default depends on the calibration method, gaussian, triangle, plateau).")
    ("fusionWeight,W", po::value<hdr::EFunctionType>(&fusionWeightFunction)->default_value(fusionWeightFunction),
       "Weight function used to fuse all LDR images together (gaussian, triangle, plateau).")
    ("outputResponse", po::value<std::string>(&outputResponsePath),
       "(For debug) Output camera response function folder or complete path.")
    ("recoverPath", po::value<std::string>(&recoverSourcePath)->default_value(recoverSourcePath),
      "(For debug) Folder path for recovering LDR images at the target exposures by applying inverse response on HDR images.");

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  std::vector<std::vector<std::string>> imagesPaths;
  getInputPaths(imagesFolder, imagesPaths);
  std::size_t nbGroups = imagesPaths.size();
  std::size_t nbImages = imagesPaths.front().size();
  for(int g = 0; g < nbGroups; ++g)
  {
    if(imagesPaths[g].size() != nbImages)
    {
      ALICEVISION_LOG_ERROR("All groups should have the same number of LDR images.");
      return EXIT_FAILURE;
    }
  }

  ALICEVISION_LOG_INFO(nbGroups << " group(s) of LDR images found");

  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups(nbGroups);
  std::vector< std::vector< image::Image<image::RGBfColor> > > ldrImageGroups_sorted(nbGroups);
  std::vector< std::vector<float> > times(nbGroups);
  std::vector< std::vector<float> > times_sorted(nbGroups);
  std::vector<float> targetTimes(nbGroups);
  std::vector<oiio::ParamValueList> targetMetadatas(nbGroups);

  hdr::rgbCurve response(0);
  std::size_t channelQuantization;
  image::EImageColorSpace loadColorSpace;

  targets.resize(nbGroups);
  outputHDRImagesPath.resize(nbGroups);

  std::string outputPath = outputHDRImagesPath.front();

  if(fs::is_directory(outputPath))
  {
    if(nbGroups == 1)
      outputHDRImagesPath.front() = (fs::path(outputPath) / ("hdr.exr")).string();
    else
      for(int g = 0; g < nbGroups; ++g)
        outputHDRImagesPath[g] = (fs::path(outputPath) / ("hdr_" + std::to_string(g) + ".exr")).string();
  }
  else if(outputHDRImagesPath.size() != nbGroups)
  {
    ALICEVISION_LOG_ERROR("Please provide one output path for each group of LDR images.");
    return EXIT_FAILURE;
  }

  // read input response file or set the correct channel quantization according to the calibration method used
  if(!inputResponsePath.empty())
  {
    response = hdr::rgbCurve(inputResponsePath);    // use the camera response function set in "responseCalibration", calibrationMethod is set to "none"
    channelQuantization = response.getSize();
  }
  else
  {
    channelQuantization = std::pow(2, 10); //RAW 10 bit precision, 2^10 values between black and white point
    response.resize(channelQuantization);
  }

  // set correct color space according to calibration method
  if(calibrationMethod == ECalibrationMethod::LINEAR)
    loadColorSpace = image::EImageColorSpace::LINEAR;
  else
    loadColorSpace = image::EImageColorSpace::SRGB;

  // force clamped value correction between 0 and 1
  clampedValueCorrection = clamp(clampedValueCorrection, 0.f, 1.f);

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
    case ECalibrationMethod::DEBEVEC:     calibrationWeight.setTriangular();  break;
    case ECalibrationMethod::ROBERTSON:   calibrationWeight.setRobertsonWeight(); break;
    case ECalibrationMethod::GROSSBERG:   break;
    }
  }
  else
    calibrationWeight.setFunction(hdr::EFunctionType_stringToEnum(calibrationWeightFunction));


  for(int g = 0; g < nbGroups; ++g)
  {
    if(nbGroups > 1)
      ALICEVISION_LOG_INFO("Group " << g << " : ");

    std::vector<image::Image<image::RGBfColor>> &ldrImages = ldrImageGroups[g];
    ldrImages.resize(nbImages);
    std::vector<float> &ldrTimes = times[g];
    //  std::vector<float> &ldrEv = ev.at(0);
    std::vector<std::string> &inputImagesNames = imagesPaths[g];

    std::vector<std::string> stemImages(nbImages);
    std::vector<std::string> nameImages(nbImages);
    std::vector<oiio::ParamValueList> metadatas(nbImages);
    std::vector<float> aperture;
    std::vector<std::string> colorSpace;

    for(int i=0; i<nbImages; ++i)
    {
      std::string &imagePath = inputImagesNames[i];
      stemImages[i] = fs::path(imagePath).stem().string();
      nameImages[i] = fs::path(imagePath).filename().string();

      ALICEVISION_LOG_INFO("Reading " << imagePath);

      image::readImage(imagePath, ldrImages[i], loadColorSpace);

      metadatas[i] = image::readImageMetadata(imagePath);

      // Debevec and Robertson algorithms use shutter speed as ev value
      // TODO: in the future, we should use EVs instead of just shutter speed.
      try
      {
  //      const float iso = std::stof(metadata.at("Exif:PhotographicSensitivity"));
  //      const float iso = std::stof(metadata.at("Exif:ISOSpeedRatings"));

       aperture.emplace_back(metadatas[i].get_float("FNumber"));
       ldrTimes.emplace_back(metadatas[i].get_float("ExposureTime"));
       colorSpace.emplace_back(metadatas[i].get_string("oiio:ColorSpace"));

  //      ldrEv.push_back(std::log2(pow(aperture, 2) / shutter) + std::log2(iso/100));
      }
      catch(std::exception& e)
      {
        ALICEVISION_LOG_ERROR("no metadata in images : " << imagePath);
        return EXIT_FAILURE;
      }
    }


    // assert that all images have the same aperture and same color space
    if(std::adjacent_find(aperture.begin(), aperture.end(), std::not_equal_to<float>()) != aperture.end())
    {
      ALICEVISION_LOG_ERROR("Input images have different apertures.");
      return EXIT_FAILURE;
    }
    if(std::adjacent_find(colorSpace.begin(), colorSpace.end(), std::not_equal_to<std::string>()) != colorSpace.end())
    {
      ALICEVISION_LOG_ERROR("Input images have different color spaces.");
      return EXIT_FAILURE;
    }


    std::vector<float> &ldrTimes_sorted = times_sorted[g];
    ldrTimes_sorted = ldrTimes;
    std::sort(ldrTimes_sorted.begin(), ldrTimes_sorted.end());

    // we sort the images according to their exposure time
    ldrImageGroups_sorted[g].resize(nbImages);
    for(int i=0; i<nbImages; ++i)
    {
      std::vector<float>::iterator it = std::find(ldrTimes.begin(), ldrTimes.end(), ldrTimes_sorted.at(i));
      if(it == ldrTimes.end())
      {
        ALICEVISION_LOG_ERROR("sorting failed");
        return EXIT_FAILURE;
      }

      ldrImageGroups_sorted[g].at(i) = ldrImages.at(std::distance(ldrTimes.begin(), it));
    }

    // find target exposure time corresponding to the image name given by user
    if(!targets[g].empty())
    {
      nameImages.insert(nameImages.end(), stemImages.begin(), stemImages.end());
      nameImages.insert(nameImages.end(), inputImagesNames.begin(), inputImagesNames.end());    //search target for filenames only, filenames + extensions and complete paths
      std::vector<std::string>::iterator it = std::find(nameImages.begin(), nameImages.end(), targets[g]);
      if(it == nameImages.end())
      {
        ALICEVISION_CERR("Invalid target name: \"" << targets[g] << "\"");
        return EXIT_FAILURE;
      }

      std::size_t targetIndex = std::distance(nameImages.begin(), it) % nbImages;
      targetMetadatas[g] = metadatas[targetIndex];
      targetTimes[g] = ldrTimes.at(targetIndex);
    }
    else
    {
      targetTimes[g] = ldrTimes_sorted.at(ldrTimes_sorted.size()/2);
      std::size_t targetIndex = std::distance(ldrTimes.begin(), std::find(ldrTimes.begin(), ldrTimes.end(), targetTimes[g]));
      targets[g] = nameImages.at(targetIndex);
      targetMetadatas[g] = metadatas[targetIndex];
    }
  }


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
        const int nbPoints = 10000;
        hdr::DebevecCalibrate calibration;
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, fisheye, calibrationWeight, lambda, response);

        response.exponential();
        response.scale();
      }
      break;

      case ECalibrationMethod::ROBERTSON:
      {
        ALICEVISION_LOG_INFO("Robertson calibration");
        hdr::RobertsonCalibrate calibration(10);
        const int nbPoints = 1000000;
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, fisheye, calibrationWeight, response);
        response.scale();
      }
      break;

      case ECalibrationMethod::GROSSBERG:
      {
        ALICEVISION_LOG_INFO("Grossberg calibration");
        const int nbPoints = 1000000;
        hdr::GrossbergCalibrate calibration(3);
        calibration.process(ldrImageGroups_sorted, channelQuantization, times_sorted, nbPoints, fisheye, response);
      }
      break;
    }
  }

  if(!outputResponsePath.empty())
  {
    std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
    if(fs::is_directory(fs::path(outputResponsePath)))
    {
      outputResponsePath = (fs::path(outputResponsePath) / (std::string("response_") + methodName + std::string(".csv"))).string();
    }

    response.write(outputResponsePath);
    ALICEVISION_LOG_INFO("Camera response function written as " << outputResponsePath);
  }


  for(int g = 0; g < nbGroups; ++g)
  {
    image::Image<image::RGBfColor> HDRimage(ldrImageGroups[g].front().Width(), ldrImageGroups[g].front().Height(), false);

    hdr::hdrMerge merge;
    merge.process(ldrImageGroups_sorted[g], times_sorted[g], fusionWeight, response, HDRimage, targetTimes[g], false, clampedValueCorrection);

    image::writeImage(outputHDRImagesPath[g], HDRimage, image::EImageColorSpace::AUTO, targetMetadatas[g]);

    ALICEVISION_LOG_INFO("Successfull HDR fusion of " << nbImages << " LDR images centered on " << targets[g]);
    ALICEVISION_LOG_INFO("HDR image written as " << outputHDRImagesPath[g]);
  }


  // test of recovery of source target image from HDR
  if(!recoverSourcePath.empty())
  {
    fs::path recoverPath = fs::path(recoverSourcePath);

    for(int g = 0; g < nbGroups; ++g)
    {
      image::Image<image::RGBfColor> HDRimage;
      image::readImage(outputHDRImagesPath[g], HDRimage, image::EImageColorSpace::NO_CONVERSION);

      // calcul of mean value of target images
      std::size_t targetIndex = std::distance(times[g].begin(), std::find(times[g].begin(), times[g].end(), targetTimes[g]));
      float meanVal[3] = {0.f, 0.f, 0.f};
      image::Image<image::RGBfColor> &targetImage = ldrImageGroups[g].at(targetIndex);
      image::Image<image::RGBfColor> targetRecover(targetImage.Width(), targetImage.Height(), false);
      for(std::size_t channel=0; channel<3; ++channel)
      {
        for(std::size_t y = 0; y < targetImage.Height(); ++y)
        {
          for(std::size_t x = 0; x < targetImage.Width(); ++x)
          {
            meanVal[channel] += targetImage(y, x)(channel);
          }
        }
        meanVal[channel] /= targetImage.size();
      }
      recoverSourceImage(HDRimage, response, channelQuantization, meanVal, targetRecover);

      if(nbGroups == 1)
        recoverSourcePath = (recoverPath / (std::string("recovered.exr"))).string();
      else
        recoverSourcePath = (recoverPath / (std::string("recovered_") + std::to_string(g) + std::string(".exr"))).string();

      image::writeImage(recoverSourcePath, targetRecover, image::EImageColorSpace::AUTO);
      ALICEVISION_LOG_INFO("Recovered target source image written as " << recoverSourcePath);
    }
  }

  return EXIT_SUCCESS;
}
