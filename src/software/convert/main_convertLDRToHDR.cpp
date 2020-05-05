// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <OpenImageIO/imagebufalgo.h>

/*SFMData*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/*HDR Related*/
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/RobertsonCalibrate.hpp>
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/GrossbergCalibrate.hpp>
#include <aliceVision/hdr/emorCurve.hpp>
#include <aliceVision/hdr/LaguerreBACalibration.hpp>

/*Command line parameters*/
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

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
    GROSSBERG,
    LAGUERRE,
};

/**
* @brief convert an enum ECalibrationMethod to its corresponding string
* @param ECalibrationMethod
* @return String
*/
inline std::string ECalibrationMethod_enumToString(const ECalibrationMethod calibrationMethod)
{
    switch (calibrationMethod)
    {
    case ECalibrationMethod::LINEAR:      return "linear";
    case ECalibrationMethod::ROBERTSON:   return "robertson";
    case ECalibrationMethod::DEBEVEC:     return "debevec";
    case ECalibrationMethod::GROSSBERG:   return "grossberg";
    case ECalibrationMethod::LAGUERRE:    return "laguerre";
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

    if (methodName == "linear")      return ECalibrationMethod::LINEAR;
    if (methodName == "robertson")   return ECalibrationMethod::ROBERTSON;
    if (methodName == "debevec")     return ECalibrationMethod::DEBEVEC;
    if (methodName == "grossberg")   return ECalibrationMethod::GROSSBERG;
    if (methodName == "laguerre")    return ECalibrationMethod::LAGUERRE;

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

Eigen::Matrix3d getRotationForCode(int code) {

  Eigen::Matrix3d R_metadata = Eigen::Matrix3d::Identity();

  switch (code)
  {
  case 0:
    R_metadata = Eigen::Matrix3d::Identity();
    break;
  case 3:
    R_metadata = Eigen::AngleAxisd(M_PI, Vec3(0,0,1));
    break;
  case 5:
    R_metadata = Eigen::AngleAxisd(-M_PI_2, Vec3(0,0,1));
    break;  
  case 6:
    R_metadata = Eigen::AngleAxisd(M_PI_2, Vec3(0,0,1));
    break;
  default:
    ALICEVISION_LOG_ERROR("RotationCode flip is not valid ?");
    break;
  }

  return R_metadata;
}

int main(int argc, char * argv[])
{
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmInputDataFilename = "";
  std::string sfmOutputDataFilename = "";
  int nbBrackets = 0;
  ECalibrationMethod calibrationMethod = ECalibrationMethod::LINEAR;
  float highlightCorrectionFactor = 1.0f;
  float highlightTargetLux = 120000.0f;
  bool fisheye = false;
  int channelQuantizationPower = 10;
  int calibrationNbPoints = 0;
  int calibrationDownscale = 4;
  bool refineExposures = false;
  bool byPass = false;

  std::string calibrationWeightFunction = "default";
  hdr::EFunctionType fusionWeightFunction = hdr::EFunctionType::GAUSSIAN;

  // Command line parameters
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaExternalInfo");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
    ;

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("nbBrackets,b", po::value<int>(&nbBrackets)->default_value(nbBrackets), "bracket count per HDR image (0 means automatic).")
    ("calibrationMethod,m", po::value<ECalibrationMethod>(&calibrationMethod)->default_value(calibrationMethod),
        "Name of method used for camera calibration: linear, robertson (slow), debevec, grossberg, laguerre.")
    ("highlightTargetLux", po::value<float>(&highlightTargetLux)->default_value(highlightTargetLux),
        "Highlights maximum luminance.")
    ("highlightCorrectionFactor", po::value<float>(&highlightCorrectionFactor)->default_value(highlightCorrectionFactor),
        "float value between 0 and 1 to correct clamped highlights in dynamic range: use 0 for no correction, 1 for full correction to maxLuminance.")
    ("fisheyeLens,f", po::value<bool>(&fisheye)->default_value(fisheye),
        "Set to 1 if images are taken with a fisheye lens and to 0 if not. Default value is set to 1.")
    ("byPass", po::value<bool>(&byPass)->default_value(byPass),
        "bypass HDR creation and use medium bracket as input for next steps")
    ("channelQuantizationPower", po::value<int>(&channelQuantizationPower)->default_value(channelQuantizationPower),
        "Quantization level like 8 bits or 10 bits.")      
    ("calibrationWeight,w", po::value<std::string>(&calibrationWeightFunction)->default_value(calibrationWeightFunction),
        "Weight function used to calibrate camera response (default depends on the calibration method, gaussian, triangle, plateau).")
    ("fusionWeight,W", po::value<hdr::EFunctionType>(&fusionWeightFunction)->default_value(fusionWeightFunction),
        "Weight function used to fuse all LDR images together (gaussian, triangle, plateau).")
    ("calibrationNbPoints", po::value<int>(&calibrationNbPoints)->default_value(calibrationNbPoints),
        "Number of points used to calibrate (Use 0 for automatic selection based on the calibration method).")
    ("calibrationDownscale", po::value<int>(&calibrationDownscale)->default_value(calibrationDownscale),
        "Image downscale used to calibration the response function.")
    ("calibrationRefineExposures", po::value<bool>(&refineExposures)->default_value(refineExposures),
        "Refine exposures provided by metadata (shutter speed, f-number, iso). Only available for 'laguerre' calibration method. Default value is set to 0.")
    ;

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");
  
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

  system::Logger::get()->setLogLevel(verboseLevel);

  // Analyze path
  boost::filesystem::path path(sfmOutputDataFilename);
  std::string outputPath = path.parent_path().string();

  // Read sfm data
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  size_t countImages = sfmData.getViews().size();
  if (countImages == 0)
  {
    ALICEVISION_LOG_ERROR("The input SfMData contains no input !");
    return EXIT_FAILURE;
  }

  if (nbBrackets > 0 && countImages % nbBrackets != 0)
  {
    ALICEVISION_LOG_ERROR("The input SfMData file is not compatible with the number of brackets.");
    return EXIT_FAILURE;
  }

  // Make sure there is only one kind of image in dataset
  if (sfmData.getIntrinsics().size() > 2)
  {
    ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
    return EXIT_FAILURE;
  }

  if (sfmData.getIntrinsics().size() > 1) {
    
    unsigned int refw = sfmData.getIntrinsics().begin()->second->w();
    unsigned int refh = sfmData.getIntrinsics().begin()->second->h();

    for (auto item : sfmData.getIntrinsics()) {
      if (item.second->w() == refw && item.second->h() == refh) {
        continue;
      }

      if (item.second->w() == refh && item.second->h() == refw) {
        continue;
      }

      ALICEVISION_LOG_ERROR("Multiple intrinsics : Different kind of images in dataset");
      return EXIT_FAILURE;
    }
  }
  
  sfmData::Views & views = sfmData.getViews();

  /* Read the flip values from metadata, or create it if necessary*/
  std::map<int, size_t> count_flips;
  for (const auto & v : views) {
    if (v.second->hasMetadata("raw:flip")) {
      std::string str = v.second->getMetadata("raw:flip");
      int flip_code = std::stoi(str);
      count_flips[flip_code]++;
    }
    else {
      /* Add fake raw:flip if needed */
      int width = v.second->getWidth();
      int height = v.second->getHeight();
      if (width > height) {
        v.second->addMetadata("raw:flip", "0");
        count_flips[0]++;
      } else {
        v.second->addMetadata("raw:flip", "5");
        count_flips[5]++;
      }
    }
  }
  
  if (count_flips.size() <= 1 && sfmData.getIntrinsics().size() == 2) {
    ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found, count flips: " << count_flips.size() << ")");
    return EXIT_FAILURE;
  }


  /*Decide which rotation is the most used*/
  int max_flip = -1; 
  size_t max_count = 0;
  for (auto item : count_flips) {
    if (item.second > max_count) {
      max_flip = item.first;
      max_count = item.second;
    }
  }

  /*Get the intrinsic of the best flip*/
  IndexT refIntrinsic = UndefinedIndexT;
  for (auto & v : views)
  {
    /* Now, all views have raw:flip */
    std::string str = v.second->getMetadata("raw:flip");
    int flip_code = std::stoi(str);

    if (flip_code == max_flip) {
      IndexT intid = v.second->getIntrinsicId();
      if (refIntrinsic != intid && refIntrinsic != UndefinedIndexT) {
        ALICEVISION_LOG_ERROR("Multiple intrinsics for the correct flip code !");
        return EXIT_FAILURE;
      }

      refIntrinsic = intid;
    }
  }

  for (sfmData::Intrinsics::iterator it = sfmData.getIntrinsics().begin(); it != sfmData.getIntrinsics().end(); it++) {
    if (it->first != refIntrinsic) {
      it = sfmData.getIntrinsics().erase(it);
    }
  }

  for (auto & v : views)
  {
    /* Now, all views have raw:flip */
    std::string str = v.second->getMetadata("raw:flip");
    int flip_code = std::stoi(str);

    if (flip_code == max_flip) {
      continue;
    }

    if (refIntrinsic != v.second->getIntrinsicId()) {
      v.second->setIntrinsicId(refIntrinsic);
    }

    Eigen::Matrix3d R = getRotationForCode(flip_code) * getRotationForCode(max_flip).transpose();
    Eigen::AngleAxisd aa(R);
    Eigen::Vector3d axis = aa.axis();
    double angle = aa.angle();

    if (axis(2) < -0.99) {
      axis(2) = 1.0;
      angle = -angle;
    }

    /*Prepare output file*/
    image::Image<image::RGBfColor> output;
    boost::filesystem::path origImgPath(v.second->getImagePath());
    std::string origFilename = origImgPath.stem().string();
    std::string rotatedImagePath = (fs::path(outputPath) / (origFilename + ".exr")).string();
    oiio::ParamValueList metadata = image::readImageMetadata(v.second->getImagePath());

    /*Read input file*/
    image::Image<image::RGBfColor> originalImage;
    image::readImage(v.second->getImagePath(), originalImage, image::EImageColorSpace::LINEAR);
    oiio::ImageBuf bufInput(oiio::ImageSpec(originalImage.Width(), originalImage.Height(), 3, oiio::TypeDesc::FLOAT), originalImage.data());

    /*Find the correct operation to perform*/
    bool validTransform = false;
    if (axis(2) > 0.99) {
      if (std::abs(angle - M_PI_2) < 1e-4) {
        validTransform = true;
        output.resize(originalImage.Height(), originalImage.Width());
        oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT), output.data());
        oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
      }
      else if (std::abs(angle + M_PI_2) < 1e-4) {
        validTransform = true;
        output.resize(originalImage.Height(), originalImage.Width());
        oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT), output.data());
        oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
      }
      else if (std::abs(std::abs(angle) - M_PI) < 1e-4) {
        validTransform = true;
        output.resize(originalImage.Width(), originalImage.Height());
        oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT), output.data());
        oiio::ImageBufAlgo::rotate180(bufOutput, bufInput);
      }
    }
    
    if (validTransform == false) {
      ALICEVISION_LOG_ERROR("Unrecognized intermediate transformation : ");
      ALICEVISION_LOG_ERROR(axis.transpose());
      ALICEVISION_LOG_ERROR(angle);
      return EXIT_FAILURE;
    }
    
    
    image::writeImage(rotatedImagePath, output, image::EImageColorSpace::AUTO, metadata);
    v.second->setWidth(output.Width());
    v.second->setHeight(output.Height());
    v.second->setImagePath(rotatedImagePath);
  }
  

  // Order views by their image names (without path and extension to make sure we handle rotated images)
  std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
  for (auto & viewIt: sfmData.getViews())
  {
    viewsOrderedByName.push_back(viewIt.second);
  }
  std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
    if (a == nullptr || b == nullptr) return true;
    
    boost::filesystem::path path_a(a->getImagePath());
    boost::filesystem::path path_b(b->getImagePath());

    return (path_a.stem().string() < path_b.stem().string());
  });

  {
    // Print a warning if the aperture changes.
    std::set<float> fnumbers;
    for (auto & view : viewsOrderedByName)
    {
      fnumbers.insert(view->getMetadataFNumber());
    }
    if(fnumbers.size() != 1)
    {
      ALICEVISION_LOG_WARNING("Different apertures amongst the dataset. For correct HDR, you should only change the shutter speed (and eventually the ISO).");
      ALICEVISION_LOG_WARNING("Used f-numbers:");
      for (auto f : fnumbers)
      {
        ALICEVISION_LOG_WARNING(" * " << f);
      }
    }
  }

  // Make groups
  std::vector<std::vector<std::shared_ptr<sfmData::View>>> groupedViews;
  {
      std::vector<std::shared_ptr<sfmData::View>> group;
      std::vector<float> exposures;
      for (auto & view : viewsOrderedByName)
      {
          if (nbBrackets > 0)
          {
              group.push_back(view);
              if (group.size() == nbBrackets)
              {
                  groupedViews.push_back(group);
                  group.clear();
              }
          }
          else
          {
              // Automatically determines the number of brackets
              float exp = view->getCameraExposureSetting();
              if (!exposures.empty() && exp != exposures.back() && exp == exposures.front())
              {
                  groupedViews.push_back(group);
                  group.clear();
                  exposures.clear();
              }
              exposures.push_back(exp);
              group.push_back(view);
          }
      }
      if (!group.empty())
          groupedViews.push_back(group);
  }
  if (nbBrackets <= 0)
  {
      std::set<std::size_t> sizeOfGroups;
      for (auto & group : groupedViews)
      {
          sizeOfGroups.insert(group.size());
      }
      if (sizeOfGroups.size() == 1)
      {
          ALICEVISION_LOG_INFO("Number of brackets automatically detected: " << *sizeOfGroups.begin() << ". It will generate " << groupedViews.size() << " hdr images.");
      }
      else
      {
          ALICEVISION_LOG_ERROR("Exposure groups do not have a consistent number of brackets.");
      }
  }
  else if (nbBrackets == 1) {
    byPass = true;
  }

  std::vector<std::shared_ptr<sfmData::View>> targetViews;
  for (auto & group : groupedViews)
  {
    // Sort all images by exposure time
    std::sort(group.begin(), group.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
      if (a == nullptr || b == nullptr) return true;
      return (a->getCameraExposureSetting() < b->getCameraExposureSetting());
    });

    // Target views are the middle exposed views
    // For add number, there is no ambiguity on the middle image.
    // For even number, we arbitrarily choose the more exposed view.
    const int middleIndex = group.size() / 2;

    targetViews.push_back(group[middleIndex]);
  }

  // Build camera exposure table
  std::vector<std::vector<float>> groupedExposures;
  for (int i = 0; i < groupedViews.size(); i++)
  {
    const std::vector<std::shared_ptr<sfmData::View>> & group = groupedViews[i];
    std::vector<float> exposures;

    for (int j = 0; j < group.size(); j++)
    {
      float etime = group[j]->getCameraExposureSetting();
      exposures.push_back(etime);
    }
    groupedExposures.push_back(exposures); 
  }

  // Build table of file names
  std::vector<std::vector<std::string>> groupedFilenames;
  for (int i = 0; i < groupedViews.size(); i++)
  {
    const std::vector<std::shared_ptr<sfmData::View>> & group = groupedViews[i];

    std::vector<std::string> filenames;

    for (int j = 0; j < group.size(); j++)
    {
      filenames.push_back(group[j]->getImagePath());
    }

    groupedFilenames.push_back(filenames);
  }


  sfmData::SfMData outputSfm;
  sfmData::Views & vs = outputSfm.getViews();
  outputSfm.getIntrinsics() = sfmData.getIntrinsics();

  /*If bypass, simply use central bracket*/
  if (byPass) {
    for(int g = 0; g < groupedFilenames.size(); ++g)
    {
      vs[targetViews[g]->getViewId()] = targetViews[g];
    }

    // Export output sfmData
    if (!sfmDataIO::Save(outputSfm, sfmOutputDataFilename, sfmDataIO::ESfMData::ALL))
    {
      ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
      return EXIT_FAILURE;
    } 

    return EXIT_SUCCESS;
  }
  

  size_t channelQuantization = std::pow(2, channelQuantizationPower);
  // set the correct weight functions corresponding to the string parameter
  hdr::rgbCurve calibrationWeight(channelQuantization);
  std::transform(calibrationWeightFunction.begin(), calibrationWeightFunction.end(), calibrationWeightFunction.begin(), ::tolower);
  std::string calibrationWeightFunctionV = calibrationWeightFunction;
  if (calibrationWeightFunction == "default")
  {
    switch (calibrationMethod)
    {
      case ECalibrationMethod::LINEAR:      break;
      case ECalibrationMethod::DEBEVEC:     calibrationWeight.setTriangular(); calibrationWeightFunctionV="triangular"; break;
      case ECalibrationMethod::ROBERTSON:   calibrationWeight.setRobertsonWeight(); calibrationWeightFunctionV = "robertsonWeight"; break;
      case ECalibrationMethod::GROSSBERG:   break;
      case ECalibrationMethod::LAGUERRE:    break;
    }
  }
  else
  {
    calibrationWeight.setFunction(hdr::EFunctionType_stringToEnum(calibrationWeightFunction));
  }

  {
      std::string methodName = calibrationWeightFunctionV;
      std::string outputHtmlPath = (fs::path(outputPath) / (std::string("calibration_weight_") + methodName + std::string(".html"))).string();

      calibrationWeight.writeHtml(outputHtmlPath, "Calibration weight: " + methodName);
  }

  hdr::rgbCurve response(channelQuantization);

  const float lambda = 0.1f;

  // calculate the response function according to the method given in argument or take the response provided by the user
  {
      switch (calibrationMethod)
      {
      case ECalibrationMethod::LINEAR:
      {
          // set the response function to linear
          response.setLinear();

          {
              hdr::rgbCurve r = response;
              r.applyGamma(2.2);
              std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
              std::string outputResponsePath = (fs::path(outputPath) / (std::string("response_log_") + methodName + std::string(".csv"))).string();
              std::string outputResponsePathHtml = (fs::path(outputPath) / (std::string("response_log_") + methodName + std::string(".html"))).string();

              r.write(outputResponsePath);
              r.writeHtml(outputResponsePathHtml, "Camera Response Curve " + methodName);
              ALICEVISION_LOG_INFO("Camera response function written as " << outputResponsePath);
          }
      }
      break;
      case ECalibrationMethod::DEBEVEC:
      {
          ALICEVISION_LOG_INFO("Debevec calibration");
          const float lambda = channelQuantization * 1.f;
          if(calibrationNbPoints <= 0)
              calibrationNbPoints = 1000000;
          hdr::DebevecCalibrate calibration;
          calibration.process(groupedFilenames, channelQuantization, groupedExposures, calibrationNbPoints, calibrationDownscale, fisheye, calibrationWeight, lambda, response);

          {
              std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
              std::string outputResponsePath = (fs::path(outputPath) / (std::string("response_log_") + methodName + std::string(".csv"))).string();
              std::string outputResponsePathHtml = (fs::path(outputPath) / (std::string("response_log_") + methodName + std::string(".html"))).string();

              response.write(outputResponsePath);
              response.writeHtml(outputResponsePathHtml, "Camera Response Curve " + methodName);
              ALICEVISION_LOG_INFO("Camera response function written as " << outputResponsePath);
          }

          {
              hdr::rgbCurve r = response;
              r.applyGammaInv(2.2);
              std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
              std::string outputResponsePath = (fs::path(outputPath) / (std::string("response_invGamma_") + methodName + std::string(".csv"))).string();
              std::string outputResponsePathHtml = (fs::path(outputPath) / (std::string("response_invGamma_") + methodName + std::string(".html"))).string();

              r.write(outputResponsePath);
              r.writeHtml(outputResponsePathHtml, "Camera Response Curve " + methodName);
              ALICEVISION_LOG_INFO("Camera response function written as " << outputResponsePath);
          }

          response.exponential();
          response.scale();
      }
      break;
      case ECalibrationMethod::ROBERTSON:
      {
          /*
          ALICEVISION_LOG_INFO("Robertson calibration");
          hdr::RobertsonCalibrate calibration(10);
          if(calibrationNbPoints <= 0)
            calibrationNbPoints = 1000000;
          calibration.process(groupedFilenames, channelQuantization, groupedExposures, calibrationNbPoints, fisheye, calibrationWeight, response);
          response.scale();
          */
      }
      break;
      case ECalibrationMethod::GROSSBERG:
      {
          ALICEVISION_LOG_INFO("Grossberg calibration");
          if (calibrationNbPoints <= 0)
              calibrationNbPoints = 1000000;
          hdr::GrossbergCalibrate calibration(5);
          calibration.process(groupedFilenames, channelQuantization, groupedExposures, calibrationNbPoints, fisheye, response);
      }
      break;
      case ECalibrationMethod::LAGUERRE:
      {
          ALICEVISION_LOG_INFO("Laguerre calibration");
          if(calibrationNbPoints <= 0)
          {
              calibrationNbPoints = 1000000;
          }

          hdr::LaguerreBACalibration calibration;
          calibration.process(groupedFilenames, channelQuantization, groupedExposures, calibrationNbPoints, calibrationDownscale, fisheye, refineExposures, response);
      }
      break;
      }
  }

  ALICEVISION_LOG_INFO("Calibration done.");

  {
      std::string methodName = ECalibrationMethod_enumToString(calibrationMethod);
      std::string outputResponsePath = (fs::path(outputPath) / (std::string("response_") + methodName + std::string(".csv"))).string();
      std::string outputResponsePathHtml = (fs::path(outputPath) / (std::string("response_") + methodName + std::string(".html"))).string();

      response.write(outputResponsePath);
      response.writeHtml(outputResponsePathHtml, "Camera Response Curve " + methodName);
      ALICEVISION_LOG_INFO("Camera response function written as " << outputResponsePath);
  }

  // HDR Fusion

  hdr::rgbCurve fusionWeight(channelQuantization);
  fusionWeight.setFunction(fusionWeightFunction);

  {
      std::string methodName = EFunctionType_enumToString(fusionWeightFunction);
      std::string outputHtmlPath = (fs::path(outputPath) / (std::string("fusion_weight_") + methodName + std::string(".html"))).string();
      fusionWeight.writeHtml(outputHtmlPath, "Fusion weight: " + methodName);
  }

  image::EImageColorSpace mergeColorspace = image::EImageColorSpace::LINEAR;

  switch (calibrationMethod)
  {
  case ECalibrationMethod::LINEAR:
  case ECalibrationMethod::GROSSBERG:
  case ECalibrationMethod::LAGUERRE:
      mergeColorspace = image::EImageColorSpace::LINEAR;
      break;
  case ECalibrationMethod::DEBEVEC:
  case ECalibrationMethod::ROBERTSON:
      mergeColorspace = image::EImageColorSpace::SRGB;
      break;
  }
  for(int g = 0; g < groupedFilenames.size(); ++g)
  {
    std::vector<image::Image<image::RGBfColor>> images(groupedViews[g].size());
    std::shared_ptr<sfmData::View> targetView = targetViews[g];

    // Load all images of the group
    for (int i = 0; i < images.size(); i++)
    {
      ALICEVISION_LOG_INFO("Load " << groupedFilenames[g][i]);
      image::readImage(groupedFilenames[g][i], images[i], mergeColorspace);
    }

    // Merge HDR images
    hdr::hdrMerge merge;
    float targetCameraExposure = targetView->getCameraExposureSetting();
    image::Image<image::RGBfColor> HDRimage;
    merge.process(images, groupedExposures[g], fusionWeight, response, HDRimage, targetCameraExposure);

    if(highlightCorrectionFactor > 0.0)
    {
      merge.postProcessHighlight(images, groupedExposures[g], fusionWeight, response, HDRimage, targetCameraExposure, highlightCorrectionFactor, highlightTargetLux);
    }

    // Output image file path
    std::stringstream  sstream;
    sstream << "hdr_" << std::setfill('0') << std::setw(4) << g << ".exr";
    std::string hdrImagePath = (fs::path(outputPath) / sstream.str()).string();

    // Write an image with parameters from the target view
    oiio::ParamValueList targetMetadata = image::readImageMetadata(targetView->getImagePath());
    image::writeImage(hdrImagePath, HDRimage, image::EImageColorSpace::AUTO, targetMetadata);

    targetViews[g]->setImagePath(hdrImagePath);
    vs[targetViews[g]->getViewId()] = targetViews[g];
  }

  // Export output sfmData
  if (!sfmDataIO::Save(outputSfm, sfmOutputDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
    return EXIT_FAILURE;
  }  

  return EXIT_SUCCESS;
}
