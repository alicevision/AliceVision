// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/lightingEstimation/lightingEstimation.hpp>
#include <aliceVision/image/io.hpp>

#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo_util.h>

#include <boost/program_options.hpp> 
#include <boost/filesystem.hpp>

#include <string>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

enum class EAlbedoEstimation
{
  CONSTANT,
  PICTURE,
  MEDIAN_FILTER,
  BLUR_FILTER
};

/**
 * @brief get informations about each describer type
 * @return String
 */
std::string EAlbedoEstimation_informations()
{
  return  "Albedo estimation method used for light estimation:\n"
          "* constant: constant color.\n"
          "* picture: from original picture.\n"
          "* median_filter: from original picture with median filter.\n"
          "* blur_filter: from original picture with blur filter.\n";
}

/**
 * @brief convert an enum EAlbedoEstimation to its corresponding string
 * @param EAlbedoEstimation
 * @return String
 */
inline std::string EAlbedoEstimation_enumToString(EAlbedoEstimation albedoEstimation)
{
  switch(albedoEstimation)
  {
    case EAlbedoEstimation::CONSTANT:       return "constant";
    case EAlbedoEstimation::PICTURE:        return "picture";
    case EAlbedoEstimation::MEDIAN_FILTER:  return "median_filter";
    case EAlbedoEstimation::BLUR_FILTER:    return "blur_filter";
  }
  throw std::out_of_range("Invalid albedoEstimation enum: " + std::to_string(int(albedoEstimation)));
}

/**
 * @brief convert a string albedoEstimation to its corresponding enum EAlbedoEstimation
 * @param String
 * @return EAlbedoEstimation
 */
inline EAlbedoEstimation EAlbedoEstimation_stringToEnum(const std::string& albedoEstimation)
{
  std::string albedo = albedoEstimation;
  std::transform(albedo.begin(), albedo.end(), albedo.begin(), ::tolower); //tolower

  if(albedo == "constant")       return EAlbedoEstimation::CONSTANT;
  if(albedo == "picture")        return EAlbedoEstimation::PICTURE;
  if(albedo == "median_filter")  return EAlbedoEstimation::MEDIAN_FILTER;
  if(albedo == "blur_filter")    return EAlbedoEstimation::BLUR_FILTER;

  throw std::out_of_range("Invalid albedoEstimation: " + albedoEstimation);
}

inline std::ostream& operator<<(std::ostream& os, EAlbedoEstimation v)
{
    return os << EAlbedoEstimation_enumToString(v);
}

inline std::istream& operator>>(std::istream& in, EAlbedoEstimation& v)
{
    std::string token;
    in >> token;
    v = EAlbedoEstimation_stringToEnum(token);
    return in;
}

enum class ELightingEstimationMode
{
  GLOBAL = 1,
  PER_IMAGE = 2
};

inline std::string ELightingEstimationMode_enumToString(ELightingEstimationMode v)
{
  switch(v)
  {
    case ELightingEstimationMode::GLOBAL:       return "global";
    case ELightingEstimationMode::PER_IMAGE:    return "per_image";
  }
  throw std::out_of_range("Invalid LightEstimationMode enum: " + std::to_string(int(v)));
}

inline ELightingEstimationMode ELightingEstimationMode_stringToEnum(const std::string& v)
{
  std::string vv = v;
  std::transform(vv.begin(), vv.end(), vv.begin(), ::tolower); //tolower

  if(vv == "global")       return ELightingEstimationMode::GLOBAL;
  if(vv == "per_image")    return ELightingEstimationMode::PER_IMAGE;

  throw std::out_of_range("Invalid LightEstimationMode: " + v);
}

inline std::ostream& operator<<(std::ostream& os, ELightingEstimationMode v)
{
    return os << ELightingEstimationMode_enumToString(v);
}

inline std::istream& operator>>(std::istream& in, ELightingEstimationMode& v)
{
    std::string token;
    in >> token;
    v = ELightingEstimationMode_stringToEnum(token);
    return in;
}

enum class ELightingColor {
  Luminance = 1,
  RGB = 3
};

inline std::string ELightingColor_enumToString(ELightingColor v)
{
  switch(v)
  {
    case ELightingColor::RGB:
      return "rgb";
    case ELightingColor::Luminance:
      return "luminance";
  }
  throw std::out_of_range("Invalid LightingColor type Enum: " + std::to_string(int(v)));
}

inline ELightingColor ELightingColor_stringToEnum(const std::string& v)
{
  std::string vv = v;
  std::transform(vv.begin(), vv.end(), vv.begin(), ::tolower); //tolower

  if(vv == "rgb")
    return ELightingColor::RGB;
  if(vv == "luminance")
    return ELightingColor::Luminance;
  throw std::out_of_range("Invalid LightingColor type string " + v);
}

inline std::ostream& operator<<(std::ostream& os, ELightingColor v)
{
    return os << ELightingColor_enumToString(v);
}

inline std::istream& operator>>(std::istream& in, ELightingColor& v)
{
    std::string token;
    in >> token;
    v = ELightingColor_stringToEnum(token);
    return in;
}

void initAlbedo(image::Image<image::RGBfColor>& albedo, const image::Image<image::RGBfColor>& picture, EAlbedoEstimation albedoEstimationMethod, int albedoEstimationFilterSize, const std::string& outputFolder, IndexT viewId)
{
  switch(albedoEstimationMethod)
  {
    case EAlbedoEstimation::CONSTANT:
    {
      albedo.resize(picture.Width(), picture.Height());
      albedo.fill(image::RGBfColor(.5f,.5f,.5f));
    }
    break;
    case EAlbedoEstimation::PICTURE:
    {
      albedo = picture;
    }
    break;
    case EAlbedoEstimation::MEDIAN_FILTER:
    {
      albedo.resize(picture.Width(), picture.Height());
      const oiio::ImageBuf pictureBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 3, oiio::TypeDesc::FLOAT), const_cast<void*>((void*)&picture(0,0)(0)));
      oiio::ImageBuf albedoBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 3, oiio::TypeDesc::FLOAT), albedo.data());
      oiio::ImageBufAlgo::median_filter(albedoBuf, pictureBuf, albedoEstimationFilterSize, albedoEstimationFilterSize);
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo,
                        image::EImageColorSpace::AUTO);
    }
    break;
    case EAlbedoEstimation::BLUR_FILTER:
    {
      albedo.resize(picture.Width(), picture.Height());
      const oiio::ImageBuf pictureBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 3, oiio::TypeDesc::FLOAT), const_cast<void*>((void*)&picture(0,0)(0)));
      oiio::ImageBuf albedoBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 3, oiio::TypeDesc::FLOAT), albedo.data());
      oiio::ImageBuf K;
      oiio::ImageBufAlgo::make_kernel(K, "gaussian", albedoEstimationFilterSize, albedoEstimationFilterSize);
      oiio::ImageBufAlgo::convolve(albedoBuf, pictureBuf, K);
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo,
                        image::EImageColorSpace::AUTO);
    }
    break;
  }
}

void initAlbedo(image::Image<float>& albedo, const image::Image<float>& picture, EAlbedoEstimation albedoEstimationMethod, int albedoEstimationFilterSize, const std::string& outputFolder, IndexT viewId)
{
  switch(albedoEstimationMethod)
  {
    case EAlbedoEstimation::CONSTANT:
    {
      albedo.resize(picture.Width(), picture.Height());
      albedo.fill(.5f);
    }
    break;
    case EAlbedoEstimation::PICTURE:
    {
      albedo = picture;
    }
    break;
    case EAlbedoEstimation::MEDIAN_FILTER:
    {
      albedo.resize(picture.Width(), picture.Height());
      const oiio::ImageBuf pictureBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 1, oiio::TypeDesc::FLOAT), const_cast<float*>(picture.data()));
      oiio::ImageBuf albedoBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 1, oiio::TypeDesc::FLOAT), albedo.data());
      oiio::ImageBufAlgo::median_filter(albedoBuf, pictureBuf, albedoEstimationFilterSize, albedoEstimationFilterSize);
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo,
                        image::EImageColorSpace::AUTO);
    }
    break;
    case EAlbedoEstimation::BLUR_FILTER:
    {
      albedo.resize(picture.Width(), picture.Height());
      const oiio::ImageBuf pictureBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 1, oiio::TypeDesc::FLOAT), const_cast<float*>(picture.data()));
      oiio::ImageBuf albedoBuf(oiio::ImageSpec(picture.Width(), picture.Height(), 1, oiio::TypeDesc::FLOAT), albedo.data());
      oiio::ImageBuf K;
      oiio::ImageBufAlgo::make_kernel(K, "gaussian", albedoEstimationFilterSize, albedoEstimationFilterSize);
      oiio::ImageBufAlgo::convolve(albedoBuf, pictureBuf, K);
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo,
                        image::EImageColorSpace::AUTO);
    }
    break;
  }
}

int main(int argc, char** argv)
{
  system::Timer timer;

  // command-line parameters

  std::string verboseLevel = aliceVision::system::EVerboseLevel_enumToString(aliceVision::system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string depthMapsFilterFolder;
  std::string imagesFolder;
  std::string outputFolder;

  EAlbedoEstimation albedoEstimationMethod = EAlbedoEstimation::CONSTANT;
  ELightingEstimationMode lightEstimationMode = ELightingEstimationMode::PER_IMAGE;

  int albedoEstimationFilterSize = 3;
  ELightingColor lightingColor = ELightingColor::RGB;

  po::options_description allParams("AliceVision lighthingEstimation");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("depthMapsFilterFolder", po::value<std::string>(&depthMapsFilterFolder)->required(),
      "Filtered depth maps folder.")
    ("imagesFolder", po::value<std::string>(&imagesFolder)->required(),
      "Images used for depth map computation.\n"
      "Filename should be the image uid.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Folder for output lighting vector files.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("lightingColor", po::value<ELightingColor>(&lightingColor)->default_value(lightingColor),
      "Lighting color.")
    ("lightingEstimationMode", po::value<ELightingEstimationMode>(&lightEstimationMode)->default_value(lightEstimationMode),
      "Lighting Estimation Mode.")
    ("albedoEstimationName", po::value<EAlbedoEstimation>(&albedoEstimationMethod)->default_value(albedoEstimationMethod),
      EAlbedoEstimation_informations().c_str())
    ("albedoEstimationFilterSize", po::value<int>(&albedoEstimationFilterSize)->default_value(albedoEstimationFilterSize),
      "Albedo filter size for estimation method using filter.");

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

  // set verbose level
  aliceVision::system::Logger::get()->setLogLevel(verboseLevel);

  // read the input SfM scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // initialization
  mvsUtils::MultiViewParams mp(sfmData, imagesFolder, "", depthMapsFilterFolder, false);

  lightingEstimation::LighthingEstimator estimator;

  for(const auto& viewPair : sfmData.getViews())
  {
    const IndexT viewId = viewPair.first;

    const std::string picturePath = mp.getImagePath(mp.getIndexFromViewId(viewId));
    const std::string normalsPath = mvsUtils::getFileNameFromViewId(mp, viewId, mvsUtils::EFileType::normalMap, 0);

    image::Image<image::RGBfColor> normals;
    image::readImage(normalsPath, normals, image::EImageColorSpace::LINEAR);

    if(lightingColor == ELightingColor::Luminance)
    {
      image::Image<float> albedo, picture;
      image::readImage(picturePath, picture, image::EImageColorSpace::LINEAR);

      initAlbedo(albedo, picture, albedoEstimationMethod, albedoEstimationFilterSize, outputFolder, viewId);

      estimator.addImage(albedo, picture, normals);
    }
    else if(lightingColor == ELightingColor::RGB)
    {
      image::Image<image::RGBfColor> albedo, picture;
      image::readImage(picturePath, picture, image::EImageColorSpace::LINEAR);

      initAlbedo(albedo, picture, albedoEstimationMethod, albedoEstimationFilterSize, outputFolder, viewId);

      estimator.addImage(albedo, picture, normals);
    }

    if(lightEstimationMode == ELightingEstimationMode::PER_IMAGE)
    {
      ALICEVISION_LOG_INFO("Solving view: " << viewId << " lighting estimation...");
      lightingEstimation::LightingVector shl;
      estimator.estimateLigthing(shl);
      estimator.clear(); // clear aggregate data

      std::ofstream file((fs::path(outputFolder) / (std::to_string(viewId) + ".shl")).string());
      if(file.is_open())
        file << shl;
    }
    else
    {
      ALICEVISION_LOG_INFO("View: " << viewId << " added to the problem.");
    }
  }

  if(lightEstimationMode == ELightingEstimationMode::GLOBAL)
  {
    ALICEVISION_LOG_INFO("Solving global scene lighting estimation...");
    lightingEstimation::LightingVector shl;
    estimator.estimateLigthing(shl);

    std::ofstream file((fs::path(outputFolder) / ("global.shl")).string());
    if(file.is_open())
      file << shl;
  }

  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
  return EXIT_SUCCESS;
}
