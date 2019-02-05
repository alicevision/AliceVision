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
    case ELightingEstimationMode::PER_IMAGE:        return "per_image";
  }
  throw std::out_of_range("Invalid LightEstimationMode enum: " + std::to_string(int(v)));
}

inline ELightingEstimationMode ELightingEstimationMode_stringToEnum(const std::string& v)
{
  std::string vv = v;
  std::transform(vv.begin(), vv.end(), vv.begin(), ::tolower); //tolower

  if(vv == "global")       return ELightingEstimationMode::GLOBAL;
  if(vv == "per_image")        return ELightingEstimationMode::PER_IMAGE;

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
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo);
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
      image::writeImage((fs::path(outputFolder) / (std::to_string(viewId) + "_albedo.jpg")).string(), albedo);
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
  lightingEstimation::ELightingColor lightingColor = lightingEstimation::ELightingColor::RGB;

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
    ("lightingColor", po::value<lightingEstimation::ELightingColor>(&lightingColor)->default_value(lightingColor),
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

  if(lightEstimationMode == ELightingEstimationMode::PER_IMAGE)
  {
    for(const auto& viewPair : sfmData.getViews())
    {
      const IndexT viewId = viewPair.first;

      const std::string picturePath = mp.getImagePath(mp.getIndexFromViewId(viewId));
      const std::string normalsPath = mvsUtils::getFileNameFromViewId(&mp, viewId, mvsUtils::EFileType::normalMap, 0);

      lightingEstimation::LightingVector shl;
      image::Image<image::RGBfColor> picture;
      image::Image<image::RGBfColor> normals;

      image::readImage(picturePath, picture);
      image::readImage(normalsPath, normals);

      image::Image<image::RGBfColor> albedo;
      initAlbedo(albedo, picture, albedoEstimationMethod, albedoEstimationFilterSize, outputFolder, viewId);

      lightingEstimation::estimateLigthing(shl, albedo, picture, normals, lightingColor);

      std::ofstream file((fs::path(outputFolder) / (std::to_string(viewId) + ".shl")).string());
      if(file.is_open())
        file << shl;
    }
  }
  else if(lightEstimationMode == ELightingEstimationMode::GLOBAL)
  {
    using namespace Eigen;
    using namespace aliceVision::lightingEstimation;

    MatrixXf rhoTimesN[3];
    MatrixXf colors[3];

    for(const auto& viewPair : sfmData.getViews())
    {
      const IndexT viewId = viewPair.first;

      const std::string picturePath = mp.getImagePath(mp.getIndexFromViewId(viewId));
      const std::string normalsPath = mvsUtils::getFileNameFromViewId(&mp, viewId, mvsUtils::EFileType::normalMap, 0);

      image::Image<image::RGBfColor> picture;
      image::Image<image::RGBfColor> normals;

      image::readImage(picturePath, picture);
      image::readImage(normalsPath, normals);

      image::Image<image::RGBfColor> albedo;
      initAlbedo(albedo, picture, albedoEstimationMethod, albedoEstimationFilterSize, outputFolder, viewId);

      // Augmented normales
      image::Image<AugmentedNormal> augNormals(normals.cast<AugmentedNormal>());

      const std::size_t nbPixels = albedo.Width() * albedo.Height();

      // estimate Lighting per Channel
      for(std::size_t c = 0; c < 3; ++c)
      {
          // Map albedo, image
          Map<MatrixXf, 0, InnerStride<3>> albedoC((float*)&(albedo(0,0)(c)), nbPixels, 1);
          Map<MatrixXf, 0, InnerStride<3>> pictureC((float*)&(picture(0,0)(c)), nbPixels, 1);

          accumulateImageData(rhoTimesN[c], colors[c], albedoC, pictureC, augNormals);

          ALICEVISION_LOG_INFO("GLOBAL: estimateLigthingLuminance (channel=" << c << "): " << rhoTimesN[c].rows() << "x" << rhoTimesN[c].cols());
      }
    }

    LightingVector shl;
    if(lightingColor == ELightingColor::Luminance)
    {
        MatrixXf all_rhoTimesN;
        MatrixXf all_colors;

        for(std::size_t c = 0; c < 3; ++c)
        {
            all_rhoTimesN.resize(all_rhoTimesN.rows() + rhoTimesN[c].rows(), rhoTimesN[c].cols());
            all_colors.resize(all_colors.rows() + colors[c].rows(), colors[c].cols());

            all_rhoTimesN.bottomLeftCorner(rhoTimesN[c].rows(), rhoTimesN[c].cols()) = rhoTimesN[c];
            all_colors.bottomLeftCorner(colors[c].rows(), colors[c].cols()) = colors[c];
        }

        Eigen::Matrix<float, 9, 1> lightingL;
        estimateLigthing(lightingL, all_rhoTimesN, all_colors);

        // lighting vectors fusion
        shl.col(0) = lightingL;
        shl.col(1) = lightingL;
        shl.col(2) = lightingL;
    }
    else if(lightingColor == ELightingColor::RGB)
    {
        // estimate Lighting per Channel
        for(std::size_t c = 0; c < 3; ++c)
        {
            Eigen::Matrix<float, 9, 1> lightingC;
            estimateLigthing(lightingC, rhoTimesN[c], colors[c]);

            // lighting vectors fusion
            shl.col(c) = lightingC;
        }
    }

    std::ofstream file((fs::path(outputFolder) / ("scene.shl")).string());
    if(file.is_open())
      file << shl;
  }
  else
  {
    throw std::runtime_error("Invalid lightingEstimationMode: " + ELightingEstimationMode_enumToString(lightEstimationMode));
  }

  ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
  return EXIT_SUCCESS;
}
