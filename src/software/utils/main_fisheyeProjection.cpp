// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2016 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/image/convertion.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/camera/camera.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> 

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <boost/regex.hpp>
#include <boost/math/constants/constants.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace oiio = OIIO;
namespace bmf =  boost::math::float_constants;


/**
 * @brief Function to map 3D coordinates onto a 2D image according a spherical projection
 */
namespace SphericalMapping
{
  Vec3 get3DPoint(const Vec2& pos2d, int width, int height)
  {
    const double x = pos2d(0) - width;
    const double y = height/2.0 - pos2d(1);

    const double longitude = M_PI * 2.0 * x / width;  // between -PI and PI
    const double latitude = M_PI * y / height;        // between -PI/2 and PI/2

    const double Px = cos(latitude) * cos(longitude);
    const double Py = cos(latitude) * sin(longitude);
    const double Pz = sin(latitude);

    return Vec3(Px, Py, Pz);
  }

  Vec2 get2DCoordinates(const Vec3& ray, int inSize)
  {
    const double Px = ray(0);
    const double Py = ray(1);
    const double Pz = ray(2);

//    float aperture = 2.0 * std::atan(36.0 / (8.0 * 2.0));
//    float aperture = 2.0 * asin(1.0 / (2.0 * fNumber));

    const float r = 2.f * std::atan2(std::sqrt(Square(Px) + Square(Pz)), Py) / bmf::pi;
    const float theta = atan2(Pz, Px);
    const Vec2 v(r*cos(theta), r*sin(theta));
    return v * inSize/2;
  }
}


float sigmoid(float x, float sigwidth, float sigMid)
{
  return 1.0f / (1.0f + expf(10.0f * ((x - sigMid) / sigwidth)));
}

/**
 * @brief convert metadata "exif:OriginalDateTime" into float and fill a vector with returned value
 * @param[in] dateTime - input string containing date and time of photo shooting
 * @param[out] times - vector containing all times of photo shooting in float
 */
void convertTime(const std::string& dateTime, std::vector<float>& times)
{
  const boost::regex dateTimeExp(
  // Syntax of Exif:DateTimeOriginal tag
      "(?<date>[\\d\\:]*) "   // YYYY:MM:DD date separated with time with a space
      "(?<h>\\d*):"  // HH: hour
      "(?<m>\\d*):"  // MM: minutes
      "(?<s>\\d*)"   // SS: seconds
  );

  // convert time from string to float to sort images by time for future stitching
  boost::smatch what;
  if(boost::regex_search(dateTime, what, dateTimeExp))
  {
      times.push_back(24.0 * std::stof(what["h"]) + 60.0 * std::stof(what["m"]) + std::stof(what["s"]));
  }
}


/**
 * @brief Function to set fisheye images with correct orientation and add an alpha channel
 * @param[in] imageIn - input RGBf fisheye image
 * @param[in] metadata - contains orientation information
 * @param[out] buffer - to store input metadata in output image
 * @param[out] imageAlpha - output RGBAf fisheye image correctly oriented
 */
void setFisheyeImage(image::Image<image::RGBfColor>& imageIn, float blurWidth_param, oiio::ImageBuf& buffer, image::Image<image::RGBAfColor>& imageAlpha)
{
  bool correctOrientation = oiio::ImageBufAlgo::reorient(buffer, buffer);

  if(!correctOrientation)
  {
    ALICEVISION_LOG_ERROR("Can't set correct orientation of image");
  }

  std::size_t width = buffer.spec().width;
  std::size_t height = buffer.spec().height;

  imageIn.resize(width, height, false);
  buffer.get_pixels(buffer.roi(), buffer.spec().format, imageIn.data());

  imageAlpha.resize(width, height, false);

  const float maxRadius = std::min(width, height) * 0.5f;
  const float blurWidth = maxRadius * blurWidth_param;
  const float blurMid = maxRadius - blurWidth/2.f;
  const Vec2i center(width/2.f, height/2.f);

  for(std::size_t x = 0; x < width; ++x)
  {
    for(std::size_t y = 0; y < height; ++y)
    {
      const image::RGBfColor& inPixel = imageIn(y, x);
      image::RGBAfColor& alphaPixel = imageAlpha(y, x);
      const float dist = std::sqrt((x-center(0)) * (x-center(0)) + (y-center(1)) * (y-center(1)));
      if(dist > maxRadius)
      {
        alphaPixel.a() = 0.f;
      }
      else
      {
        alphaPixel.r() = inPixel.r();
        alphaPixel.g() = inPixel.g();
        alphaPixel.b() = inPixel.b();
        alphaPixel.a() = sigmoid(dist, blurWidth, blurMid);
      }
    }
  }
}


/**
 * @brief Project fisheye image into an equirectangular map and merge it to output panorama
 * @param[in] imageIn - input RGBAf fisheye image
 * @param[in] nbImages
 * @param[in] iter - index of input image for merging into panorama
 * @param[in] rotations - contains adjustment rotations on each image set by user
 * @param[out] imageOut - output panorama which is incremented at each call
 */
void fisheyeToEquirectangular(image::Image<image::RGBAfColor>& imageIn, int nbImages, int iter, const std::array<std::vector<double>, 3>& rotations, image::Image<image::RGBAfColor>& imageOut)
{
  std::size_t inWidth = imageIn.Width();
  std::size_t inHeight = imageIn.Height();
  std::size_t inSize = std::min(inWidth, inHeight);

  const double zRotation = double(30.0 - iter * 360.0 / nbImages) + rotations[2].at(iter);
  const double xRotation = double(-abs(90.0 - abs(zRotation))/30.0) + rotations[0].at(iter);
  const double yRotation = rotations[1].at(iter);

  const image::Sampler2d<image::SamplerLinear> sampler;
  for(int j = 0; j < inSize; ++j)
  {
    for(int i = 0; i < 2 * inSize; ++i)
    {
      Vec3 ray = SphericalMapping::get3DPoint(Vec2(i,j), 2*inSize, inSize);

      ray = rotationXYZ(degreeToRadian(xRotation), degreeToRadian(yRotation), degreeToRadian(zRotation)) * ray;
      const Vec2 x = SphericalMapping::get2DCoordinates(ray, inSize);
      const Vec2 xFish(inWidth/2 - x(0), inHeight/2 - x(1));

      const image::RGBAfColor& pixel = sampler(imageIn, xFish(1), xFish(0));
      float alpha = pixel.a();

      imageOut(j, i).r() = pixel.r()*alpha + imageOut(j, i).r()*(1.f-alpha);
      imageOut(j, i).g() = pixel.g()*alpha + imageOut(j, i).g()*(1.f-alpha);
      imageOut(j, i).b() = pixel.b()*alpha + imageOut(j, i).b()*(1.f-alpha);
      imageOut(j, i).a() = pixel.a()*alpha + imageOut(j, i).a()*(1.f-alpha);
    }
  }
}


/**
 * @brief Load input images and call functions to stitch 360° panorama
 * @param[in] imagePaths - input images paths
 * @param[in] metadatas - input metadata for each image
 * @param[in] rotations - contains adjustment rotations on each image set by user
 * @param[out] outputFolder - output folder path to write panorama
 */
void stitchPanorama(const std::vector<std::string>& imagePaths, const std::vector<oiio::ParamValueList>& metadatas, float blurWidth, const std::array<std::vector<double>, 3>& rotations, std::string& outputPath)
{
  int nbImages = imagePaths.size();
  image::Image<image::RGBAfColor> imageOut;
  std::vector<oiio::ImageBuf> buffers(nbImages);
  oiio::ImageBuf& bufferOut = buffers[0];
  std::size_t inSize;

  for(int i=0; i<nbImages; ++i)
  {
    oiio::ImageBuf& buffer = buffers[i];
    image::Image<image::RGBfColor> imageIn;
    image::Image<image::RGBAfColor> imageAlpha;

    ALICEVISION_LOG_INFO("Projecting " << imagePaths[i] << " into equirectangular space");

    image::readImage(imagePaths[i], imageIn, image::EImageColorSpace::LINEAR);
    image::getBufferFromImage(imageIn, buffer);
    buffer.specmod().extra_attribs = metadatas[i];

    setFisheyeImage(imageIn, blurWidth, buffer, imageAlpha);

    if(i == 0)
    {
      inSize = std::min(bufferOut.spec().width, bufferOut.spec().height);
      imageOut.resize(2*inSize, inSize, true, image::RGBAfColor(0.f, 0.f, 0.f, 0.f));
    }

    fisheyeToEquirectangular(imageAlpha, nbImages, i, rotations, imageOut);
  }

  // save equirectangular image with fisheye's metadata
  if(fs::is_directory(fs::path(outputPath)))
  {
    outputPath = (fs::path(outputPath) / ("panorama.exr")).string();
  }
  image::writeImage(outputPath, imageOut, image::EImageColorSpace::AUTO, bufferOut.specmod().extra_attribs);

  ALICEVISION_LOG_INFO("Panorama successfully written as " << outputPath);
}


int aliceVision_main(int argc, char** argv)
{
  // command-line parameters
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::vector<std::string> inputPath;                      // media file path list
  std::string outputFolder;                   // output folder for panorama
  float blurWidth = 0.2f;
  std::vector<double> xRotation;
  std::vector<double> yRotation;
  std::vector<double> zRotation;

  po::options_description allParams("This program is used to stitch multiple fisheye images into an equirectangular 360° panorama\n"
                                    "AliceVision fisheyeProjection");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::vector<std::string>>(&inputPath)->required()->multitoken(),
      "List of fisheye images or a folder containing them.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output panorama folder or complete path");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("blurWidth,b", po::value<float>(&blurWidth)->default_value(blurWidth),
      "Blur width of alpha channel for all fisheye (between 0 and 1), determine the transitions sharpness.")
    ("xRotation,x", po::value<std::vector<double>>(&xRotation)->multitoken(),
      "Angles to rotate each image on axis x : horizontal axis on the panorama.")
    ("yRotation,y", po::value<std::vector<double>>(&yRotation)->multitoken(),
      "Angles to rotate each image on axis y : vertical axis on the panorama.")
    ("zRotation,z", po::value<std::vector<double>>(&zRotation)->multitoken(),
      "Angles to rotate each image on axis z : depth axis on the panorama.");

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
  
  // check output folder and update to its absolute path
  outputFolder = fs::absolute(outputFolder).string();

  std::vector<std::string> imagePaths;
  std::vector<float> times;
  std::vector<oiio::ParamValueList> metadatas;

  for(const std::string& entry: inputPath)
  {
    const fs::path path = fs::absolute(entry);
    if(fs::exists(path) && fs::is_directory(path))
    {
      for(fs::directory_entry& file : boost::make_iterator_range(fs::directory_iterator(path), {}))
      {
        if(image::isSupported(file.path().extension().string()))
        {
          imagePaths.push_back(file.path().string());

          const oiio::ParamValueList metadata = image::readImageMetadata(file.path().string());
          metadatas.push_back(metadata);
          std::string dateTime;
          dateTime = metadata.get_string("Exif:DateTimeOriginal");

          if(dateTime.empty())
          {
            ALICEVISION_LOG_ERROR("Can't sort images : no time metadata in " << file.path().string());
            return EXIT_FAILURE;
          }
          convertTime(dateTime, times);
        }
      }
    }

    else if(fs::exists(path) && image::isSupported(path.extension().string()))
    {
      imagePaths.push_back(path.string());

      oiio::ParamValueList metadata = image::readImageMetadata(entry);
      metadatas.push_back(metadata);
      std::string dateTime;
      dateTime = metadata.get_string("Exif:DateTimeOriginal");
      if(dateTime.empty())
      {
        ALICEVISION_LOG_ERROR("Can't sort images : no time metadata in " << entry);
        return EXIT_FAILURE;
      }
      convertTime(dateTime, times);
    }

    else
    {
      ALICEVISION_LOG_ERROR("Can't find file or folder " << inputPath);
      return EXIT_FAILURE;
    }
  }

  if(imagePaths.empty())
  {
    ALICEVISION_LOG_ERROR("No valid image file found in input folder or paths");
    return EXIT_FAILURE;
  }

  const auto nbImages = imagePaths.size();
  std::vector<float> times_sorted = times;
  std::vector<std::string> imagePaths_sorted;
  std::vector<oiio::ParamValueList> metadatas_sorted;
  xRotation.resize(nbImages, 0.0);
  yRotation.resize(nbImages, 0.0);
  zRotation.resize(nbImages, 0.0);
  const std::array<std::vector<double>, 3> rotations = {xRotation, yRotation, zRotation};

  // sort images according to their metadata "DateTime"
  std::sort(times_sorted.begin(), times_sorted.end());
  for(std::size_t i=0; i<nbImages; ++i)
  {
    std::vector<float>::iterator it = std::find(times.begin(), times.end(), times_sorted[i]);
    if(it != times.end())
    {
      const std::size_t index = std::distance(times.begin(), it);
      imagePaths_sorted.push_back(imagePaths.at(index));
      metadatas_sorted.push_back(metadatas.at(index));
    }
    else
    {
      ALICEVISION_LOG_ERROR("sorting failed");
      return EXIT_FAILURE;
    }
  }

  ALICEVISION_LOG_INFO(nbImages << " file paths found.");

  stitchPanorama(imagePaths_sorted, metadatas_sorted, blurWidth, rotations, outputFolder);

  return EXIT_SUCCESS;
}
