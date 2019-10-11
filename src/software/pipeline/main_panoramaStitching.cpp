// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/panorama/ReconstructionEngine_panorama.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

class LaplacianPyramid {
public:
  LaplacianPyramid(const size_t width_base, const size_t height_base) :
    _width_base(width_base), 
    _height_base(height_base)
  {
    size_t min_dim = std::min(_width_base, _height_base);
    _scales = static_cast<size_t>(floor(log2(double(min_dim) / 32.0)));

    /*Create pyramid*/
    size_t new_width = _width_base;
    size_t new_height = _height_base;
    for (int i = 0; i < _scales; i++) {

      _pyramid.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _differences.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _differences_full.push_back(image::Image<image::RGBAfColor>(_width_base, _height_base, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _buffer_1.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _buffer_2.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
    

      new_height /= 2;
      new_width /= 2;
    }
  }

  bool process(const image::Image<image::RGBAfColor> & input) {

    //Build pyramid
    _pyramid[0] = input;
    for (int lvl = 1; lvl < _scales; lvl++) {
      downscale(_pyramid[lvl], _pyramid[lvl - 1]);
    }

    //Differences    
    for (int lvl = 0; lvl < _scales - 1; lvl++) {
      upscale(_buffer_1[lvl], _pyramid[lvl + 1]);
      substract(_differences[lvl], _pyramid[lvl], _buffer_1[lvl]);
    }

    _differences[_differences.size() - 1] = _pyramid[_differences.size() - 1];

    /*Upscale to the max*/
    for (int max_level = 0; max_level < _scales; max_level++) {

      image::Image<image::RGBAfColor> & refOut = _differences_full[max_level];
      
      _buffer_1[max_level] = _differences[max_level];
      for (int lvl = max_level - 1; lvl >= 0; lvl--) {  
        upscale(_buffer_1[lvl], _buffer_1[lvl + 1]);
      }
      refOut = _buffer_1[0];

      
      for (int i = 0; i < refOut.Height(); i++) {
        for (int j = 0; j < refOut.Width(); j++) { 
          if (!(input(i, j).a() > std::numeric_limits<float>::epsilon())) {
            refOut(i, j).r() = 0.0f;
            refOut(i, j).g() = 0.0f;
            refOut(i, j).b() = 0.0f;
            refOut(i, j).a() = 0.0f;
          }
        }
      }
    }

    return true;
  }

  bool rebuild(image::Image<image::RGBAfColor> & output) {

    output = _differences_full[_scales - 1];
    for (int lvl = _scales - 2; lvl >= 0; lvl--) {
      add(output, output, _differences_full[lvl]);
    }

    return true;
  }

  bool blend(const LaplacianPyramid & other, const image::Image<float> & weightMap) {

    for (int lvl = 0; lvl < _scales; lvl++) {

      blend(_differences_full[lvl], _differences_full[lvl], other._differences_full[lvl], weightMap);
    }

    return true;
  }
 
  bool downscale(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & input) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        output(i, j) = input(i * 2, j * 2);
      }
    }

    return true;
  }

  bool upscale(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & input) {

    int width = output.Width();
    int height = output.Height();

    width = 2 * (width / 2);
    height = 2 * (height / 2);


    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {

        output(i, j) = input(i / 2, j / 2);
      }
    }

    return true;
  }

  bool blend(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & inputA, const image::Image<image::RGBAfColor> & inputB, const image::Image<float> & weightMap) {

    

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        float weight = weightMap(i, j);
        float mweight = 1.0f - weight;

  

        const image::RGBAfColor & pixA = inputA(i, j);
        const image::RGBAfColor & pixB = inputB(i, j);

        if (pixA.a() > std::numeric_limits<float>::epsilon()) {

          if (pixB.a() > std::numeric_limits<float>::epsilon()) {
            output(i, j).r() = mweight * pixA.r() +  weight * pixB.r();
            output(i, j).g() = mweight * pixA.g() +  weight * pixB.g();
            output(i, j).b() = mweight * pixA.b() +  weight * pixB.b();
            output(i, j).a() = 1.0f;
          }
          else {
            output(i, j) = pixA;
          }
        }
        else {
          if (pixB.a() > std::numeric_limits<float>::epsilon()) {
            output(i, j) = pixB;
          }
          else {
            output(i, j).r() = 0.0f;
            output(i, j).g() = 0.0f;
            output(i, j).b() = 0.0f;
            output(i, j).a() = 0.0f;
          }
        }

        
      }
    }

    return true;
  }

  bool substract(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & inputA, const image::Image<image::RGBAfColor> & inputB) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        output(i, j).r() = inputA(i, j).r() - inputB(i, j).r();
        output(i, j).g() = inputA(i, j).g() - inputB(i, j).g();
        output(i, j).b() = inputA(i, j).b() - inputB(i, j).b();
        output(i, j).a() = 1.0f;

        if (!(inputA(i, j).a() > std::numeric_limits<float>::epsilon())) {
          output(i, j).r() = 0.0f;
          output(i, j).g() = 0.0f;
          output(i, j).b() = 0.0f;
          output(i, j).a() = 0.0f;
        }
      }
    }

    return true;
  }

  bool add(image::Image<image::RGBAfColor> & output, const image::Image<image::RGBAfColor> & inputA, const image::Image<image::RGBAfColor> & inputB) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

      
        if (inputA(i, j).a() > std::numeric_limits<float>::epsilon()) {
          if (inputB(i, j).a() > std::numeric_limits<float>::epsilon()) {
            output(i, j).r() = inputA(i, j).r() + inputB(i, j).r();
            output(i, j).g() = inputA(i, j).g() + inputB(i, j).g();
            output(i, j).b() = inputA(i, j).b() + inputB(i, j).b();
            output(i, j).a() = 1.0f;
          }
          else {
            output(i, j).r() = inputA(i, j).r();
            output(i, j).g() = inputA(i, j).g();
            output(i, j).b() = inputA(i, j).b();
            output(i, j).a() = 1.0f;
          }
        }
        else {
          output(i, j).r() = 0.0f;
          output(i, j).g() = 0.0f;
          output(i, j).b() = 0.0f;
          output(i, j).a() = 0.0f;
        }
      }
    }

    return true;
  }

public:
  std::vector<image::Image<image::RGBAfColor>> _pyramid;
  std::vector<image::Image<image::RGBAfColor>> _differences;
  std::vector<image::Image<image::RGBAfColor>> _differences_full;
  std::vector<image::Image<image::RGBAfColor>> _buffer_1;
  std::vector<image::Image<image::RGBAfColor>> _buffer_2;

  size_t _width_base;
  size_t _height_base;
  size_t _scales;
};

float sigmoid(float x, float sigwidth, float sigMid)
{
  return 1.0f / (1.0f + expf(10.0f * ((x - sigMid) / sigwidth)));
}

Eigen::Matrix3d rotationBetweenVectors(const Vec3 & v1, const Vec3 & v2) {

  const Vec3 v1n = v1.normalized();
  const Vec3 v2n = v2.normalized();
  const Vec3 cr = v1n.cross(v2n);
  const Vec3 axis = cr.normalized();

  double cosangle = v1n.dot(v2n);
  double sinangle = cr.norm();

  Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
  S(0, 1) = -axis(2);
  S(0, 2) = axis(1);
  S(1, 0) = axis(2);
  S(1, 2) = -axis(0);
  S(2, 0) = -axis(1);
  S(2, 1) = axis(0);

  return Eigen::Matrix3d::Identity() + sinangle * S + (1 - cosangle) * S * S;
}

bool computeDistanceMap(image::Image<int> & distance, const image::Image<image::RGBAfColor> & imageWithMask) {

  int m = imageWithMask.Height();
  int n = imageWithMask.Width();

  int maxval = m * n;

  distance = image::Image<int> (n, m, false); 
  for(int x = 0; x < n; ++x) {

    //A corner is when mask becomes 0
    bool b = (imageWithMask(0, x).a() > std::numeric_limits<float>::epsilon()) ? false : true;
    if (b) {
      distance(0, x) = 0;
    }
    else {
      distance(0, x) = maxval * maxval;
    }

    for (int y = 1; y < m; y++) {
      bool b = (imageWithMask(y, x).a() > std::numeric_limits<float>::epsilon()) ? false : true;
      if (b) {
        distance(y, x) = 0;
      }
      else {          
        distance(y, x) = 1 + distance(y - 1, x);
      }
    }

    for (int y = m - 2; y >= 0; y--) {
      if (distance(y + 1, x) < distance(y, x)) {
        distance(y, x) = 1 + distance(y + 1, x);
      }
    }
  }

  for (int y = 0; y < m; y++) {  
    int q;
    std::map<int, int> s;
    std::map<int, int> t;

    q = 0;
    s[0] = 0;
    t[0] = 0;

    std::function<int (int, int)> f = [distance, y](int x, int i) { 
      int gi = distance(y, i);
      return (x - i)*(x - i) + gi * gi; 
    };

    std::function<int (int, int)> sep = [distance, y](int i, int u) { 
      int gu = distance(y, u);
      int gi = distance(y, i);

      int nom = (u * u) - (i * i) + (gu * gu) - (gi * gi);
      int denom = 2 * (u - i);

      return nom / denom;
    };

    for (int u = 1; u < n; u++) {

      while (q >= 0 && (f(t[q], s[q]) > f(t[q], u))) {
        q = q - 1;
      }

      if (q < 0) {
        q = 0;
        s[0] = u;
      }
      else {
        int w = 1 + sep(s[q], u);
        if (w  < n) {
          q = q + 1;
          s[q] = u;
          t[q] = w;
        }
      }
    }

    for (int u = n - 1; u >= 0; u--) {
      distance(y, u) = f(u, s[q]);
      if (u == t[q]) {
        q = q - 1;
      }
    }
  }

  return true;
}

/**
 * @brief Function to map equirectangular coordinates onto a world unit vector according a spherical projection
 */
namespace SphericalMapping
{
  Vec3 fromEquirectangular(const Vec2 & equirectangular, int width, int height)
  {
    const double latitude = (equirectangular(1) / double(height)) * M_PI  - M_PI_2;
    const double longitude = ((equirectangular(0) / double(width)) * 2.0 * M_PI) - M_PI;

    const double Px = cos(latitude) * sin(longitude);
    const double Py = sin(latitude);
    const double Pz = cos(latitude) * cos(longitude);

    return Vec3(Px, Py, Pz);
  }

  Vec2 toEquirectangular(const Vec3 & spherical, int width, int height) {

    double vertical_angle = asin(spherical(1));
    double horizontal_angle = atan2(spherical(0), spherical(2));

    double latitude =  ((vertical_angle + M_PI_2) / M_PI) * height;
    double longitude =  ((horizontal_angle + M_PI) / (2.0 * M_PI)) * width;

    return Vec2(longitude, latitude);
  }

  Vec2 toLongitudeLatitude(const Vec3 & spherical) {
    
    double latitude = asin(spherical(1));
    double longitude = atan2(spherical(0), spherical(2));

    return Vec2(longitude, latitude);
  }
}

inline std::istream& operator>>(std::istream& in, std::pair<int, int>& out)
{
    in >> out.first;
    in >> out.second;
    return in;
}

bool estimate_panorama_size(const sfmData::SfMData & sfmData, float fisheyeMaskingMargin, std::pair<int, int> & panoramaSize) {

  panoramaSize.first = 512;
  panoramaSize.second = 256;

  image::Image<image::RGBAfColor> BufferCoords(panoramaSize.first, panoramaSize.second, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));
  {
    int imageIndex = 0;
    std::vector<float> determinants;

    for(auto& viewIt: sfmData.getViews())
    {
      IndexT viewId = viewIt.first;
      const sfmData::View& view = *viewIt.second.get();
      if(!sfmData.isPoseAndIntrinsicDefined(&view))
        continue;
        

      const sfmData::CameraPose camPose = sfmData.getPose(view);
      const camera::IntrinsicBase& intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

      const float maxRadius = std::min(intrinsic.w(), intrinsic.h()) * 0.5f * (1.0 - fisheyeMaskingMargin);
      const Vec2i center(intrinsic.w()/2.f, intrinsic.h()/2.f);

      for(int y = 0; y < panoramaSize.second; ++y)
      {
        for(int x = 0; x < panoramaSize.first; ++x)
        {
          BufferCoords(y, x).a() = 0.0;
          // equirectangular to unit vector
          Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x,y), panoramaSize.first, panoramaSize.second);

          if(camPose.getTransform().depth(ray) < 0)
          {
            // point is not in front of the camera
            continue;
          }

          // unit vector to camera
          const Vec2 pix_disto = intrinsic.project(camPose.getTransform(), ray, true);

          if( pix_disto(0) < 0 || pix_disto(0) >= intrinsic.w() || pix_disto(1) < 0 || pix_disto(1) >= intrinsic.h())
          {
            //the pixel is out of the image
            continue;
          }

          const float dist = std::sqrt((pix_disto(0)-center(0)) * (pix_disto(0)-center(0)) + (pix_disto(1)-center(1)) * (pix_disto(1)-center(1)));
          if (dist > maxRadius * 0.8)
          {
            continue;
          }

          BufferCoords(y, x).r() = pix_disto(0);
          BufferCoords(y, x).g() = pix_disto(1);
          BufferCoords(y, x).a() = 1.0;
        }
      }



      for(int y = 1; y < panoramaSize.second - 1; ++y)
      {
        for(int x = 1; x < panoramaSize.first - 1; ++x)
        {
          double x00 = BufferCoords(y, x).r();
          double x10 = BufferCoords(y, x + 1).r();
          double x01 = BufferCoords(y + 1, x).r();
          
          double y00 = BufferCoords(y, x).g();
          double y10 = BufferCoords(y, x + 1).g();
          double y01 = BufferCoords(y + 1, x).g();

          double m00 = BufferCoords(y, x).a();
          double m10 = BufferCoords(y, x + 1).a();
          double m01 = BufferCoords(y + 1, x).a();

          if (m00 != 1.0 || m10  != 1.0 || m01 != 1.0) {
            continue;
          }

          double dxx = x10 - x00;
          double dxy = x01 - x00;
          double dyx = y10 - y00;
          double dyy = y01 - y00;

          float det = std::abs(dxx*dyy - dxy*dyx);
          determinants.push_back(det);
        }
      }

      break;
    }

    std::nth_element(determinants.begin(), determinants.begin() + determinants.size() / 5, determinants.end());
    double scale = sqrt(determinants[determinants.size() / 2]);
    
    panoramaSize.first *= scale;
    panoramaSize.second *= scale;
  }

  return true;
}


int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputPanorama;

  float scaleFactor = 0.2f;
  std::pair<int, int> panoramaSize = {0, 0};
  bool fisheyeMasking = false;
  float fisheyeMaskingMargin = 0.05f;
  float transitionSize = 1000.0f;
  int debugSubsetStart = 0;
  int debugSubsetSize = 0;
  char filename[512];

  po::options_description allParams(
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation.\n"
    "AliceVision PanoramaStitching");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(),
      "Path of the output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("scaleFactor", po::value<float>(&scaleFactor)->default_value(scaleFactor),
      "Scale factor to resize the output resolution (e.g. 0.5 for downscaling to half resolution).")
    ("fisheyeMasking", po::value<bool>(&fisheyeMasking)->default_value(fisheyeMasking),
      "For fisheye images, skip the invalid pixels on the borders.")
    ("fisheyeMaskingMargin", po::value<float>(&fisheyeMaskingMargin)->default_value(fisheyeMaskingMargin),
      "Margin for fisheye images (in percentage of the image).")
    ("transitionSize", po::value<float>(&transitionSize)->default_value(transitionSize),
      "Size of the transition between images (in pixels).")
    ("debugSubsetStart", po::value<int>(&debugSubsetStart)->default_value(debugSubsetStart),
     "For debug only: Index of first image of the subset to merge.")
    ("debugSubsetSize", po::value<int>(&debugSubsetSize)->default_value(debugSubsetSize),
      "For debug only: Number of images in the subset to merge.")
    //("panoramaSize", po::value<std::pair<int, int>>(&panoramaSize)->default_value(panoramaSize),
    //  "Image size of the output panorama image file.")
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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // load input SfMData scene
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS|sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  auto validViews = sfmData.getValidViews();
  int nbCameras = sfmData.getValidViews().size();

  ALICEVISION_LOG_INFO(nbCameras << " loaded from " << sfmDataFilename);
  if(nbCameras == 0)
  {
    ALICEVISION_LOG_ERROR("Failed to get valid cameras from input images.");
    return -1;
  }

  if(panoramaSize.first == 0 || panoramaSize.second == 0)
  {
    estimate_panorama_size(sfmData, fisheyeMaskingMargin, panoramaSize);
  }

  panoramaSize.first *= scaleFactor;
  panoramaSize.second *= scaleFactor;

  /*Make sure panorama size is even */
  panoramaSize.first = 2 * (panoramaSize.first / 2);
  panoramaSize.second = 2 * (panoramaSize.second / 2);

  ALICEVISION_LOG_INFO("Output panorama size: " << panoramaSize.first << ", " << panoramaSize.second);

  // Create panorama buffer
  image::Image<image::RGBAfColor> output;
  image::Image<image::RGBAfColor> imageOut(panoramaSize.first, panoramaSize.second, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));


  int imageIndex = 0;
  for(auto& viewIt: sfmData.getViews())
  {
    
    IndexT viewId = viewIt.first;

    if (!sfmData.isPoseAndIntrinsicDefined(viewId)) {
      continue;
    }

    const sfmData::View& view = *viewIt.second.get();
    if(!sfmData.isPoseAndIntrinsicDefined(&view))
      continue;

    if(debugSubsetSize != 0)
    {
      if(imageIndex < debugSubsetStart)
      {
        ++imageIndex;
        continue;
      }
      else if(imageIndex >= debugSubsetStart + debugSubsetSize)
      {
        break;
      }
    }
    ++imageIndex;
    //if (imageIndex != 0 && imageIndex != 4 && imageIndex != 8) continue;
    

    const sfmData::CameraPose camPose = sfmData.getPose(view);
    const camera::IntrinsicBase& intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());
    std::string imagePath = view.getImagePath();

    // Image RGB
    image::Image<image::RGBfColor> imageIn;
    ALICEVISION_LOG_INFO("Reading " << imagePath);
    //sprintf(filename, "%s.png", imagePath.c_str());
    image::readImage(imagePath, imageIn, image::EImageColorSpace::LINEAR);

    
    
    //image::writeImage(filename, imageIn, image::EImageColorSpace::AUTO);
    ALICEVISION_LOG_INFO(" - " << imageIn.Width() << "x" << imageIn.Height());

    const float maxRadius = std::min(imageIn.Width(), imageIn.Height()) * 0.5f * (1.0 - fisheyeMaskingMargin);
    const float blurWidth = maxRadius * transitionSize;
    const float blurMid = maxRadius - transitionSize/2.f;
    const Vec2i center(imageIn.Width()/2.f, imageIn.Height()/2.f);

    const image::Sampler2d<image::SamplerLinear> sampler;   

    image::Image<image::RGBAfColor> perCameraOutput(panoramaSize.first, panoramaSize.second, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));    

    for(int y = 0; y < panoramaSize.second; ++y)
    {
      for(int x = 0; x < panoramaSize.first; ++x)
      {
        //if (x < 596 || x > 643) continue;
        //if (y < 255 || y > 285) continue;

        // equirectangular to unit vector
        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x,y), panoramaSize.first, panoramaSize.second);    

        //Check that this ray should be visible
        Vec3 transformedRay = camPose.getTransform()(ray);
        if (!intrinsic.isVisibleRay(transformedRay)) {
          continue;
        }

        // unit vector to camera
        const Vec2 pix_disto = intrinsic.project(camPose.getTransform(), ray, true);

        if( pix_disto(0) < 0 || pix_disto(0) >= imageIn.Width() - 1 || pix_disto(1) < 0 || pix_disto(1) >= imageIn.Height() - 1)
        {
          // the pixel is out of the image
          continue;
        }

        float contribution = 1.0f;
        if(fisheyeMasking)
        {
          const float dist = std::sqrt((pix_disto(0)-center(0)) * (pix_disto(0)-center(0)) + (pix_disto(1)-center(1)) * (pix_disto(1)-center(1)));
          if(dist > maxRadius)
          {
              // contribution = 0.0f;
              continue;
          }
          else
          {
              contribution = sigmoid(dist, transitionSize, blurMid);
          }
        }       

        const image::RGBfColor pixel = sampler(imageIn, pix_disto(1), pix_disto(0));
        if(contribution > 0.0f)
        {
          contribution = 1.0f;
          perCameraOutput(y, x).r() += pixel.r() * contribution;
          perCameraOutput(y, x).g() += pixel.g() * contribution;
          perCameraOutput(y, x).b() += pixel.b() * contribution;
          perCameraOutput(y, x).a() += contribution;
        }
      }
    }   

    

    image::Image<int> distanceMap;
    image::Image<float> weightMap(perCameraOutput.Width(), perCameraOutput.Height());
    computeDistanceMap(distanceMap, perCameraOutput);

    for(int y = 0; y < perCameraOutput.Height(); ++y)
    {
      for(int x = 0; x < perCameraOutput.Width(); ++x)
       {
        int dist = distanceMap(y, x);

        weightMap(y, x) = 0.0f;
        if (dist > 0)
        {
          float fdist = sqrtf(float(dist));
          weightMap(y, x) = 1.0f - sigmoid(fdist, 100, 50);
        }
      }
    }

    LaplacianPyramid pyramid(panoramaSize.first, panoramaSize.second);
    pyramid.process(perCameraOutput);

    LaplacianPyramid blending_pyramid(panoramaSize.first, panoramaSize.second);
    blending_pyramid.process(imageOut);
    blending_pyramid.blend(pyramid, weightMap);
    blending_pyramid.rebuild(imageOut);
    

    sprintf(filename, "%s_blend_%d.exr", outputPanorama.c_str(), imageIndex);
    image::writeImage(filename, imageOut, image::EImageColorSpace::NO_CONVERSION);

    /*for (int i = 0; i < pyramid._scales; i++) {
      sprintf(filename, "%s_diff_%d_%d.exr", outputPanorama.c_str(), imageIndex, i);
      image::writeImage(filename, pyramid._differences_full[i], image::EImageColorSpace::NO_CONVERSION);
    }

    sprintf(filename, "%s_cam_%d.exr", outputPanorama.c_str(), imageIndex);
    image::writeImage(filename, perCameraOutput, image::EImageColorSpace::NO_CONVERSION);*/
  }

  
  /*blending_pyramid.rebuild(imageOut);

  for(int y = 0; y < imageOut.Height(); ++y)
  {
    for(int x = 0; x < imageOut.Width(); ++x)
    {
      image::RGBAfColor& pixel = imageOut(y, x);
      if(pixel.a() > std::numeric_limits<float>::epsilon())
      {
        pixel.r() /= pixel.a();
        pixel.g() /= pixel.a();
        pixel.b() /= pixel.a();
        pixel.a() = 1.0f; // TMP: comment to keep the alpha with the number of contribution for debugging
      }
    }
  }

  image::writeImage(outputPanorama, imageOut, image::EImageColorSpace::LINEAR);*/

  return EXIT_SUCCESS;
}