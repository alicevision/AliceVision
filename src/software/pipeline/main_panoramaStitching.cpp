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
    _kernel = oiio::ImageBufAlgo::make_kernel("gaussian", 11.0f, 11.0f);
    _kernel_alphas = oiio::ImageBufAlgo::make_kernel("gaussian", 21.0f, 21.0f);

    size_t min_dim = std::min(_width_base, _height_base);
    size_t scales = static_cast<size_t>(floor(log2(double(min_dim))));
    
    size_t new_width = _width_base;
    size_t new_height = _height_base;
    for (int i = 0; i < scales; i++) {

      _difference_buffers.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _alphas.push_back(image::Image<image::RGBAfColor>(new_width, new_height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f)));
      _work_buffers_1.push_back(image::Image<image::RGBAfColor>(new_width, new_height));
      _work_buffers_2.push_back(image::Image<image::RGBAfColor>(new_width, new_height));
      _work_buffers_3.push_back(image::Image<image::RGBAfColor>(new_width, new_height));

      new_height /= 2;
      new_width /= 2;
    }
  }

  bool process(const image::Image<image::RGBAfColor> & input) {

    
    _work_buffers_1[0] = input;

    for (int i = 0; i < input.Height(); i++) {
      for (int j = 0; j < input.Width(); j++) {
        _alphas[0](i, j).r() = input(i, j).a();
        _alphas[0](i, j).g() = input(i, j).a();
        _alphas[0](i, j).b() = input(i, j).a();
        _alphas[0](i, j).a() = 1.0;

        if (_work_buffers_1[0](i, j).a() > std::numeric_limits<float>::epsilon()) {
          _work_buffers_1[0](i, j).a() = 1.0f;
        }
      }
    }
  

    /*
    Loop over the levels.
    0 being the base level with full resolution
    levels.size() - 1 being the least defined image
    */
    for (int lvl = 0; lvl < _difference_buffers.size() - 1; lvl++) {

      oiio::ImageSpec spec_base(_difference_buffers[lvl].Width(), _difference_buffers[lvl].Height(), 4, oiio::TypeDesc::FLOAT);
      oiio::ImageSpec spec_base_half(_difference_buffers[lvl + 1].Width(), _difference_buffers[lvl + 1].Height(), 4, oiio::TypeDesc::FLOAT);

      oiio::ImageBuf _difference_buffer_oiio(spec_base, (void*)(_difference_buffers[lvl].data()));
      oiio::ImageBuf _work_buffer_1_oiio(spec_base, (void*)(_work_buffers_1[lvl].data()));
      oiio::ImageBuf _work_buffer_2_oiio(spec_base, (void*)(_work_buffers_2[lvl].data()));
      oiio::ImageBuf _work_buffer_3_oiio(spec_base, (void*)(_work_buffers_3[lvl].data()));
      oiio::ImageBuf _alphas_buffer_oiio(spec_base, (void*)(_alphas[lvl].data()));
      oiio::ImageBuf _difference_buffer_half_oiio(spec_base_half, (void*)(_difference_buffers[lvl + 1].data()));
      oiio::ImageBuf _work_buffer_1_half_oiio(spec_base_half, (void*)(_work_buffers_1[lvl + 1].data()));
      oiio::ImageBuf _work_buffer_2_half_oiio(spec_base_half, (void*)(_work_buffers_2[lvl + 1].data()));
      oiio::ImageBuf _alphas_buffer_half_oiio(spec_base_half, (void*)(_alphas[lvl + 1].data()));
      
      
      /*Low pass alpha*/
      oiio::ImageBufAlgo::convolve(_work_buffer_2_oiio, _alphas_buffer_oiio, _kernel_alphas, false);

      /*Subsample alpha*/
      oiio::ImageBufAlgo::resample(_alphas_buffer_half_oiio, _work_buffer_2_oiio, false);

      /*First, low pass the image*/
      oiio::ImageBufAlgo::convolve(_work_buffer_2_oiio, _work_buffer_1_oiio, _kernel, false);
      trimAlpha(_work_buffers_2[lvl]);

      /*Subsample image*/
      oiio::ImageBufAlgo::resample(_work_buffer_1_half_oiio, _work_buffer_2_oiio, false);
      trimAlpha(_work_buffers_1[lvl + 1]);

      /*Upsample back reduced image*/
      oiio::ImageBufAlgo::resample(_work_buffer_2_oiio, _work_buffer_1_half_oiio, false);
      trimAlpha(_work_buffers_2[lvl]);

      /*Low pass upsampled image*/
      oiio::ImageBufAlgo::convolve(_work_buffer_3_oiio, _work_buffer_2_oiio, _kernel, true);
      trimAlpha(_work_buffers_3[lvl]);

      /*Difference*/
      substract(_difference_buffers[lvl], _work_buffers_1[lvl], _work_buffers_3[lvl]);
    }

    _difference_buffers[_difference_buffers.size() - 1] = _work_buffers_1[_work_buffers_1.size() - 1];

    return true;
  }

  bool rebuild(image::Image<image::RGBAfColor> & output) {

    _work_buffers_1[_difference_buffers.size() - 1] = _difference_buffers[_difference_buffers.size() - 1];

    for (int lvl = _difference_buffers.size() - 2; lvl >= 0; lvl--) {

      oiio::ImageSpec spec_base(_difference_buffers[lvl].Width(), _difference_buffers[lvl].Height(), 4, oiio::TypeDesc::FLOAT);
      oiio::ImageSpec spec_base_half(_difference_buffers[lvl + 1].Width(), _difference_buffers[lvl + 1].Height(), 4, oiio::TypeDesc::FLOAT);

      oiio::ImageBuf _work_buffer_1_oiio(spec_base, (void*)(_work_buffers_1[lvl].data()));
      oiio::ImageBuf _work_buffer_2_oiio(spec_base, (void*)(_work_buffers_2[lvl].data()));
      oiio::ImageBuf _work_buffer_1_half_oiio(spec_base_half, (void*)(_work_buffers_1[lvl + 1].data()));
      oiio::ImageBuf _difference_buffer_half_oiio(spec_base_half, (void*)(_difference_buffers[lvl + 1].data()));

      /*Upsample back reduced image*/
      oiio::ImageBufAlgo::resample(_work_buffer_1_oiio, _work_buffer_1_half_oiio, false);
      trimAlpha(_work_buffers_1[lvl]);

      /*Low pass upsampled image*/
      oiio::ImageBufAlgo::convolve(_work_buffer_2_oiio, _work_buffer_1_oiio, _kernel, false);
      trimAlpha(_work_buffers_2[lvl]);

      /*Add with next*/
      add(_work_buffers_1[lvl], _difference_buffers[lvl], _work_buffers_2[lvl]);
    }

    output = _work_buffers_1[0];

    return true;
  }

  bool blend(const LaplacianPyramid & other) {

    if (_difference_buffers.size() != other._difference_buffers.size()) {
      return false;
    }

    for (size_t lvl = 0; lvl < _difference_buffers.size(); lvl++) {

      //_difference_buffers[lvl].fill(image::RGBAfColor(0.0f,0.0f,0.0f,0.0f));

      image::Image<image::RGBAfColor> & img_first = _difference_buffers[lvl];
      const image::Image<image::RGBAfColor> & img_second = other._difference_buffers[lvl];
      const image::Image<image::RGBAfColor> & alpha = other._alphas[lvl];
      
      for (int i = 0; i < img_first.Height(); i++) {
        for (int j = 0; j < img_first.Width(); j++) {
          
          image::RGBAfColor & pix_first = img_first(i, j);
          const image::RGBAfColor & pix_second = img_second(i, j);
          const image::RGBAfColor & pix_alpha = alpha(i, j);

          double weight = 0.5f;
          
          if (pix_second.a() == 0.0) {
            continue;
          }
          
          if (pix_first.a() == 0.0) {
            pix_first.r() = pix_second.r();
            pix_first.g() = pix_second.g();
            pix_first.b() = pix_second.b();
            pix_first.a() = pix_second.a();
            continue;
          }

          double mweight = 1.0 - weight;

          pix_first.r() = mweight * pix_first.r() + weight * pix_second.r();
          pix_first.g() = mweight * pix_first.g() + weight * pix_second.g();
          pix_first.b() = mweight * pix_first.b() + weight * pix_second.b(); 
          pix_first.a() = 1.0f;
        }
      }
    }

    return true;
  }

  void divideByAlpha(image::Image<image::RGBAfColor> & image) {

    for(int y = 0; y < image.Height(); ++y) {
      for(int x = 0; x < image.Width(); ++x) {
        image::RGBAfColor& pixel = image(y, x);
        if(pixel.a() > 0.01f) {
          pixel = pixel / pixel.a();
        }
      }
    }
  }

  void trimAlpha(image::Image<image::RGBAfColor> & image) {

    for(int y = 0; y < image.Height(); ++y) {
      for(int x = 0; x < image.Width(); ++x) {
        image::RGBAfColor& pixel = image(y, x);
        if(pixel.a() != 1.0f) {
          pixel.r() = 0.0;
          pixel.g() = 0.0;
          pixel.b() = 0.0;
          pixel.a() = 0.0;
        }
      }
    }
  }

  void copyMask(image::Image<image::RGBAfColor> & dst, const image::Image<image::RGBAfColor> & src) {

    for(int y = 0; y < src.Height(); ++y) {
      for(int x = 0; x < src.Width(); ++x) {
        const image::RGBAfColor & pixelSrc = src(y, x);
        image::RGBAfColor & pixelDst = dst(y, x);
        
        pixelDst = pixelDst * pixelSrc.a();
      }
    }
  }

  void substract(image::Image<image::RGBAfColor> & result, const image::Image<image::RGBAfColor> & A, const image::Image<image::RGBAfColor> & B) {

    for(int y = 0; y < A.Height(); ++y) {
      for(int x = 0; x < A.Width(); ++x) {
        const image::RGBAfColor & pixelA = A(y, x);
        const image::RGBAfColor & pixelB = B(y, x);
        image::RGBAfColor & pixelR = result(y, x);
        
        if(pixelA.a() > std::numeric_limits<float>::epsilon()) {
          pixelR.r() = pixelA.r() - pixelB.r();
          pixelR.g() = pixelA.g() - pixelB.g();
          pixelR.b() = pixelA.b() - pixelB.b();
          pixelR.a() = 1.0f;
        }
        else {
          pixelR.r() = 0.0f;
          pixelR.g() = 0.0f;
          pixelR.b() = 0.0f;
          pixelR.a() = 0.0f;
        }
      }
    }
  }


  void add(image::Image<image::RGBAfColor> & result, const image::Image<image::RGBAfColor> & A, const image::Image<image::RGBAfColor> & B) {

    for(int y = 0; y < A.Height(); ++y) {
      for(int x = 0; x < A.Width(); ++x) {
        const image::RGBAfColor & pixelA = A(y, x);
        const image::RGBAfColor & pixelB = B(y, x);
        image::RGBAfColor & pixelR = result(y, x);
        
        if(pixelA.a() > std::numeric_limits<float>::epsilon()) {
          pixelR.r() = pixelA.r() + pixelB.r();
          pixelR.g() = pixelA.g() + pixelB.g();
          pixelR.b() = pixelA.b() + pixelB.b();
          pixelR.a() = 1.0f;
        }
        else {
          pixelR.r() = 0.0f;
          pixelR.g() = 0.0f;
          pixelR.b() = 0.0f;
          pixelR.a() = 0.0f;
        }
      }
    }
  }

 

public:
  size_t _width_base;
  size_t _height_base;
  oiio::ImageBuf _kernel;
  oiio::ImageBuf _kernel_alphas;

  std::vector<image::Image<image::RGBAfColor>> _difference_buffers;
  std::vector<image::Image<image::RGBAfColor>> _alphas;
  std::vector<image::Image<image::RGBAfColor>> _work_buffers_1;
  std::vector<image::Image<image::RGBAfColor>> _work_buffers_2;
  std::vector<image::Image<image::RGBAfColor>> _work_buffers_3;
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
  LaplacianPyramid blending_pyramid(panoramaSize.first, panoramaSize.second);
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
  
    

    const sfmData::CameraPose camPose = sfmData.getPose(view);
    const camera::IntrinsicBase& intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());
    std::string imagePath = view.getImagePath();

    // Image RGB
    image::Image<image::RGBfColor> imageIn;
    ALICEVISION_LOG_INFO("Reading " << imagePath);
    image::readImage(imagePath, imageIn, image::EImageColorSpace::LINEAR);
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
        // equirectangular to unit vector
        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x,y), panoramaSize.first, panoramaSize.second);    

        //Check that this ray should be visible
        Vec3 transformedRay = camPose.getTransform()(ray);
        if (!intrinsic.isVisibleRay(transformedRay)) {
          continue;
        }

        // unit vector to camera
        const Vec2 pix_disto = intrinsic.project(camPose.getTransform(), ray, true);

        if( pix_disto(0) < 0 || pix_disto(0) >= imageIn.Width() || pix_disto(1) < 0 || pix_disto(1) >= imageIn.Height())
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
          perCameraOutput(y, x).r() += pixel.r() * contribution;
          perCameraOutput(y, x).g() += pixel.g() * contribution;
          perCameraOutput(y, x).b() += pixel.b() * contribution;
          perCameraOutput(y, x).a() += contribution;
        }
      }
    }   


    LaplacianPyramid pyramid(panoramaSize.first, panoramaSize.second);
    pyramid.process(perCameraOutput);
    blending_pyramid.blend(pyramid);

    /*image::Image<image::RGBAfColor> output;
    pyramid.rebuild(output);*/

    /*blending_pyramid.rebuild(imageOut);
    
    char filename[512];
    sprintf(filename, "%s%d.png", outputPanorama.c_str(), imageIndex);
    image::writeImage(filename, imageOut, image::EImageColorSpace::AUTO);*/
  }

  
  blending_pyramid.rebuild(imageOut);

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

  image::writeImage(outputPanorama, imageOut, image::EImageColorSpace::SRGB);

  return EXIT_SUCCESS;
}