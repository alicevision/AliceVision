#include <boost/filesystem.hpp>

#define BOOST_TEST_MODULE hdr
#include <boost/test/included/unit_test.hpp>
#include <boost/filesystem.hpp>

#include <aliceVision/image/all.hpp>
#include <aliceVision/system/Logger.hpp>

#include "DebevecCalibrate.hpp"
#include "LaguerreBACalibration.hpp"
#include "GrossbergCalibrate.hpp"
#include "sampling.hpp"

#include <random>
#include <array>

using namespace aliceVision;


bool buildBrackets(std::vector<std::string> & paths, std::vector<float> & times, const hdr::rgbCurve & gt_response) {
  
  
  times = {1.0 / 8000.0, 1.0 / 1600.0, 1.0 / 320.0, 1.0 / 60.0, 1.0/ 13.0, 1.0 / 4.0,  0.5, 0.92};
  
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

  /* Generate a random image */
  image::Image<image::RGBfColor> img(512, 512, true, image::RGBfColor(0.0f));
  for (int i = 0; i < img.Height(); i++) {
    for (int j = 0; j < img.Width(); j++) {
      float r = distribution(generator);
      float g = distribution(generator);
      float b = distribution(generator);
      img(i, j) = image::RGBfColor(r, g, b);
    }
  }

  for (double time : times) {
    image::Image<image::RGBfColor> img_bracket(img.Width(), img.Height());
    for (int i = 0; i < img.Height(); i++) {
      for (int j = 0; j < img.Width(); j++) {
        image::RGBfColor color = img(i, j);
        
        for (int k = 0; k < 3; k++) {
          float radiance = color[k];
          float radiance_dt = radiance * time;
          float val = gt_response(radiance_dt, k);         
          img_bracket(i, j)[k] = val;
        }
      }
    }

    boost::filesystem::path temp = boost::filesystem::temp_directory_path();
    temp /= boost::filesystem::unique_path();
    temp += ".exr";

    ALICEVISION_LOG_INFO("writing to " << temp.string());

    image::writeImage(temp.string(), img_bracket, image::EImageColorSpace::LINEAR);
    paths.push_back(temp.string());
  }

  return true;
}


/*BOOST_AUTO_TEST_CASE(hdr_laguerre)
{
  std::vector<std::string> paths;
  std::vector<float> times;

  const size_t quantization = pow(2, 10);
  hdr::rgbCurve gt_curve(quantization);

  std::array<float, 3> laguerreParams = {-0.2, 0.4, -0.3};
  for (int i = 0; i < quantization; i++) {
    float x = float(i) / float(quantization - 1);
    gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
    gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
    gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
  }

  buildBrackets(paths, times, gt_curve);

  
  std::vector<std::vector<std::string>> all_paths;
  all_paths.push_back(paths);
  std::vector<std::vector<float>> exposures;
  exposures.push_back(times);
  hdr::LaguerreBACalibration calib;
  hdr::rgbCurve response(quantization);
  calib.process(all_paths, quantization, exposures, 500000, 1.0, false, false, response);

  for (int i = 0; i < quantization; i++) {
    float x = float(i) / float(quantization - 1);
    BOOST_CHECK(std::abs(hdr::laguerreFunctionInv(laguerreParams[0], x) - response(x, 0)) < 1e-2);
  }

  for (int imageId = 0; imageId < paths.size() - 1; imageId++) {

    image::Image<image::RGBfColor> imgA, imgB;
    image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
    image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

    BOOST_CHECK(imgA.size() == imgB.size());
    double ratioExposures = times[imageId] / times[imageId + 1];

    bool relatively_similar = true;
    for (int i = 0; i < imgA.Height(); i++) {
      for (int j = 0; j < imgA.Width(); j++) {
        image::RGBfColor Ba = imgA(i, j);
        image::RGBfColor Bb = imgB(i, j);
        for (int k = 0; k < 3; k++) {
          double diff = std::abs(response(Ba(k), k) - ratioExposures * response(Bb(k), k));
            
          if (diff > 1e-3) {
            relatively_similar = false;
            return;
          }
        }
      } 
    }

    BOOST_CHECK(relatively_similar);
  }
}*/

/*
BOOST_AUTO_TEST_CASE(hdr_debevec)
{
  std::vector<std::string> paths;
  std::vector<float> times;

  const size_t quantization = pow(2, 10);
  hdr::rgbCurve gt_curve(quantization);

  std::array<float, 3> laguerreParams = {-0.2, 0.4, -0.3};
  for (int i = 0; i < quantization; i++) {
    float x = float(i) / float(quantization - 1);
    gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
    gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
    gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
  }

  buildBrackets(paths, times, gt_curve);

  std::vector<std::vector<std::string>> all_paths;
  all_paths.push_back(paths);

  std::vector<std::vector<float>> exposures;
  exposures.push_back(times);

  hdr::DebevecCalibrate calib;
  hdr::rgbCurve response(quantization);
  hdr::rgbCurve calibrationWeight(quantization);

  calibrationWeight.setTriangular();
  calib.process(all_paths, quantization, exposures, 10000, 1.0, false, calibrationWeight, 0.1, response);
  response.exponential();
  //response.scale();

  for (int imageId = 0; imageId < paths.size() - 1; imageId++) {
    image::Image<image::RGBfColor> imgA, imgB;
    image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
    image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

    BOOST_CHECK(imgA.size() == imgB.size());
    double ratioExposures = times[imageId] / times[imageId + 1];

    bool relatively_similar = true;
    for (int i = 0; i < imgA.Height(); i++) {
      for (int j = 0; j < imgA.Width(); j++) {
        image::RGBfColor Ba = imgA(i, j);
        image::RGBfColor Bb = imgB(i, j);
        for (int k = 0; k < 1; k++) {
          double diff = std::abs(response(Ba(k), k) - ratioExposures * response(Bb(k), k));
            
          if (diff > 1e-3) {
            std::cout << diff << " " << k << " " << i << " " << j << " " << Ba(k) << " " << Bb(k) << " " << ratioExposures << " " << imageId << std::endl;
            relatively_similar = false;
          }
        }
      }
    }

    BOOST_CHECK(relatively_similar);
  }
}*/


BOOST_AUTO_TEST_CASE(hdr_grossberg)
{
  std::vector<std::string> paths;
  std::vector<float> times;

  const size_t quantization = pow(2, 10);
  hdr::rgbCurve gt_curve(quantization);
  
  
  std::array<float, 3> laguerreParams = {-0.2, -0.2, -0.2};
  for (int i = 0; i < quantization; i++) {
    float x = float(i) / float(quantization - 1);
    gt_curve.getCurve(0)[i] = hdr::laguerreFunction(laguerreParams[0], x);
    gt_curve.getCurve(1)[i] = hdr::laguerreFunction(laguerreParams[1], x);
    gt_curve.getCurve(2)[i] = hdr::laguerreFunction(laguerreParams[2], x);
  }


  buildBrackets(paths, times, gt_curve);

  std::vector<std::vector<std::string>> all_paths;
  std::vector<std::vector<float>> exposures;

  all_paths.push_back(paths);
  exposures.push_back(times);

  hdr::GrossbergCalibrate calib(9);
  hdr::rgbCurve response(quantization);
  const size_t nbPoints = 100000;
  calib.process(all_paths, quantization, exposures, nbPoints, false, response);

  for (int imageId = 0; imageId < paths.size() - 1; imageId++) {

    image::Image<image::RGBfColor> imgA, imgB;
    image::readImage(paths[imageId], imgA, image::EImageColorSpace::LINEAR);
    image::readImage(paths[imageId + 1], imgB, image::EImageColorSpace::LINEAR);

    BOOST_CHECK(imgA.size() == imgB.size());
    double ratioExposures = times[imageId] / times[imageId + 1];

    bool relatively_similar = true;
    for (int i = 0; i < imgA.Height(); i++) {
      for (int j = 0; j < imgA.Width(); j++) {

        image::RGBfColor Ba = imgA(i, j);
        image::RGBfColor Bb = imgB(i, j);

        for (int k = 0; k < 3; k++) {
          double diff = std::abs(response(Ba(k), k) - ratioExposures * response(Bb(k), k));
          
          if (diff > 1e-3) {
            std::cout << diff << " " << k << " " << i << " " << j << " " << Ba(k) << " " << Bb(k) << " " << ratioExposures << " " << imageId << std::endl;
            return ;
            relatively_similar = false;
          }
        }
      }
    }

    BOOST_CHECK(relatively_similar);
  }
}
