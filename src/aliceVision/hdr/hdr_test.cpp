#include <boost/filesystem.hpp>

#define BOOST_TEST_MODULE hdr
#include <boost/test/included/unit_test.hpp>

#include <aliceVision/image/all.hpp>

#include <random>

using namespace aliceVision;

const std::vector<double> times = {1.0/8000.0, 1.0/1600.0, 1.0/320.0, 1.0/60.0, 1.0/13.0, 1/2.0, 2.0};


BOOST_AUTO_TEST_CASE(hdr)
{
  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(0.0f, 1.0f);

  image::Image<image::RGBfColor> img(512, 512, true, image::RGBfColor(0.0f));
  for (int i = 0; i < 512; i++) {
    for (int j = 0; j < 512; j++) {
      float r = distribution(generator);
      float g = distribution(generator);
      float b = distribution(generator);
      img(i, j) = image::RGBfColor(r, g, b);
    }
  }
}

