#pragma once

#include <aliceVision/image/all.hpp>

namespace aliceVision {

class GaussianPyramidNoMask {
public:
  GaussianPyramidNoMask(const size_t width_base, const size_t height_base, const size_t limit_scales = 64);

  bool process(const image::Image<image::RGBfColor> & input);


  bool downscale(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input);

  const size_t getScalesCount() const {
    return _scales;
  }

  const std::vector<image::Image<image::RGBfColor>> & getPyramidColor() const {
    return _pyramid_color;
  }

  std::vector<image::Image<image::RGBfColor>> & getPyramidColor() {
    return _pyramid_color;
  }

protected:
  std::vector<image::Image<image::RGBfColor>> _pyramid_color;
  std::vector<image::Image<image::RGBfColor>> _filter_buffer;
  size_t _width_base;
  size_t _height_base;
  size_t _scales;
};

}