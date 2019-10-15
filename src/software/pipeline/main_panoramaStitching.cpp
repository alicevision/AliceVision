/**
 * Input and geometry
*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/**
 * Image stuff
 */
#include <aliceVision/image/all.hpp>

/*Logging stuff*/
#include <aliceVision/system/Logger.hpp>

/*Reading command line options*/
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

namespace SphericalMapping
{
  /**
   * Map from equirectangular to spherical coordinates
   * @param equirectangular equirectangular coordinates
   * @param width number of pixels used to represent longitude
   * @param height number of pixels used to represent latitude
   * @return spherical coordinates
   */
  Vec3 fromEquirectangular(const Vec2 & equirectangular, int width, int height)
  {
    const double latitude = (equirectangular(1) / double(height)) * M_PI  - M_PI_2;
    const double longitude = ((equirectangular(0) / double(width)) * 2.0 * M_PI) - M_PI;

    const double Px = cos(latitude) * sin(longitude);
    const double Py = sin(latitude);
    const double Pz = cos(latitude) * cos(longitude);

    return Vec3(Px, Py, Pz);
  }

  /**
   * Map from Spherical to equirectangular coordinates
   * @param spherical spherical coordinates
   * @param width number of pixels used to represent longitude
   * @param height number of pixels used to represent latitude
   * @return equirectangular coordinates
   */
  Vec2 toEquirectangular(const Vec3 & spherical, int width, int height) {

    double vertical_angle = asin(spherical(1));
    double horizontal_angle = atan2(spherical(0), spherical(2));

    double latitude =  ((vertical_angle + M_PI_2) / M_PI) * height;
    double longitude =  ((horizontal_angle + M_PI) / (2.0 * M_PI)) * width;

    return Vec2(longitude, latitude);
  }

  /**
   * Map from Spherical to equirectangular coordinates in radians
   * @param spherical spherical coordinates
   * @return equirectangular coordinates
   */
  Vec2 toLongitudeLatitude(const Vec3 & spherical) {
    
    double latitude = asin(spherical(1));
    double longitude = atan2(spherical(0), spherical(2));

    return Vec2(longitude, latitude);
  }
}

class Pyramid {
public:
  Pyramid(const size_t width_base, const size_t height_base, const size_t limit_scales = 64) :
    _width_base(width_base), 
    _height_base(height_base)
  {
    /**
     * Compute optimal scale
     * The smallest level will be at least of size min_size
     */
    size_t min_dim = std::min(_width_base, _height_base);
    size_t min_size = 32;
    _scales = std::min(limit_scales, static_cast<size_t>(floor(log2(double(min_dim) / float(min_size)))));
    

    /**
     * Create pyramid
     **/
    size_t new_width = _width_base;
    size_t new_height = _height_base;
    for (int i = 0; i < _scales; i++) {

      _pyramid_color.push_back(image::Image<image::RGBfColor>(new_width, new_height, true, image::RGBfColor(0.0f, 0.0f, 0.0f)));
      _pyramid_mask.push_back(image::Image<bool>(new_width, new_height, true, false));
      new_height /= 2;
      new_width /= 2;
    }
  }

  bool process(const image::Image<image::RGBfColor> & input, const image::Image<bool> & mask) {

    if (input.Height() != _pyramid_color[0].Height()) return false;
    if (input.Width() != _pyramid_color[0].Width()) return false;
    if (input.Height() != mask.Height()) return false;
    if (input.Width() != mask.Width()) return false;


    /** 
     * Build pyramid
    */
    _pyramid_color[0] = input;
    _pyramid_mask[0] = mask;
    for (int lvl = 1; lvl < _scales; lvl++) {
      downscale(_pyramid_color[lvl], _pyramid_color[lvl - 1]);
      downscale(_pyramid_mask[lvl], _pyramid_mask[lvl - 1]);
    }

    return true;
  }
 
  static bool downscale(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        output(i, j).r() = 0.25 * (input(i * 2, j * 2).r() + input(i * 2 + 1, j * 2).r() + input(i * 2, j * 2 + 1).r() + input(i * 2 + 1, j * 2 + 1).r());
        output(i, j).g() = 0.25 * (input(i * 2, j * 2).g() + input(i * 2 + 1, j * 2).g() + input(i * 2, j * 2 + 1).g() + input(i * 2 + 1, j * 2 + 1).g());
        output(i, j).b() = 0.25 * (input(i * 2, j * 2).b() + input(i * 2 + 1, j * 2).b() + input(i * 2, j * 2 + 1).b() + input(i * 2 + 1, j * 2 + 1).b());
      }
    }

    return true;
  }

  static bool downscale(image::Image<bool> & output, const image::Image<bool> & input) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        output(i, j) = input(i * 2, j * 2) & input(i * 2 + 1, j * 2) & input(i * 2, j * 2 + 1) & input(i * 2 + 1, j * 2 + 1) ;
      }
    }

    return true;
  }

  const size_t getScalesCount() const {
    return _scales;
  }

  const std::vector<image::Image<image::RGBfColor>> & getPyramidColor() const {
    return _pyramid_color;
  }

  const std::vector<image::Image<bool>> & getPyramidMask() const {
    return _pyramid_mask;
  }

  std::vector<image::Image<image::RGBfColor>> & getPyramidColor() {
    return _pyramid_color;
  }

  std::vector<image::Image<bool>> & getPyramidMask() {
    return _pyramid_mask;
  }

protected:
  std::vector<image::Image<image::RGBfColor>> _pyramid_color;
  std::vector<image::Image<bool>> _pyramid_mask;
  size_t _width_base;
  size_t _height_base;
  size_t _scales;
};

class LaplacianPyramid {
public:
  LaplacianPyramid(const size_t width_base, const size_t height_base, const size_t limit_scales = 64)
  : 
  _pyramid(width_base, height_base, limit_scales),
  _pyramid_differences(width_base, height_base, limit_scales),
  _pyramid_buffer(width_base, height_base, limit_scales)
  {
    for (int i = 0; i < _pyramid.getScalesCount(); i++) {
      _differences_full.push_back(image::Image<image::RGBfColor>(width_base, height_base, true, image::RGBfColor(0.0f, 0.0f, 0.0f)));
      _differences_masks_full.push_back(image::Image<bool>(width_base, height_base, true, false));
    }
  }

  bool process(const image::Image<image::RGBfColor> & input, const image::Image<bool> & mask) {

    /**
     * Compute pyramid of images
     */
    if (!_pyramid.process(input, mask)) {
      return false;
    }

    /**
     * For each level, upscale once and differentiate with upper level
     */
    const std::vector<image::Image<image::RGBfColor>> & pyramid_color = _pyramid.getPyramidColor();
    const std::vector<image::Image<bool>> & pyramid_mask = _pyramid.getPyramidMask();
    std::vector<image::Image<image::RGBfColor>> & difference_color = _pyramid_differences.getPyramidColor();
    std::vector<image::Image<bool>> & difference_mask = _pyramid_differences.getPyramidMask();
    std::vector<image::Image<image::RGBfColor>> & buffers_color = _pyramid_buffer.getPyramidColor();
    std::vector<image::Image<bool>> & buffers_mask = _pyramid_buffer.getPyramidMask();
    
    for (int lvl = 0; lvl < _pyramid.getScalesCount() - 1; lvl++) {

      upscale(buffers_color[lvl], pyramid_color[lvl + 1]);
      upscale(buffers_mask[lvl], pyramid_mask[lvl + 1]);
      
      substract(difference_color[lvl], difference_mask[lvl], pyramid_color[lvl], pyramid_mask[lvl], buffers_color[lvl], buffers_mask[lvl]);
    }

    /**
     * Last level is simply a copy of the original pyramid last level (nothing to substract with)
     */
    difference_color[_pyramid.getScalesCount() - 1] = pyramid_color[_pyramid.getScalesCount() - 1];
    difference_mask[_pyramid.getScalesCount() - 1] = pyramid_mask[_pyramid.getScalesCount() - 1];

    /*Upscale to the max each level*/
    for (int max_level = 0; max_level < _pyramid.getScalesCount(); max_level++) {

      image::Image<image::RGBfColor> & refOut = _differences_full[max_level];
      image::Image<bool> & maskOut = _differences_masks_full[max_level];

      buffers_color[max_level] = difference_color[max_level];
      buffers_mask[max_level] = difference_mask[max_level];
      for (int lvl = max_level - 1; lvl >= 0; lvl--) {  
        upscale(buffers_color[lvl], buffers_color[lvl + 1]);
        upscale(buffers_mask[lvl], buffers_mask[lvl + 1]);
      }

      refOut = buffers_color[0];
      maskOut = buffers_mask[0];
    }

    return true;
  }

  bool blend(const LaplacianPyramid & other) {

    for (int lvl = 0; lvl < _pyramid.getScalesCount(); lvl++) {

      image::Image<image::RGBfColor> & output = _differences_full[lvl];
      image::Image<bool> & mask = _differences_masks_full[lvl];

      const image::Image<image::RGBfColor> & inputA = _differences_full[lvl];
      const image::Image<bool> & maskA = _differences_masks_full[lvl];
      const image::Image<image::RGBfColor> & inputB = other._differences_full[lvl];
      const image::Image<bool> & maskB = other._differences_masks_full[lvl];

      for (int i = 0; i < output.Height(); i++) {
        for (int j = 0; j < output.Width(); j++) {

          float weight = 0.5f;
          float mweight = 1.0f - weight;

          const image::RGBfColor & pixA = inputA(i, j);
          const image::RGBfColor & pixB = inputB(i, j);

          if (maskA(i, j)) {
            if (maskB(i, j)) {
              output(i, j).r() = mweight * pixA.r() +  weight * pixB.r();
              output(i, j).g() = mweight * pixA.g() +  weight * pixB.g();
              output(i, j).b() = mweight * pixA.b() +  weight * pixB.b();
              mask(i, j) = true;
            }
            else {
              output(i, j) = pixA;
              mask(i, j) = true;
            }
          }
          else {
            if (maskB(i, j)) {
              output(i, j) = pixB;
              mask(i, j) = true;
            }
            else {
              output(i, j).r() = 0.0f;
              output(i, j).g() = 0.0f;
              output(i, j).b() = 0.0f;
              mask(i, j) = false;
            }
          }
        }
      }
    }

    return true;
  }

  bool rebuild(image::Image<image::RGBfColor> & output, image::Image<bool> & mask) {

    size_t scales = _pyramid.getScalesCount();
    output = _differences_full[scales - 1];
    mask = _differences_masks_full[scales - 1];

    for (int lvl = scales - 2; lvl >= 0; lvl--) {
      add(output, mask, output, mask, _differences_full[lvl], _differences_masks_full[lvl]);
    }

    return true;
  }

private:
  bool upscale(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input) {

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

  bool upscale(image::Image<bool> & output, const image::Image<bool> & input) {

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

  bool substract(image::Image<image::RGBfColor> & output, image::Image<bool> & output_mask, const image::Image<image::RGBfColor> & inputA, const image::Image<bool> & maskA, const image::Image<image::RGBfColor> & inputB, const image::Image<bool> & maskB) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        if (maskA(i, j)) {
          if (maskB(i, j)) {
            output(i, j).r() = inputA(i, j).r() - inputB(i, j).r();
            output(i, j).g() = inputA(i, j).g() - inputB(i, j).g();
            output(i, j).b() = inputA(i, j).b() - inputB(i, j).b();
          }
          else {
            output(i, j) = inputA(i, j);
          }
          output_mask(i, j) = true;
        }
        else {
          output_mask(i, j) = false;
        }        
      }
    }

    return true;
  }

  bool add(image::Image<image::RGBfColor> & output, image::Image<bool> & output_mask, const image::Image<image::RGBfColor> & inputA, const image::Image<bool> & maskA, const image::Image<image::RGBfColor> & inputB, const image::Image<bool> & maskB) {

    for (int i = 0; i < output.Height(); i++) {
      for (int j = 0; j < output.Width(); j++) {

        
        if (maskA(i, j)) {
          if (maskB(i, j)) {
            output(i, j).r() = inputA(i, j).r() + inputB(i, j).r();
            output(i, j).g() = inputA(i, j).g() + inputB(i, j).g();
            output(i, j).b() = inputA(i, j).b() + inputB(i, j).b();
            output_mask(i, j) = true;
          }
          else {
            output(i, j) = inputA(i, j);
            output_mask(i, j) = true;
          }
        }
        else {
          if (maskB(i, j)) {
            output(i, j) = inputB(i, j);
            output_mask(i, j) = true;
          }
          else {
            output(i, j).r() = 0.0f;
            output(i, j).g() = 0.0f;
            output(i, j).b() = 0.0f;
            output_mask(i, j) = false;
          }
        }
      }
    }

    return true;
  }
 

private:
  Pyramid _pyramid;
  Pyramid _pyramid_differences;
  Pyramid _pyramid_buffer;
  
  std::vector<image::Image<image::RGBfColor>> _differences_full;
  std::vector<image::Image<bool>> _differences_masks_full;
};

class CoordinatesMap {
public:
  /**
   * Build coordinates map given camera properties
   * @param panoramaSize desired output panoramaSize 
   * @param pose the camera pose wrt an arbitrary reference frame
   * @param intrinsics the camera intrinsics
   */
  bool build(const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {

    aliceVision::image::Image<Eigen::Vector2d> buffer_coordinates(panoramaSize.first, panoramaSize.second, false);
    aliceVision::image::Image<bool> buffer_mask(panoramaSize.first, panoramaSize.second, true, false);

    size_t max_x = 0;
    size_t max_y = 0;
    size_t min_x = panoramaSize.first;
    size_t min_y = panoramaSize.second;

    for (size_t y = 0; y < panoramaSize.second; y++) {

      for (size_t x = 0; x < panoramaSize.first; x++) {

        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x,y), panoramaSize.first, panoramaSize.second);

        /**
        * Check that this ray should be visible.
        * This test is camera type dependent
        */
        Vec3 transformedRay = pose(ray);
        if (!intrinsics.isVisibleRay(transformedRay)) {
          continue;
        }

        /**
         * Project this ray to camera pixel coordinates
         */
        const Vec2 pix_disto = intrinsics.project(pose, ray, true);

        /**
         * Ignore invalid coordinates
         */
        if (!intrinsics.isVisible(pix_disto)) {
          continue;
        }

        buffer_coordinates(y, x) = pix_disto;
        buffer_mask(y, x) = true;
  
        min_x = std::min(x, min_x);
        min_y = std::min(y, min_y);
        max_x = std::max(x, max_x);
        max_y = std::max(y, max_y);
      }
    }

    _offset_x = min_x;
    _offset_y = min_y;

    _real_width = max_x - min_x + 1;
    _real_height = max_y - min_y + 1;

    /* Make sure the buffer is a power of 2 for potential pyramids*/
    size_t rectified_width = pow(2.0, ceil(log2(double(_real_width))));
    size_t rectified_height = pow(2.0, ceil(log2(double(_real_height))));

    /* Resize buffers */
    _coordinates = aliceVision::image::Image<Eigen::Vector2d>(rectified_width, rectified_height, false);
    _mask = aliceVision::image::Image<bool>(rectified_width, rectified_height, true, false);

    _coordinates.block(0, 0, _real_height, _real_width) =  buffer_coordinates.block(_offset_y, _offset_x, _real_height, _real_width);
    _mask.block(0, 0, _real_height, _real_width) =  buffer_mask.block(_offset_y, _offset_x, _real_height, _real_width);

    return true;
  }

  size_t getOffsetX() const {
    return _offset_x;
  }

  size_t getOffsetY() const {
    return _offset_y;
  }

  size_t getWidth() const {
    return _real_width;
  }

  size_t getHeight() const {
    return _real_height;
  }

  const aliceVision::image::Image<Eigen::Vector2d> & getCoordinates() const {
    return _coordinates;
  }

  const aliceVision::image::Image<bool> & getMask() const {
    return _mask;
  }

private:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  size_t _real_width = 0;
  size_t _real_height = 0;

  aliceVision::image::Image<Eigen::Vector2d> _coordinates;
  aliceVision::image::Image<bool> _mask;
};

class Warper {
public:
  virtual bool warp(const CoordinatesMap & map, const aliceVision::image::Image<image::RGBfColor> & source) {

    /**
     * Copy additional info from map
     */
    _offset_x = map.getOffsetX();
    _offset_y = map.getOffsetY();
    _real_width = map.getWidth();
    _real_height = map.getHeight();
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2d> & coordinates = map.getCoordinates();

    /**
     * Create buffer
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height());
    

    /**
     * Simple warp
     */
    for (size_t i = 0; i < _color.Height(); i++) {
      for (size_t j = 0; j < _color.Width(); j++) {

        bool valid = _mask(i, j);
        if (!valid) {
          continue;
        }

        const Eigen::Vector2d & coord = coordinates(i, j);
        const image::RGBfColor pixel = sampler(source, coord(1), coord(0));

        _color(i, j) = pixel;
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getColor() const {
    return _color;
  }

  const aliceVision::image::Image<bool> & getMask() const {
    return _mask;
  }
  

  size_t getOffsetX() const {
    return _offset_x;
  }

  size_t getOffsetY() const {
    return _offset_y;
  }

  size_t getWidth() const {
    return _real_width;
  }

  size_t getHeight() const {
    return _real_height;
  }

private:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  size_t _real_width = 0;
  size_t _real_height = 0;

  aliceVision::image::Image<image::RGBfColor> _color;
  aliceVision::image::Image<bool> _mask;
};

class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBfColor(0.0f, 0.0f, 0.0f)), 
  _mask(outputWidth, outputHeight, true, false)
  {
  }

  bool append(const Warper & warper) {

    const aliceVision::image::Image<bool> & inputMask = warper.getMask(); 
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor(); 

    for (size_t i = 0; i < warper.getHeight(); i++) {

      size_t pano_i = warper.getOffsetY() + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < warper.getWidth(); j++) {
        
        if (!inputMask(i, j)) {
          continue;
        }
        
        size_t pano_j = warper.getOffsetX() + j;
        if (pano_j >= _panorama.Width()) {
          continue;
        }

        _panorama(pano_i, pano_j) = color(i, j);
        _mask(pano_i, pano_j) = true;
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getPanorama() const {
    return _panorama;
  }

  const aliceVision::image::Image<bool> & getPanoramaMask() const {
    return _mask;
  }

protected:
  aliceVision::image::Image<image::RGBfColor> _panorama;
  aliceVision::image::Image<bool> _mask;
};


class AverageCompositer : public Compositer {
  
public:

  AverageCompositer(size_t outputWidth, size_t outputHeight) : Compositer(outputWidth, outputHeight) {
    
  }

  bool append(const Warper & warper) {

    const aliceVision::image::Image<bool> & inputMask = warper.getMask(); 
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor(); 

    for (size_t i = 0; i < warper.getHeight(); i++) {

      size_t pano_i = warper.getOffsetY() + i;
      if (pano_i >= _panorama.Height()) {
        continue;
      }

      for (size_t j = 0; j < warper.getWidth(); j++) {
        
        if (!inputMask(i, j)) {
          continue;
        }
        
        size_t pano_j = warper.getOffsetX() + j;
        if (pano_j >= _panorama.Width()) {
          continue;
        }

        if (!_mask(pano_i, pano_j)) {
          _panorama(pano_i, pano_j) = color(i, j);
        }
        else {
          _panorama(pano_i, pano_j).r() = 0.5 * (_panorama(pano_i, pano_j).r() + color(i, j).r());
          _panorama(pano_i, pano_j).g() = 0.5 * (_panorama(pano_i, pano_j).g() + color(i, j).g());
          _panorama(pano_i, pano_j).b() = 0.5 * (_panorama(pano_i, pano_j).b() + color(i, j).b());
          
        }

        _mask(pano_i, pano_j) = true;
      }
    }

    return true;
  }
};

class LaplacianCompositer : public Compositer {
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight) :
  Compositer(outputWidth, outputHeight)
  {
    
  }

  bool append(const Warper & warper) {

    const aliceVision::image::Image<bool> & inputMask = warper.getMask(); 
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor(); 

    aliceVision::image::Image<bool> enlargedMask(_panorama.Width(), _panorama.Height(), true, false);
    aliceVision::image::Image<image::RGBfColor> enlargedInput(_panorama.Width(), _panorama.Height(), false);

    enlargedMask.block(warper.getOffsetY(), warper.getOffsetX(), warper.getHeight(), warper.getWidth()) = inputMask.block(0, 0, warper.getHeight(), warper.getWidth());
    enlargedInput.block(warper.getOffsetY(), warper.getOffsetX(), warper.getHeight(), warper.getWidth()) = color.block(0, 0, warper.getHeight(), warper.getWidth());

    LaplacianPyramid pyramid_camera(_panorama.Width(), _panorama.Height());
    pyramid_camera.process(enlargedInput, enlargedMask);
    
    
    LaplacianPyramid pyramid_panorama(_panorama.Width(), _panorama.Height());
    pyramid_panorama.process(_panorama, _mask);
    pyramid_panorama.blend(pyramid_camera);
    pyramid_panorama.rebuild(_panorama, _mask);

    for (int i = 0; i < _panorama.Height(); i++) {
      for (int j = 0; j < _panorama.Width(); j++) {
        if (!_mask(i, j)) {
          _panorama(i, j).r() = 0.0f;
          _panorama(i, j).g() = 0.0f;
          _panorama(i, j).b() = 0.0f;
        }
      }
    }

    return true;
  }
};


int main(int argc, char **argv) {


  /**
   * Program description
  */
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation.\n"
    "AliceVision PanoramaStitching"
  );

  /**
   * Description of mandatory parameters
   */
  std::string sfmDataFilename;
  std::string outputPanorama;
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&outputPanorama)->required(), "Path of the output folder.");
  allParams.add(requiredParams);

  /**
   * Description of optional parameters
   */
  std::pair<int, int> panoramaSize = {1024, 512};
  po::options_description optionalParams("Optional parameters");
  allParams.add(optionalParams);

  /**
   * Setup log level given command line
   */
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);


  /**
   * Effectively parse command line given parse options
   */
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


  /**
   * Set verbose level given command line
   */
  system::Logger::get()->setLogLevel(verboseLevel);

  /**
   * Load information about inputs
   * Camera images
   * Camera intrinsics
   * Camera extrinsics
   */
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS| sfmDataIO::INTRINSICS| sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  /**
   * Create compositer
  */
  LaplacianCompositer compositer(size_t(panoramaSize.first), size_t(panoramaSize.second));
  

  /**
   * Preprocessing per view
   */
  size_t pos = 0;
  for (auto & viewIt: sfmData.getViews()) {
  
    /**
     * Retrieve view
     */
    const sfmData::View& view = *viewIt.second.get();
    if (!sfmData.isPoseAndIntrinsicDefined(&view)) {
      continue;
    }

    ALICEVISION_LOG_INFO("Processing view " << view.getViewId());

    /**
     * Get intrinsics and extrinsics
     */
    const geometry::Pose3 & camPose = sfmData.getPose(view).getTransform();
    const camera::IntrinsicBase & intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

    /**
     * Prepare coordinates map
    */
   if (pos == 3 || pos == 7 ||  pos  == 15) {
    CoordinatesMap map;
    map.build(panoramaSize, camPose, intrinsic);

    /**
     * Load image and convert it to linear colorspace
     */
    std::string imagePath = view.getImagePath();
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::LINEAR);

    /**
     * Warp image
     */
    Warper warper;
    warper.warp(map, source);


    /**
     *Composite image into output
    */
    compositer.append(warper);
    
    /**
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor();
    char filename[512];
    sprintf(filename, "%s_source_%d.exr", outputPanorama.c_str(), view.getViewId());
    image::writeImage(filename, color, image::EImageColorSpace::NO_CONVERSION);
    */
    const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
    char filename[512];
    sprintf(filename, "%s_source_%d.exr", outputPanorama.c_str(), pos);
    image::writeImage(filename, panorama, image::EImageColorSpace::NO_CONVERSION);
   }
    pos++;

  
  }

  return EXIT_SUCCESS;
}