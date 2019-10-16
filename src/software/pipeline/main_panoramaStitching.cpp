/**
 * Input and geometry
*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/**
 * Image stuff
 */
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

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

float sigmoid(float x, float sigwidth, float sigMid)
{
  return 1.0f / (1.0f + expf(10.0f * ((x - sigMid) / sigwidth)));
}

bool computeDistanceMap(image::Image<int> & distance, const image::Image<bool> & mask) {

  int m = mask.Height();
  int n = mask.Width();

  int maxval = m * n;

  distance = image::Image<int> (n, m, false); 
  for(int x = 0; x < n; ++x) {

    //A corner is when mask becomes 0
    bool b = !mask(0, x);
    if (b) {
      distance(0, x) = 0;
    }
    else {
      distance(0, x) = maxval * maxval;
    }

    for (int y = 1; y < m; y++) {
      bool b = !mask(y, x);
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

class GaussianPyramidNoMask {
public:
  GaussianPyramidNoMask(const size_t width_base, const size_t height_base, const size_t limit_scales = 64) :
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

      _pyramid_color.push_back(image::Image<image::RGBfColor>(new_width, new_height, true, image::RGBfColor(0)));
      _filter_buffer.push_back(image::Image<image::RGBfColor>(new_width, new_height, true, image::RGBfColor(0)));
      new_height /= 2;
      new_width /= 2;
    }
  }

  bool process(const image::Image<image::RGBfColor> & input) {

    if (input.Height() != _pyramid_color[0].Height()) return false;
    if (input.Width() != _pyramid_color[0].Width()) return false;


    /**
     * Kernel
     */
    oiio::ImageBuf K = oiio::ImageBufAlgo::make_kernel("gaussian", 5, 5);

    /** 
     * Build pyramid
    */
    _pyramid_color[0] = input;
    for (int lvl = 0; lvl < _scales - 1; lvl++) {
      
      const image::Image<image::RGBfColor> & source = _pyramid_color[lvl];
      image::Image<image::RGBfColor> & dst = _filter_buffer[lvl];

      oiio::ImageSpec spec(source.Width(), source.Height(), 3, oiio::TypeDesc::FLOAT);

      const oiio::ImageBuf inBuf(spec, const_cast<image::RGBfColor*>(source.data()));
      oiio::ImageBuf outBuf(spec, dst.data());    
      oiio::ImageBufAlgo::convolve(outBuf, inBuf, K);

      downscale(_pyramid_color[lvl + 1], _filter_buffer[lvl]);      
    }

    return true;
  }


  bool downscale(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input) {

    for (int i = 0; i < output.Height(); i++) {
      int ui = i * 2;

      for (int j = 0; j < output.Width(); j++) {
        int uj = j * 2;

        output(i, j) = input(ui, uj);
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

template <typename T>
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

      _pyramid_color.push_back(image::Image<T>(new_width, new_height, true, T(0)));
      _pyramid_mask.push_back(image::Image<bool>(new_width, new_height, true, false));
      new_height /= 2;
      new_width /= 2;
    }
  }

  bool process(const image::Image<T> & input, const image::Image<bool> & mask) {

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
      downscale(_pyramid_color[lvl], _pyramid_mask[lvl], _pyramid_color[lvl - 1], _pyramid_mask[lvl - 1]);      
    }

    return true;
  }


  static bool downscale(image::Image<T> & output, image::Image<bool> & output_mask, const image::Image<T> & input, const image::Image<bool> & input_mask) {

    for (int i = 0; i < output.Height(); i++) {
      int ui = i * 2;

      for (int j = 0; j < output.Width(); j++) {
        int uj = j * 2;
        
        bool one = input_mask(ui, uj) || input_mask(ui, uj + 1) || input_mask(ui + 1, uj) || input_mask(ui + 1, uj + 1);

        if (!one) {
          output(i, j) = T(0.0f);
          output_mask(i, j) = false;
        }
        else {
          size_t count = 0;
          T color = T(0.0f);

          if (input_mask(ui, uj)) {
            color = input(ui, uj);
            count++;
          }
          
          if (input_mask(ui, uj + 1)) {
            color += input(ui, uj + 1);
            count++;
          }

          if (input_mask(ui + 1, uj)) {
            color += input(ui + 1, uj);
            count++;
          }

          if (input_mask(ui + 1, uj + 1)) {
            color += input(ui + 1, uj + 1);
            count++;
          }

          color /= float(count);

          output(i, j) = color;
          output_mask(i, j) = true;
        }
      }
    }

    return true;
  }

  const size_t getScalesCount() const {
    return _scales;
  }

  const std::vector<image::Image<T>> & getPyramidColor() const {
    return _pyramid_color;
  }

  const std::vector<image::Image<bool>> & getPyramidMask() const {
    return _pyramid_mask;
  }

  std::vector<image::Image<T>> & getPyramidColor() {
    return _pyramid_color;
  }

  std::vector<image::Image<bool>> & getPyramidMask() {
    return _pyramid_mask;
  }

protected:
  std::vector<image::Image<T>> _pyramid_color;
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

    return true;
  }

  bool blend(image::Image<image::RGBfColor> & output, image::Image<bool> & output_mask, const LaplacianPyramid & other, const image::Image<float> & weights) {

    const auto & differences = _pyramid_differences.getPyramidColor();
    const auto & differences_masks = _pyramid_differences.getPyramidMask();
    const auto & other_differences = other._pyramid_differences.getPyramidColor();
    const auto & other_differences_masks = other._pyramid_differences.getPyramidMask();
    auto & rescaleds = _pyramid_buffer.getPyramidColor();
    auto & rescaleds_masks = _pyramid_buffer.getPyramidMask();
    const auto & original_camera_mask = other._pyramid_differences.getPyramidMask()[0];

    output.fill(image::RGBfColor(0.0f));
    output_mask.fill(false);

    for (int lvl = _pyramid.getScalesCount() - 1; lvl >= 0; lvl--) {

      /*Rescale this level to the 0 level size for the added camera*/
      rescaleds[lvl] = other_differences[lvl];
      rescaleds_masks[lvl] = other_differences_masks[lvl];
      for (int slvl = lvl - 1; slvl >= 0; slvl--) {
        upscale(rescaleds[slvl], rescaleds[slvl + 1]);
        upscale(rescaleds_masks[slvl], rescaleds_masks[slvl + 1]);
      }

      auto rescaled_other = rescaleds[0];
      auto rescaled_masks_other = rescaleds_masks[0];

      for (int i = 0; i < rescaled_other.Height(); i++) {
        for (int j = 0; j < rescaled_other.Width(); j++) {
          if (!original_camera_mask(i, j)) {
            rescaled_other(i, j) = image::RGBfColor(0.0f);
            rescaled_masks_other(i, j) = false;
          }
        }
      }

      /*Rescale this level to the 0 level size*/
      rescaleds[lvl] = differences[lvl];
      rescaleds_masks[lvl] = differences_masks[lvl];
      for (int slvl = lvl - 1; slvl >= 0; slvl--) {
        upscale(rescaleds[slvl], rescaleds[slvl + 1]);
        upscale(rescaleds_masks[slvl], rescaleds_masks[slvl + 1]);
      }

      auto rescaled_origin = rescaleds[0];
      auto rescaled_masks_origin = rescaleds_masks[0];

      for (int i = 0; i < rescaled_origin.Height(); i++) {
        for (int j = 0; j < rescaled_origin.Width(); j++) {
          if (rescaled_masks_origin(i, j)) {
            if (rescaled_masks_other(i, j)) {

              float weight = weights(i, j);
              float mweight = 1.0f - weight;

              rescaled_origin(i, j).r() = mweight * rescaled_origin(i, j).r() + weight * rescaled_other(i, j).r();
              rescaled_origin(i, j).g() = mweight * rescaled_origin(i, j).g() + weight * rescaled_other(i, j).g();
              rescaled_origin(i, j).b() = mweight * rescaled_origin(i, j).b() + weight * rescaled_other(i, j).b();
              rescaled_masks_origin(i, j) = true;
            }
            else {
              //Nothing to do
              rescaled_masks_origin(i, j) = true;
            }
          }
          else {
            if (rescaled_masks_other(i, j)) {
              rescaled_origin(i, j) = rescaled_other(i, j);
              rescaled_masks_origin(i, j) = rescaled_masks_other(i, j);
              rescaled_masks_origin(i, j) = true;
            }
            else {
              rescaled_origin(i, j) = image::RGBfColor(0.0f);
              rescaled_masks_origin(i, j) = false;
            }
          }
        }
      }

      for (int i = 0; i < rescaled_origin.Height(); i++) {
        for (int j = 0; j < rescaled_origin.Width(); j++) {
          if (rescaled_masks_origin(i, j)) {
            output(i, j).r() = output(i, j).r() + rescaled_origin(i, j).r();
            output(i, j).g() = output(i, j).g() + rescaled_origin(i, j).g();
            output(i, j).b() = output(i, j).b() + rescaled_origin(i, j).b();
            output_mask(i, j) = true;
          }
        }
      }
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

            output(i, j).r() = 0.0f;
            output(i, j).g() = 0.0f;
            output(i, j).b() = 0.0f;
            output_mask(i, j) = false;
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
  Pyramid<image::RGBfColor> _pyramid;
  Pyramid<image::RGBfColor> _pyramid_differences;
  Pyramid<image::RGBfColor> _pyramid_buffer;
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

protected:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  size_t _real_width = 0;
  size_t _real_height = 0;

  aliceVision::image::Image<image::RGBfColor> _color;
  aliceVision::image::Image<bool> _mask;
};

class GaussianWarper : public Warper {
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
     * Create a pyramid for input
     */
    GaussianPyramidNoMask pyramid(source.Width(), source.Height());
    pyramid.process(source);
    const std::vector<image::Image<image::RGBfColor>> & mlsource = pyramid.getPyramidColor();
    size_t max_level = pyramid.getScalesCount() - 1; 

    /**
     * Create buffer
     */
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height());
    

    /**
     * Multi level warp
     */
    for (size_t i = 0; i < _color.Height(); i++) {
      for (size_t j = 0; j < _color.Width(); j++) {

        bool valid = _mask(i, j);
        if (!valid) {
          continue;
        }

        if (i == _color.Height() - 1 || j == _color.Width() - 1 || !_mask(i + 1, j) || !_mask(i, j + 1)) {
          const Eigen::Vector2d & coord = coordinates(i, j);
          const image::RGBfColor pixel = sampler(source, coord(1), coord(0));
          _color(i, j) = pixel;
          continue;
        }

        const Eigen::Vector2d & coord_mm = coordinates(i, j);
        const Eigen::Vector2d & coord_mp = coordinates(i, j + 1);
        const Eigen::Vector2d & coord_pm = coordinates(i + 1, j);
        
        double dxx = coord_pm(0) - coord_mm(0);
        double dxy = coord_mp(0) - coord_mm(0);
        double dyx = coord_pm(1) - coord_mm(1);
        double dyy = coord_mp(1) - coord_mm(1);
        double det = std::abs(dxx*dyy - dxy*dyx);
        double scale = sqrt(det);
        

        double flevel = log2(scale);
        size_t blevel = std::min(max_level, size_t(floor(flevel)));        

        double dscale, x, y;
        dscale = 1.0 / pow(2.0, blevel);
        x = coord_mm(0) * dscale;
        y = coord_mm(1) * dscale;
        /*Fallback to first level if outside*/
        if (x >= mlsource[blevel].Width() - 1 || y >= mlsource[blevel].Height() - 1) {
          _color(i, j) = sampler(mlsource[0], coord_mm(1), coord_mm(0));
          continue;
        }

        _color(i, j) = sampler(mlsource[blevel], y, x);
      }
    }

    return true;
  }
};

class Compositer {
public:
  Compositer(size_t outputWidth, size_t outputHeight) :
  _panorama(outputWidth, outputHeight, true, image::RGBfColor(1.0f, 0.0f, 0.0f)), 
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
    
    image::Image<int> distanceMap;
    computeDistanceMap(distanceMap, enlargedMask);

    image::Image<float> weightMap(_panorama.Width(), _panorama.Height());
    for(int y = 0; y < _panorama.Height(); ++y)
    {
      for(int x = 0; x < _panorama.Width(); ++x)
       {
        int dist = distanceMap(y, x);

        weightMap(y, x) = 0.0f;
        if (dist > 0)
        {
          float fdist = sqrtf(float(dist));
          weightMap(y, x) = std::min(1.0f, std::max(0.0f, 1.0f - sigmoid(fdist, 100, 50)));
        }
      }
    }

    LaplacianPyramid pyramid_panorama(_panorama.Width(), _panorama.Height());
    pyramid_panorama.process(_panorama, _mask);
    pyramid_panorama.blend(_panorama, _mask, pyramid_camera, weightMap);

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
  std::pair<int, int> panoramaSize = {2048, 1024};
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

    std::cout << camPose.rotation() << std::endl;

    /*if (pos == 8) */{
    /**
     * Prepare coordinates map
    */
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
    GaussianWarper warper;
    warper.warp(map, source);


    /**
     *Composite image into output
    */
    compositer.append(warper);
    {
    const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
    char filename[512];
    sprintf(filename, "%s_source_%d.exr", outputPanorama.c_str(), pos);
    image::writeImage(filename, panorama, image::EImageColorSpace::NO_CONVERSION);
    }
    }
    pos++;
  }

  return EXIT_SUCCESS;
}