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

Eigen::MatrixXf gaussian_kernel(size_t kernel_length, float sigma) {
  
  Eigen::VectorXd x;
  x.setLinSpaced(kernel_length + 1, -sigma, +sigma);

  Eigen::VectorXd cdf(kernel_length + 1);
  for (int i = 0; i < kernel_length + 1; i++) {
    cdf(i) = 0.5 * (1 + std::erf(x(i)/sqrt(2.0)));
  }

  Eigen::VectorXd k1d(kernel_length);
  for (int i = 0; i < kernel_length; i++) {
    k1d(i) = cdf(i + 1) - cdf(i);
  }

  Eigen::MatrixXd K = k1d * k1d.transpose();
  

  double sum = K.sum();
  K = K / sum;

  return K.cast<float>();
}

bool convolve(image::Image<float> & output, const image::Image<float> & input, const image::Image<unsigned char> & mask, const Eigen::MatrixXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (output.size() != mask.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  if (kernel.rows() != kernel.cols()) {
    return false;
  }

  int radius = kernel.rows() / 2;

  Eigen::MatrixXf kernel_scaled = kernel;

  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      float sum = 0.0f;
      float sum_mask = 0.0f;

      if (!mask(i, j)) {
        output(i, j) = 0.0f; 
        continue;
      }

      for (int k = 0; k < kernel.rows(); k++) {
        float ni = i + k - radius;
        if (ni < 0 || ni >= output.Height()) {
          continue;
        }

        for (int l = 0; l < kernel.cols(); l++) {
          float nj = j + l - radius;
          if (nj < 0 || nj >= output.Width()) {
            continue;
          }

          if (!mask(ni, nj)) {
            continue;
          }

          float val = kernel(k, l) * input(ni, nj);
          sum += val;
          sum_mask += kernel(k, l);
        }
      } 

      output(i, j) = sum / sum_mask;
    }
  }

  return true;
}

bool convolve(image::Image<image::RGBfColor> & output, const image::Image<image::RGBfColor> & input, const image::Image<unsigned char> & mask, const Eigen::MatrixXf & kernel) {

  if (output.size() != input.size()) {
    return false;
  }

  if (output.size() != mask.size()) {
    return false;
  }

  if (kernel.size() % 2 == 0) {
    return false;
  }

  if (kernel.rows() != kernel.cols()) {
    return false;
  }

  int radius = kernel.rows() / 2;

  Eigen::MatrixXf kernel_scaled = kernel;

  for (int i = 0; i < output.Height(); i++) {
    for (int j = 0; j < output.Width(); j++) {

      image::RGBfColor sum(0.0f);
      float sum_mask = 0.0f;

      if (!mask(i, j)) {
        output(i, j) = sum; 
        continue;
      }

      for (int k = 0; k < kernel.rows(); k++) {
        float ni = i + k - radius;
        if (ni < 0 || ni >= output.Height()) {
          continue;
        }

        for (int l = 0; l < kernel.cols(); l++) {
          float nj = j + l - radius;
          if (nj < 0 || nj >= output.Width()) {
            continue;
          }

          if (!mask(ni, nj)) {
            continue;
          }

          sum.r() += kernel(k, l) * input(ni, nj).r();
          sum.g() += kernel(k, l) * input(ni, nj).g();
          sum.b() += kernel(k, l) * input(ni, nj).b();
          sum_mask += kernel(k, l);
        }
      } 

      output(i, j) = sum / sum_mask;
    }
  }

  return true;
}

bool computeDistanceMap(image::Image<int> & distance, const image::Image<unsigned char> & mask) {

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
    aliceVision::image::Image<unsigned char> buffer_mask(panoramaSize.first, panoramaSize.second, true, false);

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
        buffer_mask(y, x) = 1;
  
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
    _mask = aliceVision::image::Image<unsigned char>(rectified_width, rectified_height, true, 0);

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

  const aliceVision::image::Image<unsigned char> & getMask() const {
    return _mask;
  }

private:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  size_t _real_width = 0;
  size_t _real_height = 0;

  aliceVision::image::Image<Eigen::Vector2d> _coordinates;
  aliceVision::image::Image<unsigned char> _mask;
};

class AlphaBuilder {
public:
  virtual bool build(const CoordinatesMap & map, const aliceVision::camera::IntrinsicBase & intrinsics) {
    
    float w = static_cast<float>(intrinsics.w());
    float h = static_cast<float>(intrinsics.h());
    float cx = w / 2.0f;
    float cy = h / 2.0f;
    

    const aliceVision::image::Image<Eigen::Vector2d> & coordinates = map.getCoordinates();
    const aliceVision::image::Image<unsigned char> & mask = map.getMask();

    _weights = aliceVision::image::Image<float>(coordinates.Width(), coordinates.Height());

    for (int i = 0; i < _weights.Height(); i++) {
      for (int j = 0; j < _weights.Width(); j++) {
        
        _weights(i, j) = 0.0f;

        bool valid = mask(i, j);
        if (!valid) {
          continue;
        }

        const Vec2 & coords = coordinates(i, j);

        float x = coords(0);
        float y = coords(1);

        float wx = 1.0f - std::abs((x - cx) / cx);
        float wy = 1.0f - std::abs((y - cy) / cy);
        
        _weights(i, j) = wx * wy;
      }
    }

    return true;
  }

  const aliceVision::image::Image<float> & getWeights() const {
    return _weights;
  }

private:
  aliceVision::image::Image<float> _weights;
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

  const aliceVision::image::Image<unsigned char> & getMask() const {
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
  aliceVision::image::Image<unsigned char> _mask;
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

  virtual bool append(const Warper & warper) {

    const aliceVision::image::Image<unsigned char> & inputMask = warper.getMask(); 
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
        _mask(pano_i, pano_j) = 1;
      }
    }

    return true;
  }

  const aliceVision::image::Image<image::RGBfColor> & getPanorama() const {
    return _panorama;
  }

  const aliceVision::image::Image<unsigned char> & getPanoramaMask() const {
    return _mask;
  }

protected:
  aliceVision::image::Image<image::RGBfColor> _panorama;
  aliceVision::image::Image<unsigned char> _mask;
};


class AverageCompositer : public Compositer {
public:

  AverageCompositer(size_t outputWidth, size_t outputHeight) : Compositer(outputWidth, outputHeight) {
    
  }

  virtual bool append(const Warper & warper) {

    const aliceVision::image::Image<unsigned char> & inputMask = warper.getMask(); 
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

class AlphaCompositer : public Compositer {
public:

  AlphaCompositer(size_t outputWidth, size_t outputHeight) : 
  Compositer(outputWidth, outputHeight),
  _weightmap(outputWidth, outputHeight, true, 0.0f) {
    
  }

  virtual bool append(const Warper & warper, const AlphaBuilder & alpha) {

    const aliceVision::image::Image<unsigned char> & inputMask = warper.getMask(); 
    const aliceVision::image::Image<image::RGBfColor> & color = warper.getColor(); 
    const aliceVision::image::Image<float> & inputWeights = alpha.getWeights(); 

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
          _weightmap(pano_i, pano_j) = inputWeights(i, j);
        }
        else {
          float wp = _weightmap(pano_i, pano_j);
          float wc = inputWeights(i, j);
          float wtotal = wp + wc;
          wp /= wtotal;
          wc /= wtotal;

          _panorama(pano_i, pano_j).r() = wp * _panorama(pano_i, pano_j).r() + wc * color(i, j).r();
          _panorama(pano_i, pano_j).g() = wp * _panorama(pano_i, pano_j).g() + wc * color(i, j).g();
          _panorama(pano_i, pano_j).b() = wp * _panorama(pano_i, pano_j).b() + wc * color(i, j).b();
          
          _weightmap(pano_i, pano_j) = wtotal;
        }

        _mask(pano_i, pano_j) = true;
      }
    }

    return true;
  }

protected:
  aliceVision::image::Image<float> _weightmap;
};

class LaplacianCompositer : public AlphaCompositer {
public:

  LaplacianCompositer(size_t outputWidth, size_t outputHeight) : 
  AlphaCompositer(outputWidth, outputHeight),
  _maxweightmap(outputWidth, outputHeight, true, 0.0f) {
    
  }

  virtual bool append(const Warper & warper, const AlphaBuilder & alpha) {

    const auto & weights = alpha.getWeights();

    size_t ox = warper.getOffsetX();
    size_t oy = warper.getOffsetY();

    for (int i = 0; i < weights.Height(); i++) {
      for (int j = 0; j < weights.Width(); j++) {
        float cw = weights(i, j);
        float pw = _weightmap(oy + i, ox + j);

        //if (cw > )
      }
    }

    //Eigen::MatrixXf kernel = gaussian_kernel(5, 1.0f);

    return true;
  }

private:
  aliceVision::image::Image<float> _maxweightmap;
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
  AlphaCompositer compositer(size_t(panoramaSize.first), size_t(panoramaSize.second));


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


    /*if (pos == 3 || pos == 7 || pos == 15) */{
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


    
    AlphaBuilder alphabuilder;
    alphabuilder.build(map, intrinsic);
    

    /**
     *Composite image into output
    */
    compositer.append(warper, alphabuilder);

    {
    const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
    char filename[512];
    sprintf(filename, "%s_intermediate_%d.exr", outputPanorama.c_str(), pos);
    image::writeImage(filename, panorama, image::EImageColorSpace::NO_CONVERSION);
    }
    }
    pos++;
  }

  return EXIT_SUCCESS;
}