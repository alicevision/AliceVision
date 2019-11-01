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

/*IO*/
#include <fstream>

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

Eigen::VectorXf gaussian_kernel_vector(size_t kernel_length, float sigma) {
  
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

  double sum = k1d.sum();
  k1d = k1d / sum;

  return k1d.cast<float>();
}

Eigen::MatrixXf gaussian_kernel(size_t kernel_length, float sigma) {
  
  Eigen::VectorXf k1d = gaussian_kernel_vector(kernel_length, sigma);
  Eigen::MatrixXf K = k1d * k1d.transpose();
  

  double sum = K.sum();
  K = K / sum;

  return K;
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
    oiio::ImageBuf K;
    oiio::ImageBufAlgo::make_kernel(K, "gaussian", 5, 5);

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
private:
  struct BBox {
    int left;
    int top;
    int width;
    int height;
  };

public:
  /**
   * Build coordinates map given camera properties
   * @param panoramaSize desired output panoramaSize 
   * @param pose the camera pose wrt an arbitrary reference frame
   * @param intrinsics the camera intrinsics
   */
  bool build(const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {

    BBox coarse_bbox;
    if (!computeCoarseBB(coarse_bbox, panoramaSize, pose, intrinsics)) {
      return false;
    }
    

    /* Effectively compute the warping map */
    aliceVision::image::Image<Eigen::Vector2d> buffer_coordinates(coarse_bbox.width, coarse_bbox.height, false);
    aliceVision::image::Image<unsigned char> buffer_mask(coarse_bbox.width, coarse_bbox.height, true, 0);

    size_t max_x = 0;
    size_t max_y = 0;
    size_t min_x = panoramaSize.first;
    size_t min_y = panoramaSize.second;
    
    #pragma omp parallel for reduction(min: min_x, min_y) reduction(max: max_x, max_y)
    for (size_t y = 0; y < coarse_bbox.height; y++) {

      size_t cy = y + coarse_bbox.top;

      size_t row_max_x = 0;
      size_t row_max_y = 0;
      size_t row_min_x = panoramaSize.first;
      size_t row_min_y = panoramaSize.second;


      for (size_t x = 0; x < coarse_bbox.width; x++) {

        size_t cx = x + coarse_bbox.left;

        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(cx, cy), panoramaSize.first, panoramaSize.second);

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
  
        row_min_x = std::min(x, row_min_x);
        row_min_y = std::min(y, row_min_y);
        row_max_x = std::max(x, row_max_x);
        row_max_y = std::max(y, row_max_y);
      }

      min_x = std::min(row_min_x, min_x);
      min_y = std::min(row_min_y, min_y);
      max_x = std::max(row_max_x, max_x);
      max_y = std::max(row_max_y, max_y);
    }
   
    _offset_x = coarse_bbox.left + min_x;
    if (_offset_x > panoramaSize.first) {
      /*The coarse bounding box may cross the borders where as the true coordinates may not*/
      int ox = int(_offset_x) - int(panoramaSize.first);
      _offset_x = ox;
    }
    _offset_y = coarse_bbox.top + min_y;
    _real_width = max_x - min_x + 1;
    _real_height = max_y - min_y + 1;

    /* Make sure the buffer is a power of 2 for potential pyramids*/
    size_t rectified_width = pow(2.0, ceil(log2(double(_real_width))));
    size_t rectified_height = pow(2.0, ceil(log2(double(_real_height))));

      /* Resize buffers */
    _coordinates = aliceVision::image::Image<Eigen::Vector2d>(rectified_width, rectified_height, false);
    _mask = aliceVision::image::Image<unsigned char>(rectified_width, rectified_height, true, 0);

    _coordinates.block(0, 0, _real_height, _real_width) =  buffer_coordinates.block(min_y, min_x, _real_height, _real_width);
    _mask.block(0, 0, _real_height, _real_width) =  buffer_mask.block(min_y, min_x, _real_height, _real_width);

    return true;
  }

  bool computeScale(double & result) {
    
    std::vector<double> scales;

    for (int i = 0; i < _real_height - 1; i++) {
      for (int j = 0; j < _real_width - 1; j++) {
        if (!_mask(i, j) || !_mask(i, j + 1) || !_mask(i + 1, j)) {
          continue;
        }

        double dxx = _coordinates(i, j + 1).x() - _coordinates(i, j).x();
        double dxy = _coordinates(i + 1, j).x() - _coordinates(i, j).x();
        double dyx = _coordinates(i, j + 1).y() - _coordinates(i, j).y();
        double dyy = _coordinates(i + 1, j).y() - _coordinates(i, j).y();

        double det = std::abs(dxx*dyy - dxy*dyx);
        scales.push_back(det);
      }
    }

    if (scales.size() <= 1) return false;

    std::nth_element(scales.begin(), scales.begin() + scales.size() / 2, scales.end());
    result = sqrt(scales[scales.size() / 2]);
    

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

  bool computeCoarseBB(BBox & coarse_bbox, const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {

    coarse_bbox.left = 0;
    coarse_bbox.top = 0;
    coarse_bbox.width = panoramaSize.first;
    coarse_bbox.height = panoramaSize.second;
    
    int bbox_left, bbox_top;
    int bbox_right, bbox_bottom;
    int bbox_width, bbox_height;

    /*Estimate distorted maximal distance from optical center*/
    Vec2 pts[] = {{0.0f, 0.0f}, {intrinsics.w(), 0.0f}, {intrinsics.w(), intrinsics.h()}, {0.0f, intrinsics.h()}};
    float max_radius = 0.0;
    for (int i = 0; i < 4; i++) {

      Vec2 ptmeter = intrinsics.ima2cam(pts[i]);
      float radius = ptmeter.norm();
      max_radius = std::max(max_radius, radius);
    }

    /* Estimate undistorted maximal distance from optical center */
    float max_radius_distorted = intrinsics.getMaximalDistortion(0.0, max_radius);

    /* 
    Coarse rectangle bouding box in camera space 
    We add intermediate points to ensure arclength between 2 points is never more than 180°
    */
    Vec2 pts_radius[] = {
        {-max_radius_distorted, -max_radius_distorted}, 
        {0, -max_radius_distorted},
        {max_radius_distorted, -max_radius_distorted}, 
        {max_radius_distorted, 0},
        {max_radius_distorted, max_radius_distorted},
        {0, max_radius_distorted},
        {-max_radius_distorted, max_radius_distorted},
        {-max_radius_distorted, 0}
      };


    /* 
    Transform bounding box into the panorama frame.
    Point are on a unit sphere.
    */
    Vec3 rotated_pts[8];
    for (int i = 0; i < 8; i++) {
      Vec3 pt3d = pts_radius[i].homogeneous().normalized();
      rotated_pts[i] = pose.rotation().transpose() * pt3d;
    }

    /* Vertical Default solution : no pole*/
    bbox_top = panoramaSize.second;
    bbox_bottom = 0;

    for (int i = 0; i < 8; i++) {
      int i2 = i + 1;
      if (i2 > 7) i2 = 0;
      
      Vec3 extremaY = getExtremaY(rotated_pts[i], rotated_pts[i2]);

      Vec2 res;
      res = SphericalMapping::toEquirectangular(extremaY, panoramaSize.first, panoramaSize.second);
      bbox_top = std::min(int(floor(res(1))), bbox_top);
      bbox_bottom = std::max(int(ceil(res(1))), bbox_bottom);

      res = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
      bbox_top = std::min(int(floor(res(1))), bbox_top);
      bbox_bottom = std::max(int(ceil(res(1))), bbox_bottom);
    }

    /* 
    Check if our region circumscribe a pole of the sphere :
    Check that the region projected on the Y=0 plane contains the point (0, 0)
    This is a special projection case
    */
    bool pole = isPoleInTriangle(rotated_pts[0], rotated_pts[1], rotated_pts[7]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[2], rotated_pts[3]);
    pole |= isPoleInTriangle(rotated_pts[3], rotated_pts[4], rotated_pts[5]);
    pole |= isPoleInTriangle(rotated_pts[7], rotated_pts[5], rotated_pts[6]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[3], rotated_pts[5]);
    pole |= isPoleInTriangle(rotated_pts[1], rotated_pts[5], rotated_pts[7]);
    
    
    if (pole) {
      Vec3 normal = (rotated_pts[1] - rotated_pts[0]).cross(rotated_pts[3] - rotated_pts[0]);
      if (normal(1) > 0) {
        //Lower pole
        bbox_bottom = panoramaSize.second - 1;
      }
      else {
        //upper pole
        bbox_top = 0;
      }
    }

    bbox_height = bbox_bottom - bbox_top + 1;


    /*Check if we cross the horizontal loop*/
    bool crossH;
    for (int i = 0; i < 8; i++) {
      int i2 = i + 1;
      if (i2 > 7) i2 = 0;
      
      bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
      if (i == 0) crossH = cross;
      else crossH |= cross;
    }


    if (pole) {
      /*Easy : if we cross the pole, the width is full*/
      bbox_left = 0;
      bbox_right = panoramaSize.first - 1;
      bbox_width = bbox_right - bbox_left + 1;
    }
    else if (crossH) {

      bbox_left = panoramaSize.first;
      bbox_right = 0;

      for (int i = 0; i < 8; i++) {
        int i2 = i + 1;
        if (i2 > 7) i2 = 0;

        Vec2 res_left = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
        Vec2 res_right = SphericalMapping::toEquirectangular(rotated_pts[i2], panoramaSize.first, panoramaSize.second);

        if (crossHorizontalLoop(rotated_pts[i], rotated_pts[i2])) {

          if (res_left(0) > res_right(0)) {
            //Left --> border; border --> right
            bbox_left = std::min(bbox_left, int(floor(res_left(0))));
            bbox_right = std::max(bbox_right, int(ceil(res_right(0))));

            if (i % 2 == 0) {
              //next segment is continuity
              int next = i2 + 1;
              if (next > 7) next = 0;

              Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[next], panoramaSize.first, panoramaSize.second);
              bbox_right = std::max(bbox_right, int(ceil(res(0))));
            }
            else {
              int prev = i - 1;
              if (prev < 0) prev = 7;
              Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[prev], panoramaSize.first, panoramaSize.second);
              bbox_left = std::min(bbox_left, int(ceil(res(0))));
            }
          }
          else {
            //right --> border; border --> left
            bbox_left = std::min(bbox_left, int(floor(res_right(0))));
            bbox_right = std::max(bbox_right, int(ceil(res_left(0))));

            if (i % 2 == 0) {
              //next segment is continuity
              int next = i2 + 1;
              if (next > 7) next = 0;

              Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[next], panoramaSize.first, panoramaSize.second);
              bbox_left = std::min(bbox_left, int(ceil(res(0))));
            }
            else {
              int prev = i - 1;
              if (prev < 0) prev = 7;
              Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[prev], panoramaSize.first, panoramaSize.second);
              bbox_right = std::max(bbox_right, int(ceil(res(0))));
            }
          }
        }
      }
    
      bbox_width = bbox_right + 1 + (panoramaSize.first - bbox_left + 1);
    }
    else {
      /*horizontal default solution : no border crossing, no pole*/
      bbox_left = panoramaSize.first;
      bbox_right = 0;
      for (int i = 0; i < 8; i++) {
        Vec2 res = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
        bbox_left = std::min(int(floor(res(0))), bbox_left);
        bbox_right = std::max(int(ceil(res(0))), bbox_right);
      }
      bbox_width = bbox_right - bbox_left + 1;
    }

    /*Assign solution to result*/
    coarse_bbox.left = bbox_left;
    coarse_bbox.top = bbox_top;
    coarse_bbox.width = bbox_width;
    coarse_bbox.height = bbox_height;

    return true;
  }

  Vec3 getExtremaY(const Vec3 & pt1, const Vec3 & pt2) {
    Vec3 delta = pt2 - pt1;
    double dx = delta(0);
    double dy = delta(1);
    double dz = delta(2);
    double sx = pt1(0);
    double sy = pt1(1);
    double sz = pt1(2);

    double ot_y = -(dx*sx*sy - (dy*sx)*(dy*sx) - (dy*sz)*(dy*sz) + dz*sy*sz)/(dx*dx*sy - dx*dy*sx - dy*dz*sz + dz*dz*sy);

    Vec3 pt_extrema = pt1 + ot_y * delta;

    return pt_extrema.normalized();
  }

  bool crossHorizontalLoop(const Vec3 & pt1, const Vec3 & pt2) {
    Vec3 direction = pt2 - pt1;

    /*Vertical line*/
    if (std::abs(direction(0)) < 1e-12) {
      return false;
    }

    double t = - pt1(0) / direction(0); 
    Vec3 cross = pt1 + direction * t;

    if (t >= 0.0 && t < 1.0) {
      if (cross(2) < 0.0) {
        return true;
      } 
    }

    return false;
  }

  bool isPoleInTriangle(const Vec3 & pt1, const Vec3 & pt2, const Vec3 & pt3) {
   
    double a = (pt2.x()*pt3.z() - pt3.x()*pt2.z())/(pt1.x()*pt2.z() - pt1.x()*pt3.z() - pt2.x()*pt1.z() + pt2.x()*pt3.z() + pt3.x()*pt1.z() - pt3.x()*pt2.z());
    double b = (-pt1.x()*pt3.z() + pt3.x()*pt1.z())/(pt1.x()*pt2.z() - pt1.x()*pt3.z() - pt2.x()*pt1.z() + pt2.x()*pt3.z() + pt3.x()*pt1.z() - pt3.x()*pt2.z());
    double c = 1.0 - a - b;

    if (a < 0.0 || a > 1.0) return false;
    if (b < 0.0 || b > 1.0) return false;
    if (c < 0.0 || c > 1.0) return false;
 
    return true;
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

  void dump(std::ofstream & output) {
    output.write((char*)&_offset_x, sizeof(_offset_x));
    output.write((char*)&_offset_y, sizeof(_offset_y));
    output.write((char*)&_real_width, sizeof(_real_width));
    output.write((char*)&_real_height, sizeof(_real_height));
    
    char * ptr_color = (char*)(_color.data());
    for (int i = 0; i < _real_height; i++) {
      output.write(ptr_color, _real_width * sizeof(decltype(_color)::Scalar));
      ptr_color += _color.rowStride();
    }

    char * ptr_mask = (char*)(_mask.data());
    for (int i = 0; i < _real_height; i++) {
      output.write(ptr_mask, _real_width * sizeof(decltype(_mask)::Scalar));
      ptr_mask += _mask.rowStride();
    }
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
          pano_j = pano_j - _panorama.Width();
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

    const aliceVision::image::Image<float> & camera_weights = alpha.getWeights();
    const aliceVision::image::Image<unsigned char> & camera_mask = warper.getMask();
    const aliceVision::image::Image<image::RGBfColor> & camera_color = warper.getColor();

    size_t ox = warper.getOffsetX();
    size_t oy = warper.getOffsetY();
    size_t pwidth = _panorama.Width();
    size_t pheight = _panorama.Height();
 
    /**
     * Create a copy of panorama related pixels
     */
    aliceVision::image::Image<image::RGBfColor> panorama_subcolor(camera_weights.Width(), camera_weights.Height(), true, image::RGBfColor(0.0f, 0.0f, 0.0f));
    aliceVision::image::Image<unsigned char> panorama_submask(camera_weights.Width(), camera_weights.Height(), true, false);

    if (ox + warper.getWidth() > pwidth) {

      int left_width = ox + warper.getWidth() - pwidth;
      int right_width = warper.getWidth() - left_width;
      panorama_subcolor.block(0, 0, warper.getHeight(), right_width) = _panorama.block(oy, ox, warper.getHeight(), right_width);
      panorama_subcolor.block(0, right_width, warper.getHeight(), left_width) = _panorama.block(oy, 0, warper.getHeight(), left_width);
      panorama_submask.block(0, 0, warper.getHeight(), right_width) = _mask.block(oy, ox, warper.getHeight(), right_width);
      panorama_submask.block(0, right_width, warper.getHeight(), left_width) = _mask.block(oy, 0, warper.getHeight(), left_width);
    }
    else {
      panorama_subcolor.block(0, 0, warper.getHeight(), warper.getWidth()) = _panorama.block(oy, ox, warper.getHeight(), warper.getWidth());
      panorama_submask.block(0, 0, warper.getHeight(), warper.getWidth()) = _mask.block(oy, ox, warper.getHeight(), warper.getWidth());
    }

    /**
     * Compute optimal scale
     * The smallest level will be at least of size min_size
     */
    size_t min_dim = std::min(camera_weights.Width(), camera_weights.Height());
    size_t min_size = 4;
    size_t limit_scales = 10;
    size_t scales = std::min(limit_scales, static_cast<size_t>(floor(log2(double(min_dim) / float(min_size)))));

    /**
     * Create buffers
     */
    std::vector<aliceVision::image::Image<float>> camera_weightmaps(scales);
    std::vector<aliceVision::image::Image<image::RGBfColor>> camera_colors(scales);
    std::vector<aliceVision::image::Image<image::RGBfColor>> camera_differences(scales);
    std::vector<aliceVision::image::Image<image::RGBfColor>> panorama_colors(scales);
    std::vector<aliceVision::image::Image<image::RGBfColor>> panorama_differences(scales);
    std::vector<aliceVision::image::Image<image::RGBfColor>> blended_differences(scales);
    std::vector<aliceVision::image::Image<unsigned char>> blended_mask(scales);

    for (int level = 0; level < scales; level++) {
      camera_weightmaps[level] = aliceVision::image::Image<float>(camera_weights.Width(), camera_weights.Height());
      camera_colors[level] = aliceVision::image::Image<image::RGBfColor>(camera_weights.Width(), camera_weights.Height());
      camera_differences[level] = aliceVision::image::Image<image::RGBfColor>(camera_weights.Width(), camera_weights.Height());
      panorama_colors[level] = aliceVision::image::Image<image::RGBfColor>(camera_weights.Width(), camera_weights.Height());
      panorama_differences[level] = aliceVision::image::Image<image::RGBfColor>(camera_weights.Width(), camera_weights.Height());
      blended_differences[level] = aliceVision::image::Image<image::RGBfColor>(camera_weights.Width(), camera_weights.Height());
      blended_mask[level] = aliceVision::image::Image<unsigned char>(camera_weights.Width(), camera_weights.Height());
    }

    /* Build max weight map for first level */
    auto & maxweightmap = camera_weightmaps[0];

    for (int i = 0; i < camera_weights.Height(); i++) {
      for (int j = 0; j < camera_weights.Width(); j++) {

        maxweightmap(i, j) = 0.0f;

        if (!camera_mask(i, j)) {
          continue;
        }

        float cw = camera_weights(i, j);

        int pano_y = oy + i;
        int pano_x = ox + j;
        if (pano_x >= pwidth) {
          pano_x = pano_x - pwidth;
        }

        float pw = _weightmap(pano_y, pano_x);

        if (cw > pw) {
          maxweightmap(i, j) = 1.0f;
          _weightmap(pano_y, pano_x) = cw;
        }
      }
    }
    
    Eigen::MatrixXf kernel = gaussian_kernel(5, 2.0f);

    /*Create scales*/
    camera_colors[0] = camera_color;
    panorama_colors[0] = panorama_subcolor;
    for (int level = 1; level < scales; level++) {
      convolve(camera_weightmaps[level], camera_weightmaps[level - 1], camera_mask, kernel);
      convolve(camera_colors[level], camera_colors[level - 1], camera_mask, kernel);
      convolve(panorama_colors[level], panorama_colors[level - 1], panorama_submask, kernel);
    }

  
    /*Compute differences*/
    for (int level = 0; level < scales - 1; level++) {
      camera_differences[level] = camera_colors[level] - camera_colors[level + 1];
      panorama_differences[level] = panorama_colors[level] - panorama_colors[level + 1];
    }
    camera_differences[scales - 1] = camera_colors[scales - 1];
    panorama_differences[scales - 1] = panorama_colors[scales - 1];

    /*Blend*/
    for (int level = 0; level < scales; level++) {
      const aliceVision::image::Image<image::RGBfColor> & imgpano = panorama_differences[level];
      const aliceVision::image::Image<image::RGBfColor> & imgcam = camera_differences[level];
      const aliceVision::image::Image<float> & weights = camera_weightmaps[level];
      aliceVision::image::Image<image::RGBfColor> & imgblend = blended_differences[level];
      aliceVision::image::Image<unsigned char> & maskblend = blended_mask[level];

      for (int i = 0; i < camera_weights.Height(); i++) {
        for (int j = 0; j < camera_weights.Width(); j++) {
          if (camera_mask(i, j)) {
            if (panorama_submask(i, j)) {
              float w = weights(i, j);
              float mw = 1.0f - w;
              imgblend(i, j).r() = mw * imgpano(i, j).r() + w * imgcam(i, j).r();
              imgblend(i, j).g() = mw * imgpano(i, j).g() + w * imgcam(i, j).g();
              imgblend(i, j).b() = mw * imgpano(i, j).b() + w * imgcam(i, j).b();
              maskblend(i, j) = 1;
            }
            else {
              imgblend(i, j) = imgcam(i, j);
              maskblend(i, j) = 1;
            }
          }
          else {
            if (panorama_submask(i, j)) {
              imgblend(i, j) = imgpano(i, j);
              maskblend(i, j) = 1;
            }
            else {
              maskblend(i, j) = 0;
            }
          }
        }
      }
    }

    /*Rebuild*/
    for (int level = scales - 1; level >= 0; level--) {
      
      aliceVision::image::Image<image::RGBfColor> & imgblend = blended_differences[level];
      aliceVision::image::Image<unsigned char> & maskblend = blended_mask[level];

      for (int i = 0; i < warper.getHeight(); i++) {
        for (int j = 0; j < warper.getWidth(); j++) {
          
          int pano_y = oy + i;
          int pano_x = ox + j;
          if (pano_x >= pwidth) {
            pano_x = pano_x - pwidth;
          }

          if (maskblend(i, j)) {
            if (level == scales - 1) {
              _panorama(pano_y, pano_x) = image::RGBfColor(0.0f);
              _mask(pano_y, pano_x) = 0;
            }

            _panorama(pano_y, pano_x) += imgblend(i, j);
            _mask(pano_y, pano_x) = 1;
          }
        }
      }
    }
    

    return true;
  }

private:
  aliceVision::image::Image<float> _maxweightmap;
};

bool computeOptimalPanoramaSize(std::pair<int, int> & optimalSize, const sfmData::SfMData & sfmData) {

  optimalSize.first = 512;
  optimalSize.second = 256;

   /**
   * Loop over views to estimate best scale
   */
  std::vector<double> scales;
  for (auto & viewIt: sfmData.getViews()) {
  
    /**
     * Retrieve view
     */
    const sfmData::View& view = *viewIt.second.get();
    if (!sfmData.isPoseAndIntrinsicDefined(&view)) {
      continue;
    }

    /**
     * Get intrinsics and extrinsics
     */
    const geometry::Pose3  camPose = sfmData.getPose(view).getTransform();
    const camera::IntrinsicBase & intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

    /**
     * Compute map
     */
    CoordinatesMap map;
    if (!map.build(optimalSize, camPose, intrinsic)) {
      continue;
    }

    double scale;
    if (!map.computeScale(scale)) {
      continue;
    }

    scales.push_back(scale);
  }

  
  if (scales.size() > 1) {
    double median_scale;
    std::nth_element(scales.begin(), scales.begin() + scales.size() / 2, scales.end());
    median_scale = scales[scales.size() / 2];

    double multiplier = pow(2.0, int(floor(log2(median_scale))));
    
    optimalSize.first = optimalSize.first * multiplier;
    optimalSize.second = optimalSize.second * multiplier;
  }

  return true;
}

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
  std::pair<int, int> panoramaSize = {1024, 0};
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("panoramaWidth,w", po::value<int>(&panoramaSize.first)->default_value(panoramaSize.first), "Panorama Width in pixels.");
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


  /*Order views by their image names for easier debugging*/
  std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
  for (auto & viewIt: sfmData.getViews()) {
    viewsOrderedByName.push_back(viewIt.second);
  }
  std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
    if (a == nullptr || b == nullptr) return true;
    return (a->getImagePath() < b->getImagePath());
  });


  /*If panorama width is undefined, estimate it*/
  if (panoramaSize.first <= 0) {
    std::pair<int, int> optimalPanoramaSize;
    if (computeOptimalPanoramaSize(optimalPanoramaSize, sfmData)) {
      panoramaSize = optimalPanoramaSize;
    }
  }
  else {
    panoramaSize.second = panoramaSize.first / 2;
  }

  ALICEVISION_LOG_INFO("Choosen panorama size : "  << panoramaSize.first << "x" << panoramaSize.second);

  /**
   * Create compositer
  */
  AlphaCompositer compositer(size_t(panoramaSize.first), size_t(panoramaSize.second));

  /**
   * Preprocessing per view
   */
  size_t pos = 0;
  for (const std::shared_ptr<sfmData::View> & viewIt: viewsOrderedByName) {

    /**
     * Retrieve view
     */
    const sfmData::View& view = *viewIt;
    if (!sfmData.isPoseAndIntrinsicDefined(&view)) {
      continue;
    }

    ALICEVISION_LOG_INFO("Processing view " << view.getViewId());

    /**
     * Get intrinsics and extrinsics
     */
    const geometry::Pose3 camPose = sfmData.getPose(view).getTransform();
    const camera::IntrinsicBase & intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

    /**
     * Prepare coordinates map
    */
    CoordinatesMap map;
    map.build(panoramaSize, camPose, intrinsic);

    /**
     * Load image and convert it to linear colorspace
     */
    std::string imagePath = view.getImagePath();
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::LINEAR);

      

    /**
     * Warp image
     */
    GaussianWarper warper;
    warper.warp(map, source);

    /*std::ofstream out("/home/mmoc/test.dat", std::ios::binary);
    warper.dump(out);
    out.close();*/
    
    /**
     * Alpha mask
     */
    AlphaBuilder alphabuilder;
    alphabuilder.build(map, intrinsic);
    
    /**
     *Composite image into output
    */
    ALICEVISION_LOG_INFO("Composite to final panorama\n");
    compositer.append(warper, alphabuilder);
    
    pos++;
  }


  ALICEVISION_LOG_INFO("Save final panoram\n");
  const aliceVision::image::Image<image::RGBfColor> & panorama = compositer.getPanorama();
  image::writeImage(outputPanorama, panorama, image::EImageColorSpace::SRGB);

  return EXIT_SUCCESS;
}