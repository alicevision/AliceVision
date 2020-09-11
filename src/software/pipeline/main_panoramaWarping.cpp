// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Input and geometry
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Image stuff
#include <aliceVision/image/all.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>

// Logging stuff
#include <aliceVision/system/Logger.hpp>

// Reading command line options
#include <boost/program_options.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// IO
#include <fstream>
#include <algorithm>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace bpt = boost::property_tree;
namespace fs = boost::filesystem;

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

#ifdef _MSC_VER
    // TODO
    // no support for reduction min in MSVC implementation of openmp
#else
    #pragma omp parallel for reduction(min: min_x, min_y) reduction(max: max_x, max_y)
#endif
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

    size_t real_width = max_x - min_x + 1;
    size_t real_height = max_y - min_y + 1;

      /* Resize buffers */
    _coordinates = aliceVision::image::Image<Eigen::Vector2d>(real_width, real_height, false);
    _mask = aliceVision::image::Image<unsigned char>(real_width, real_height, true, 0);

    _coordinates.block(0, 0, real_height, real_width) =  buffer_coordinates.block(min_y, min_x, real_height, real_width);
    _mask.block(0, 0, real_height, real_width) =  buffer_mask.block(min_y, min_x, real_height, real_width);

    return true;
  }

  bool computeScale(double & result, float ratioUpscale) {
    
    std::vector<double> scales;
    size_t real_height = _coordinates.Height();
    size_t real_width = _coordinates.Width();

    for (int i = 1; i < real_height - 2; i++) {
      for (int j = 1; j < real_width - 2; j++) {
        if (!_mask(i, j) || !_mask(i, j + 1) || !_mask(i + 1, j)) {
          continue;
        }

        double dxx = _coordinates(i, j + 1).x() - _coordinates(i, j).x();
        double dxy = _coordinates(i + 1, j).x() - _coordinates(i, j).x();
        double dyx = _coordinates(i, j + 1).y() - _coordinates(i, j).y();
        double dyy = _coordinates(i + 1, j).y() - _coordinates(i, j).y();

        double det = std::abs(dxx*dyy - dxy*dyx);
        if (det < 1e-12) continue;
        
        scales.push_back(det);
      }
    }

    if (scales.empty()) return false;

    std::sort(scales.begin(), scales.end());
    int selected_index = int(floor(float(scales.size() - 1) * ratioUpscale));
    result = sqrt(scales[selected_index]);

    

    return true;
  }

  size_t getOffsetX() const {
    return _offset_x;
  }

  size_t getOffsetY() const {
    return _offset_y;
  }

  const aliceVision::image::Image<Eigen::Vector2d> & getCoordinates() const {
    return _coordinates;
  }

  const aliceVision::image::Image<unsigned char> & getMask() const {
    return _mask;
  }

private:

  bool computeCoarseBB(BBox & coarse_bbox, const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {
    
    bool ret = true;

    if (isPinhole(intrinsics.getType())) {
      ret = computeCoarseBB_Pinhole(coarse_bbox, panoramaSize, pose, intrinsics);
    }
    else if (isEquidistant(intrinsics.getType())) {
      ret = computeCoarseBB_Equidistant(coarse_bbox, panoramaSize, pose, intrinsics);
    }
    else {
      coarse_bbox.left = 0;
      coarse_bbox.top = 0;
      coarse_bbox.width = panoramaSize.first;
      coarse_bbox.height = panoramaSize.second;
      ret = true;
    }

    return ret;
  }

  bool computeCoarseBB_Equidistant(BBox & coarse_bbox, const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {
    
    const aliceVision::camera::EquiDistant & cam = dynamic_cast<const camera::EquiDistant&>(intrinsics);
    

    bool loop = false;
    std::vector<bool> vec_bool(panoramaSize.second, false);

    for (int i = 0; i < panoramaSize.second; i++) {
      
      {
        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(0, i), panoramaSize.first, panoramaSize.second);

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
      }

      {
        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(panoramaSize.first - 1, i), panoramaSize.first, panoramaSize.second);

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

        vec_bool[i] = true;
        loop = true;
      }
    }

    if (vec_bool[0] || vec_bool[panoramaSize.second - 1]) {
      loop = false;
    }

    if (!loop) {
      coarse_bbox.left = 0;
      coarse_bbox.top = 0;
      coarse_bbox.width = panoramaSize.first;
      coarse_bbox.height = panoramaSize.second;
      return true;
    }

    int last_x = 0;

    for (int x = panoramaSize.first - 1; x >= 0; x--) {
      
      size_t count = 0;

      for (int i = 0; i < panoramaSize.second; i++) {
        
        if (vec_bool[i] == false) {
          continue;
        }

        Vec3 ray = SphericalMapping::fromEquirectangular(Vec2(x, i), panoramaSize.first, panoramaSize.second);

        /**
        * Check that this ray should be visible.
        * This test is camera type dependent
        */
        Vec3 transformedRay = pose(ray);
        if (!intrinsics.isVisibleRay(transformedRay)) {
          vec_bool[i] = false;
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
          vec_bool[i] = false;
          continue;
        }
        
        count++;
      }

      if (count == 0) {
        break;
      }

      last_x = x;
    }

    
    coarse_bbox.left = last_x;
    coarse_bbox.top = 0;
    coarse_bbox.width = panoramaSize.first;
    coarse_bbox.height = panoramaSize.second;

    return true;
  }

  bool computeCoarseBB_Pinhole(BBox & coarse_bbox, const std::pair<int, int> & panoramaSize, const geometry::Pose3 & pose, const aliceVision::camera::IntrinsicBase & intrinsics) {

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
    float max_radius_distorted = max_radius;//intrinsics.getMaximalDistortion(0.0, max_radius);

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
      Vec3 pt3d = intrinsics.toUnitSphere(pts_radius[i]);
      rotated_pts[i] = pose.rotation().transpose() * pt3d;
    }

    /* Vertical Default solution : no pole*/
    bbox_top = panoramaSize.second;
    bbox_bottom = 0;

    for (int i = 0; i < 8; i++) {
      int i2 = (i + 1) % 8;
      
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
    bool crossH = false;
    for (int i = 0; i < 8; i++) {
      int i2 = (i + 1) % 8;

      bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
      crossH |= cross;
    }

    if (pole) {
      /*Easy : if we cross the pole, the width is full*/
      bbox_left = 0;
      bbox_right = panoramaSize.first - 1;
      bbox_width = bbox_right - bbox_left + 1;
    }
    else if (crossH) {

      int first_cross = 0;
      for (int i = 0; i < 8; i++) {
        int i2 = (i + 1) % 8;
        bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
        if (cross) {
          first_cross = i;
          break;
        }
      }

      bbox_left = panoramaSize.first - 1;
      bbox_right = 0;
      bool is_right = true;
      for (int index = 0; index < 8; index++) {

        int i = (index + first_cross) % 8;
        int i2 = (i + 1) % 8;

        Vec2 res_1 = SphericalMapping::toEquirectangular(rotated_pts[i], panoramaSize.first, panoramaSize.second);
        Vec2 res_2 = SphericalMapping::toEquirectangular(rotated_pts[i2], panoramaSize.first, panoramaSize.second);

        /*[----right ////  left-----]*/
        bool cross = crossHorizontalLoop(rotated_pts[i], rotated_pts[i2]);
        if (cross) {
          if (res_1(0) > res_2(0)) { /*[----res2 //// res1----]*/
            bbox_left = std::min(int(res_1(0)), bbox_left);
            bbox_right = std::max(int(res_2(0)), bbox_right);
            is_right = true;
          }
          else { /*[----res1 //// res2----]*/
            bbox_left = std::min(int(res_2(0)), bbox_left);
            bbox_right = std::max(int(res_1(0)), bbox_right);
            is_right = false;
          }
        }
        else {
          if (is_right) {
            bbox_right = std::max(int(res_1(0)), bbox_right);
            bbox_right = std::max(int(res_2(0)), bbox_right);
          }
          else {
            bbox_left = std::min(int(res_1(0)), bbox_left);
            bbox_left = std::min(int(res_2(0)), bbox_left);
          }
        }
      }

      bbox_width = bbox_right + (panoramaSize.first - bbox_left);
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

    if (t >= 0.0 && t <= 1.0) {
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
    _mask = map.getMask();

    const image::Sampler2d<image::SamplerLinear> sampler;
    const aliceVision::image::Image<Eigen::Vector2d> & coordinates = map.getCoordinates();

    /**
     * Create buffer
     * No longer need to keep a 2**x size
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

protected:
  size_t _offset_x = 0;
  size_t _offset_y = 0;
  
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
    _color = aliceVision::image::Image<image::RGBfColor>(coordinates.Width(), coordinates.Height(), true, image::RGBfColor(1.0, 0.0, 0.0));
    

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

        double flevel = std::max(0.0, log2(scale));
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

bool computeOptimalPanoramaSize(std::pair<int, int> & optimalSize, const sfmData::SfMData & sfmData, const float ratioUpscale) {

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
    if (!map.computeScale(scale, ratioUpscale)) {
      continue;
    }

    scales.push_back(scale);
  }

  
  if (scales.empty()) {
    return false;
  }

  std::sort(scales.begin(), scales.end());
  int selected_index = int(floor(float(scales.size() - 1) * ratioUpscale));
  double selected_scale = scales[selected_index];

  ALICEVISION_LOG_INFO("Estimated panorama size: "  << int(optimalSize.first * selected_scale) << "x" << int(optimalSize.second * selected_scale));

  double multiplier = pow(2.0, int(floor(log2(selected_scale))));
  optimalSize.first = optimalSize.first * multiplier;
  optimalSize.second = optimalSize.second * multiplier;

  ALICEVISION_LOG_INFO("Estimated panorama size (Rounded to lower power of two): "  << optimalSize.first << "x" << optimalSize.second);

  return true;
}

int aliceVision_main(int argc, char **argv)
{
  std::string sfmDataFilename;
  std::string outputDirectory;

  std::pair<int, int> panoramaSize = {0, 0};
  int percentUpscale = 50;

  image::EStorageDataType storageDataType = image::EStorageDataType::Float;

  int rangeStart = -1;
  int rangeSize = 1;

  // Program description
  po::options_description allParams (
    "Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
    "AliceVision PanoramaWarping"
  );

  // Description of mandatory parameters
  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
    ("output,o", po::value<std::string>(&outputDirectory)->required(), "Path of the output folder.");
  allParams.add(requiredParams);

  // Description of optional parameters
  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("panoramaWidth,w", po::value<int>(&panoramaSize.first)->default_value(panoramaSize.first),
     "Panorama Width in pixels.")
    ("percentUpscale", po::value<int>(&percentUpscale)->default_value(percentUpscale),
     "Percentage of upscaled pixels.")
    ("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
      ("Storage data type: " + image::EStorageDataType_informations()).c_str())
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
     "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
     "Range size.");
  allParams.add(optionalParams);

  // Setup log level given command line
  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel), "verbosity level (fatal, error, warning, info, debug, trace).");
  allParams.add(logParams);

  // Effectively parse command line given parse options
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

  // Set verbose level given command line
  system::Logger::get()->setLogLevel(verboseLevel);

  // Load information about inputs
  // Camera images
  // Camera intrinsics
  // Camera extrinsics
  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS| sfmDataIO::INTRINSICS| sfmDataIO::EXTRINSICS)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  // Order views by their image names for easier debugging
  std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
  for (auto & viewIt: sfmData.getViews()) {
    viewsOrderedByName.push_back(viewIt.second);
  }
  std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(), [](const std::shared_ptr<sfmData::View> & a, const std::shared_ptr<sfmData::View> & b) -> bool { 
    if (a == nullptr || b == nullptr) return true;
    return (a->getImagePath() < b->getImagePath());
  });


  // If panorama width is undefined, estimate it
  if (panoramaSize.first <= 0)
  {
    float ratioUpscale = clamp(float(percentUpscale) / 100.0f, 0.0f, 1.0f);


    std::pair<int, int> optimalPanoramaSize;
    if (computeOptimalPanoramaSize(optimalPanoramaSize, sfmData, ratioUpscale))
    {
      panoramaSize = optimalPanoramaSize;
    }
    else {
      ALICEVISION_LOG_INFO("Impossible to compute an optimal panorama size");
      return EXIT_FAILURE;
    }
  }
  else
  {
    double max_scale = 1.0 / pow(2.0, 10);
    panoramaSize.first = int(ceil(double(panoramaSize.first) * max_scale) / max_scale);
    panoramaSize.second = panoramaSize.first / 2;
  }

  ALICEVISION_LOG_INFO("Choosen panorama size : "  << panoramaSize.first << "x" << panoramaSize.second);

  // Define range to compute
  if(rangeStart != -1)
  {
    if(rangeStart < 0 || rangeSize < 0 ||
       std::size_t(rangeStart) > viewsOrderedByName.size())
    {
      ALICEVISION_LOG_ERROR("Range is incorrect");
      return EXIT_FAILURE;
    }

    if(std::size_t(rangeStart + rangeSize) > viewsOrderedByName.size())
    {
      rangeSize = int(viewsOrderedByName.size()) - rangeStart;
    }
  }
  else
  {
      rangeStart = 0;
      rangeSize = int(viewsOrderedByName.size());
  }
  ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);

  // Preprocessing per view
  for(std::size_t i = std::size_t(rangeStart); i < std::size_t(rangeStart + rangeSize); ++i)
  {
    const std::shared_ptr<sfmData::View> & viewIt = viewsOrderedByName[i];

    // Retrieve view
    const sfmData::View& view = *viewIt;
    if (!sfmData.isPoseAndIntrinsicDefined(&view))
    {
      continue;
    }

    ALICEVISION_LOG_INFO("[" << int(i) + 1 - rangeStart << "/" << rangeSize << "] Processing view " << view.getViewId() << " (" << i + 1 << "/" << viewsOrderedByName.size() << ")");

    // Get intrinsics and extrinsics
    geometry::Pose3 camPose = sfmData.getPose(view).getTransform();
    std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view.getIntrinsicId());
    std::shared_ptr<camera::EquiDistant> casted = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);    

    // Prepare coordinates map
    CoordinatesMap map;
    map.build(panoramaSize, camPose, *(intrinsic.get()));

    // Load image and convert it to linear colorspace
    std::string imagePath = view.getImagePath();
    ALICEVISION_LOG_INFO("Load image with path " << imagePath);
    image::Image<image::RGBfColor> source;
    image::readImage(imagePath, source, image::EImageColorSpace::LINEAR);

    // Warp image
    GaussianWarper warper;
    warper.warp(map, source);

    // Alpha mask
    AlphaBuilder alphabuilder;
    alphabuilder.build(map, *(intrinsic.get()));

    // Export mask and image
    {
        const std::string viewIdStr = std::to_string(view.getViewId());

        oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
        const int offsetX = int(warper.getOffsetX());
        const int offsetY = int(warper.getOffsetY());
        metadata.push_back(oiio::ParamValue("AliceVision:offsetX", offsetX));
        metadata.push_back(oiio::ParamValue("AliceVision:offsetY", offsetY));
        metadata.push_back(oiio::ParamValue("AliceVision:panoramaWidth", panoramaSize.first));
        metadata.push_back(oiio::ParamValue("AliceVision:panoramaHeight", panoramaSize.second));

        // Images will be converted in Panorama coordinate system, so there will be no more extra orientation.
        metadata.remove("Orientation");
        metadata.remove("orientation");

        {
            const aliceVision::image::Image<image::RGBfColor> & cam = warper.getColor();

            oiio::ParamValueList viewMetadata = metadata;
            viewMetadata.push_back(oiio::ParamValue("AliceVision:storageDataType", image::EStorageDataType_enumToString(storageDataType)));

            const std::string viewFilepath = (fs::path(outputDirectory) / (viewIdStr + ".exr")).string();
            ALICEVISION_LOG_INFO("Store view " << i << " with path " << viewFilepath);
            image::writeImage(viewFilepath, cam, image::EImageColorSpace::AUTO, viewMetadata);
        }
        {
            const aliceVision::image::Image<unsigned char> & mask = warper.getMask();

            const std::string maskFilepath = (fs::path(outputDirectory) / (viewIdStr + "_mask.exr")).string();
            ALICEVISION_LOG_INFO("Store mask " << i << " with path " << maskFilepath);
            image::writeImage(maskFilepath, mask, image::EImageColorSpace::NO_CONVERSION, metadata);
        }
        {
            const aliceVision::image::Image<float> & weights = alphabuilder.getWeights();

            const std::string weightFilepath = (fs::path(outputDirectory) / (viewIdStr + "_weight.exr")).string();
            ALICEVISION_LOG_INFO("Store weightmap " << i << " with path " << weightFilepath);
            image::writeImage(weightFilepath, weights, image::EImageColorSpace::AUTO, metadata);
        }
    }
  }

  return EXIT_SUCCESS;
}
