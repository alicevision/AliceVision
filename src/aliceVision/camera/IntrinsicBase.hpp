// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicInitMode.hpp>
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/version.hpp>

#include <vector>

namespace aliceVision {
namespace camera {

/**
 * @brief Basis class for all intrinsic parameters of a camera
 *  Store the image size & define all basis optical modelization of a camera
 */
class IntrinsicBase
{
public:

  explicit IntrinsicBase(unsigned int width = 0, unsigned int height = 0, const std::string& serialNumber = "")
    : _w(width)
    , _h(height)
    , _serialNumber(serialNumber)
  {}

  virtual ~IntrinsicBase() = default;
  
  /**
   * @brief Get the lock state of the intrinsic
   * @return true if the intrinsic is locked
   */
  inline bool isLocked() const
  {
    return _locked;
  }
  
  /**
   * @brief Get the intrinsic image width
   * @return The intrinsic image width
   */
  inline unsigned int w() const
  {
    return _w;
  }

  /**
   * @brief Get the intrinsic image height
   * @return The intrinsic image height
   */
  inline unsigned int h() const
  {
    return _h;
  }

  /**
   * @brief Get the intrinsic sensor width
   * @return The intrinsic sensor width
   */
  inline double sensorWidth() const
  {
    return _sensorWidth;
  }

  /**
   * @brief Get the intrinsic sensor height
   * @return The intrinsic sensor height
   */
  inline double sensorHeight() const
  {
    return _sensorHeight;
  }

  /**
   * @brief Get the intrinsic serial number
   * @return The intrinsic serial number
   */
  inline const std::string& serialNumber() const
  {
    return _serialNumber;
  }

  /**
   * @brief Get the intrinsic initialization mode
   * @return The intrinsic initialization mode
   */
  inline EIntrinsicInitMode getInitializationMode() const
  {
    return _initializationMode;
  }

  /**
   * @brief operator ==
   * @param[in] other
   * @return True if equals
   */
  inline bool operator==(const IntrinsicBase& other) const
  {
      if(getParams().size() != other.getParams().size())
      {
          return false;
      }
      for(int i = 0; i < getParams().size(); i++)
      {
          const double diff = getParams()[i] - other.getParams()[i];
          if(std::abs(diff) > 1e-8)
          {
              return false;
          }
      }
    return _w == other._w &&
           _h == other._h &&
           _sensorWidth == other._sensorWidth &&
           _sensorHeight == other._sensorHeight &&
           _serialNumber == other._serialNumber &&
           _initializationMode == other._initializationMode &&
           getType() == other.getType();
  }

  inline bool operator!=(const IntrinsicBase& other) const
  {
    return !(*this == other);
  }

  /**
   * @brief Projection of a 3D point into the camera plane (Apply pose, disto (if any) and Intrinsics)
   * @param[in] pose The pose
   * @param[in] pt3D The 3d point
   * @param[in] applyDistortion If true apply distrortion if any
   * @return The 2d projection in the camera plane
   */
  virtual Vec2 project(const geometry::Pose3& pose, const Vec4& pt3D, bool applyDistortion = true) const = 0;

  /**
   * @brief Back-projection of a 2D point at a specific depth into a 3D point
   * @param[in] pt2D The 2d point
   * @param[in] applyDistortion If true apply distrortion if any
   * @param[in] pose The camera pose
   * @param[in] depth The depth
   * @return The 3d point
   */
  Vec3 backproject(const Vec2& pt2D, bool applyUndistortion = true, const geometry::Pose3& pose = geometry::Pose3(), double depth = 1.0) const
  {
      const Vec2 pt2D_cam = ima2cam(pt2D);
      const Vec2 pt2D_undist = applyUndistortion ? removeDistortion(pt2D_cam) : pt2D_cam;

      const Vec3 pt3d = depth * toUnitSphere(pt2D_undist);
      const Vec3 output = pose.inverse()(pt3d);
      return output;
  }

  Vec4 getCartesianfromSphericalCoordinates(const Vec3 & pt)
  {
    double u = pt(0);
    double v = pt(1);
    
    Vec4 rpt;
    rpt.x() = pt(0);
    rpt.y() = pt(1);
    rpt.z() = 1.0;
    rpt.w() = pt(2);

    return rpt;
  }

  Eigen::Matrix<double, 4, 3> getDerivativeCartesianfromSphericalCoordinates(const Vec3 & pt)
  {
  
    Eigen::Matrix<double, 4, 3> ret = Eigen::Matrix<double, 4, 3>::Zero();

    ret(0, 0) = 1.0;
    ret(1, 1) = 1.0;
    ret(3, 2) = 1.0;

    return ret;
  }

  /**
   * @brief get derivative of a projection of a 3D point into the camera plane
   * @param[in] pose The pose
   * @param[in] pt3D The 3d point
   * @param[in] applyDistortion If true apply distrortion if any
   * @return The projection jacobian  wrt pose
   */
  virtual Eigen::Matrix<double, 2, 16> getDerivativeProjectWrtPose(const geometry::Pose3& pose, const Vec4& pt3D) const = 0;

  /**
   * @brief get derivative of a projection of a 3D point into the camera plane
   * @param[in] pose The pose
   * @param[in] pt3D The 3d point
   * @param[in] applyDistortion If true apply distrortion if any
   * @return The projection jacobian  wrt point
   */
  virtual Eigen::Matrix<double, 2, 4> getDerivativeProjectWrtPoint(const geometry::Pose3& pose, const Vec4& pt3D) const = 0;

  /**
   * @brief get derivative of a projection of a 3D point into the camera plane
   * @param[in] pose The pose
   * @param[in] pt3D The 3d point
   * @param[in] applyDistortion If true apply distrortion if any
   * @return The projection jacobian wrt params
   */
  virtual Eigen::Matrix<double, 2, Eigen::Dynamic> getDerivativeProjectWrtParams(const geometry::Pose3& pose, const Vec4& pt3D) const = 0;

   /**
   * @brief Compute the residual between the 3D projected point X and an image observation x
   * @param[in] pose The pose
   * @param[in] X The 3D projected point
   * @param[in] x The image observation
   * @return residual
   */
  inline Vec2 residual(const geometry::Pose3& pose, const Vec4& X, const Vec2& x) const
  {
    const Vec2 proj = this->project(pose, X);
    return x - proj;
  }

  /**
   * @brief Compute the residual between the 3D projected point X and an image observation x
   * @param[in] pose The pose
   * @param[in] X The 3D projection
   * @param[in] x The image observation
   * @return residual
   */
  inline Mat2X residuals(const geometry::Pose3& pose, const Mat3X& X, const Mat2X& x) const
  {
    assert(X.cols() == x.cols());
    const std::size_t numPts = x.cols();
    Mat2X residuals = Mat2X::Zero(2, numPts);
    for(std::size_t i = 0; i < numPts; ++i)
    {
      residuals.col(i) = residual(pose, ((const Vec3&)X.col(i)).homogeneous(), x.col(i));
    }
    return residuals;
  }

  /**
   * @brief lock the intrinsic
   */
  inline void lock()
  {
    _locked  = true;
  }

  /**
   * @brief unlock the intrinsic
   */
  inline void unlock()
  {
    _locked  = false;
  }

  /**
   * @brief Set intrinsic image width
   * @param[in] width The image width
   */
  inline void setWidth(unsigned int width)
  {
    _w = width;
  }

  /**
   * @brief Set intrinsic image height
   * @param[in] height The image height
   */
  inline void setHeight(unsigned int height)
  {
    _h = height;
  }

  /**
   * @brief Set intrinsic sensor width
   * @param[in] width The sensor width
   */
  inline void setSensorWidth(double width)
  {
    _sensorWidth = width;
  }

  /**
   * @brief Set intrinsic sensor height
   * @param[in] height The sensor height
   */
  inline void setSensorHeight(double height)
  {
    _sensorHeight = height;
  }
  
  /**
   * @brief Set the serial number
   * @param[in] serialNumber The serial number
   */
  inline void setSerialNumber(const std::string& serialNumber)
  {
    _serialNumber = serialNumber;
  }

  /**
   * @brief Set The intrinsic initialization mode
   * @param[in] initializationMode THe intrintrinsic initialization mode enum
   */
  inline void setInitializationMode(EIntrinsicInitMode initializationMode)
  {
    _initializationMode = initializationMode;
  }

  // Virtual members

  /**
   * @brief Polymorphic clone
   */
  virtual IntrinsicBase* clone() const = 0;

  /**
   * @brief Assign object
   * @param[in] other
   */
  virtual void assign(const IntrinsicBase& other) = 0;

  /**
   * @brief Get embed camera type
   * @return EINTRINSIC enum
   */
  virtual EINTRINSIC getType() const = 0;

  /**
   * get a string
   * @return the string describing the intrinsic type
   */
  std::string getTypeStr() const { 
    return EINTRINSIC_enumToString(getType()); 
  }

  /**
   * @brief Get intrinsic parameters
   * @return intrinsic parameters
   */
  virtual std::vector<double> getParams() const = 0;

  /**
   * @brief Update intrinsic parameters
   * @param[in] intrinsic parameters
   * @return true if done
   */
  virtual bool updateFromParams(const std::vector<double>& params) = 0;

  /**
   * @brief import intrinsic parameters from external array
   * @param[in] intrinsic parameters
   * @param[in] inputVersion input source version (for optional transformation)
   * @return true if done
   */
  virtual bool importFromParams(const std::vector<double>& params, const Version & inputVersion) = 0;

  /**
   * @brief Transform a point from the camera plane to the image plane
   * @param[in] p A point from the camera plane
   * @return Image plane point
   */
  virtual Vec2 cam2ima(const Vec2& p) const = 0;

  /**
   * @brief Transform a point from the image plane to the camera plane
   * @param[in] p A point from the image plane
   * @return Camera plane point
   */
  virtual Vec2 ima2cam(const Vec2& p) const = 0;

  /**
   * @brief Camera model handle a distortion field
   * @return True if the camera model handle a distortion field
   */
  virtual bool hasDistortion() const
  {
    return false;
  }

  /**
   * @brief Add the distortion field to a point (that is in normalized camera frame)
   * @param[in] p The point
   * @return The point with added distortion field
   */
  virtual Vec2 addDistortion(const Vec2& p) const = 0;

  /**
   * @brief Remove the distortion to a camera point (that is in normalized camera frame)
   * @param[in] p The point
   * @return The point with removed distortion field
   */
  virtual Vec2 removeDistortion(const Vec2& p) const = 0;

  /**
   * @brief Return the undistorted pixel (with removed distortion)
   * @param[in] p The point
   * @return The undistorted pixel
   */
  virtual Vec2 get_ud_pixel(const Vec2& p) const = 0;

  /**
   * @brief Return the distorted pixel (with added distortion)
   * @param[in] p The undistorted point
   * @return The distorted pixel
   */
  virtual Vec2 get_d_pixel(const Vec2& p) const = 0;

  /**
   * @brief Normalize a given unit pixel error to the camera plane
   * @param[in] value Given unit pixel error
   * @return Normalized unit pixel error to the camera plane
   */
  virtual double imagePlaneToCameraPlaneError(double value) const = 0;


  /**
   * @brief Return true if the intrinsic is valid
   * @return True if the intrinsic is valid
   */
  virtual bool isValid() const
  {
    return _w != 0 && _h != 0;
  }

  /**
   * @brief Return true if this ray should be visible in the image
   * @param ray input ray to check for visibility
   * @return true if this ray is visible theorically
   */
  virtual bool isVisibleRay(const Vec3 & ray) const = 0;

  /**
   * @brief Return true if these pixel coordinates should be visible in the image
   * @param pix input pixel coordinates to check for visibility
   * @return true if visible
   */
  virtual bool isVisible(const Vec2 & pix) const {

    if (pix(0) < 0 || pix(0) >= _w || pix(1) < 0 || pix(1) >= _h) {
      return false;
    }

    return true;
  }

  /**
   * @brief Assuming the distortion is a function of radius, estimate the 
   * maximal undistorted radius for a range of distorted radius.
   * @param min_radius the minimal radius to consider
   * @param max_radius the maximal radius to consider
   * @return the maximal undistorted radius
   */
  virtual float getMaximalDistortion(double min_radius, double max_radius) const {

    /*Without distortion, obvious*/
    return max_radius;
  }

  /**
   * @brief Generate an unique Hash from the camera parameters (used for grouping)
   * @return Unique Hash from the camera parameters
   */
  virtual std::size_t hashValue() const
  {
    size_t seed = 0;
    stl::hash_combine(seed, static_cast<int>(this->getType()));
    stl::hash_combine(seed, _w);
    stl::hash_combine(seed, _h);
    stl::hash_combine(seed, _sensorWidth);
    stl::hash_combine(seed, _sensorHeight);
    stl::hash_combine(seed, _serialNumber);
    const std::vector<double> params = this->getParams();
    for(double param : params)
    {
        stl::hash_combine(seed, param);
    }
    return seed;
  }

  /**
   * @brief Rescale intrinsics to reflect a rescale of the camera image
   * @param factor a scale factor
   */
  virtual void rescale(float factor) {

    _w = (unsigned int)(floor(float(_w) * factor));
    _h = (unsigned int)(floor(float(_h) * factor));
  }

  /**
   * @brief transform a given point (in pixels) to unit sphere in meters
   * @param pt the input point
   * @return a point on the unit sphere
   */
  virtual Vec3 toUnitSphere(const Vec2 & pt) const = 0;

protected:

  /// initialization mode
  EIntrinsicInitMode _initializationMode = EIntrinsicInitMode::NONE;
  /// intrinsic lock
  bool _locked = false;
  unsigned int _w = 0;
  unsigned int _h = 0;
  double _sensorWidth = 36.0;
  double _sensorHeight = 24.0;
  std::string _serialNumber;
};

/**
 * @brief Apply intrinsic and extrinsic parameters to unit vector
 * from the cameras focus to a point on the camera plane
 * @param[in] pose Extrinsic pose
 * @param[in] intrinsic Intrinsic camera paremeters
 * @param[in] x Point in image
 * @return The unit vector in 3D space pointing out from the camera to the point
 */
inline Vec3 applyIntrinsicExtrinsic(const geometry::Pose3& pose,
                                    const IntrinsicBase* intrinsic,
                                    const Vec2& x) {

  // x = (u, v, 1.0)  // image coordinates
  // X = R.t() * K.inv() * x + C // Camera world point
  // getting the ray:
  // ray = X - C = R.t() * K.inv() * x
  return (pose.rotation().transpose() * intrinsic->toUnitSphere(intrinsic->removeDistortion(intrinsic->ima2cam(x)))).normalized();
}

/**
 * @brief Return the angle (degree) between two bearing vector rays
 * @param[in] ray1 First bearing vector ray
 * @param[in] ray2 Second bearing vector ray
 * @return The angle (degree) between two bearing vector rays
 */
inline double angleBetweenRays(const Vec3& ray1, const Vec3& ray2)
{
  const double mag = ray1.norm() * ray2.norm();
  const double dotAngle = ray1.dot(ray2);
  return radianToDegree(acos(clamp(dotAngle/mag, -1.0 + 1.e-8, 1.0 - 1.e-8)));
}

/**
 * @brief Return the angle (degree) between two bearing vector rays
 * @param[in] pose1 First pose
 * @param[in] intrinsic1 First intrinsic
 * @param[in] pose2 Second pose
 * @param[in] intrinsic2 Second intrinsic
 * @param[in] x1 First image point
 * @param[in] x2 Second image point
 * @return The angle (degree) between two bearing vector rays
 */
inline double angleBetweenRays(const geometry::Pose3& pose1,
                               const IntrinsicBase* intrinsic1,
                               const geometry::Pose3& pose2,
                               const IntrinsicBase* intrinsic2,
                               const Vec2& x1,
                               const Vec2& x2)
{
  const Vec3 ray1 = applyIntrinsicExtrinsic(pose1, intrinsic1, x1);
  const Vec3 ray2 = applyIntrinsicExtrinsic(pose2, intrinsic2, x2);
  return angleBetweenRays(ray1, ray2);
}

/**
 * @brief Return the angle (degree) between two poses and a 3D point.
 * @param[in] pose1 First pose
 * @param[in] pose2 Second Pose
 * @param[in] pt3D The 3d point
 * @return The angle (degree) between two poses and a 3D point.
 */
inline double angleBetweenRays(const geometry::Pose3& pose1,
                               const geometry::Pose3& pose2,
                               const Vec3& pt3D)
{
  const Vec3 ray1 = pt3D - pose1.center();
  const Vec3 ray2 = pt3D - pose2.center();
  return angleBetweenRays(ray1, ray2);
}

} // namespace camera

template <size_t M, size_t N>
Eigen::Matrix<double, M*N, M*N> getJacobian_At_wrt_A()
{
	Eigen::Matrix<double, M*N, M*N> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(IAtB) = kron(B.t, I) * vec(A) */
	/** dvec(IAtB)/dA = kron(B.t, I) * dvec(At)/dA */

	ret.fill(0);

	size_t pos_at = 0;
	for (size_t i = 0; i < M; i++)
    {
		for (size_t j = 0; j < N; j++)
        {
			size_t pos_a = N * j + i;
			ret(pos_at, pos_a) = 1;

			pos_at++;
		}
	}

	return ret;
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, M*N> getJacobian_AB_wrt_A(const Eigen::Matrix<double, M , N> & A, const Eigen::Matrix<double, N, K> & B)
{
	Eigen::Matrix<double, M*K, M*N> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(IAB) = kron(B.t, I) * vec(A) */
	/** dvec(IAB)/dA = kron(B.t, I) * dvec(A)/dA */
	/** dvec(IAB)/dA = kron(B.t, I) */

	ret.fill(0);

	Eigen::Matrix<double, K, N> Bt = B.transpose();

	for (size_t row = 0; row < K; row++)
    {
		for (size_t col = 0; col < N; col++)
        {
			ret.template block<M, M>(row * M, col * M) = Bt(row, col) * Eigen::Matrix<double, M, M>::Identity();
		}
	}

	return ret;
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, M*N> getJacobian_AtB_wrt_A(const Eigen::Matrix<double, M, N> & A, const Eigen::Matrix<double, M, K> & B)
{
	return getJacobian_AB_wrt_A<M, N, K>(A.transpose(), B) * getJacobian_At_wrt_A<M, N>();
}

template <size_t M, size_t N, size_t K>
Eigen::Matrix<double, M*K, N*K> getJacobian_AB_wrt_B(const Eigen::Matrix<double, M, N> & A, const Eigen::Matrix<double, N, K> & B)
{
	Eigen::Matrix<double, M*K, N*K> ret;

	/** vec(M1*M2*M3) = kron(M3.t, M1) * vec(M2) */
	/** vec(ABI) = kron(I, A) * vec(B) */
	/** dvec(ABI)/dB = kron(I, A) * dvec(B)/dB */
	/** dvec(ABI)/dB = kron(I, A) */

	ret.fill(0);

	for (size_t index = 0; index < K; index++)
    {
		ret.template block<M, N>(M * index, N * index) = A;
	}

	return ret;
}

} // namespace aliceVision
