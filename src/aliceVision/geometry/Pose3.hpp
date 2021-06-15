// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace geometry {

// Define a 3D Pose as a 3D transform: [R|C] t = -RC
class Pose3
{
  protected:
    Mat3 _rotation;
    Vec3 _center;

  public:
    // Constructors
    Pose3() : _rotation(Mat3::Identity()), _center(Vec3::Zero()) {}
    Pose3(const Mat3& r, const Vec3& c) : _rotation(r), _center(c) {}
    Pose3(const Mat34& Rt)
    : _rotation(Rt.block<3,3>(0,0))
    {
      Vec3 t = Rt.block<3, 1>(0,3);
      _center = -_rotation.transpose() * t;
    }

    // Accessors
    const Mat3& rotation() const { return _rotation; }
    Mat3& rotation() { return _rotation; }
    const Vec3& center() const { return _center; }
    Vec3& center() { return _center; }

    // Translation vector t = -RC
    inline Vec3 translation() const { return -(_rotation * _center); }

    // Apply pose
    inline Mat3X operator () (const Mat3X& p) const
    {
      return _rotation * (p.colwise() - _center);
    }

    // Composition
    Pose3 operator * (const Pose3& P) const
    {
      return Pose3(_rotation * P._rotation, P._center + P._rotation.transpose() * _center );
    }

    // Operator ==
    bool operator==(const Pose3& other) const
    {
      return AreMatNearEqual(_rotation, other._rotation, 1e-6) &&
              AreVecNearEqual(_center, other._center, 1e-6);
    }

    // Inverse
    Pose3 inverse() const
    {
      return Pose3(_rotation.transpose(),  -(_rotation * _center));
    }

    /// Return the depth (distance) of a point respect to the camera center
    double depth(const Vec3 &X) const
    {
      return (_rotation * (X - _center))[2];
    }

    /// Return the pose with the Scale, Rotation and translation applied.
    Pose3 transformSRt(const double S, const Mat3 & R, const Vec3 & t) const
    {
      Pose3 pose;
      pose._center = S * R * _center + t;
      pose._rotation = _rotation * R.transpose();
      return pose;
    }

    Mat4 getHomogeneous() const 
    {
      Mat4 ret = Mat4::Identity();

      ret.block<3, 3>(0, 0) = _rotation;
      ret.block<3, 1>(0, 3) = translation();

      return ret;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Build a pose from a rotation and a translation.
 * @param[in] R The 3x3 rotation.
 * @param[in] t The 3x1 translation.
 * @return The pose as [R, -R'*t]
 */
inline Pose3 poseFromRT(const Mat3& R, const Vec3& t) 
{
  return Pose3(R, -R.transpose()*t);
}

inline Pose3 randomPose()
{
    using namespace Eigen;
    Vec3 rAngles = Vec3::Random();
    Mat3 R;
    R = AngleAxisd(rAngles(0), Vec3::UnitZ())
        * AngleAxisd(rAngles(1), Vec3::UnitY())
        * AngleAxisd(rAngles(2), Vec3::UnitZ());
    return geometry::Pose3(R, Vec3::Random());
}

} // namespace geometry
} // namespace aliceVision
