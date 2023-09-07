// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <boost/math/constants/constants.hpp>
#include <aliceVision/geometry/lie.hpp>


namespace aliceVision {
namespace geometry {

// Define a 3D Pose as a SE3 matrix
class Pose3
{
protected:
    SE3::Matrix _homogeneous{SE3::Matrix::Identity()};

public:
    Pose3() = default;

    Pose3(const Mat3& c_R_w, const Vec3& w_c)
    {
        _homogeneous.setIdentity();
        _homogeneous.block<3, 3>(0, 0) = c_R_w;
        _homogeneous.block<3, 1>(0, 3) = - c_R_w * w_c;
    }

    Pose3(const Mat4& T) : _homogeneous(T)
    {
    }

    // Accessors
    Mat3 rotation() const 
    { 
        return _homogeneous.block<3, 3>(0, 0); 
    }

    void setRotation(const Mat3& rotation)
    {
        _homogeneous.block<3, 3>(0, 0) = rotation;
    }

    Vec3 translation() const
    { 
        return _homogeneous.block<3, 1>(0, 3); 
    }

    Vec3 center() const
    { 
        return - rotation().transpose() * translation(); 
    }

    void setCenter(const Vec3& center)
    {
        _homogeneous.block<3, 1>(0, 3) = - rotation() * center;
    }

    // Apply pose
    inline Mat3X operator () (const Mat3X& p) const
    {
        return rotation() * (p.colwise() - center());
    }

    // Composition
    Pose3 operator * (const Pose3& P) const
    {
        return Pose3(_homogeneous * P._homogeneous);
    }

    // Operator ==
    bool operator==(const Pose3& other) const
    {
        return AreMatNearEqual(rotation(), other.rotation(), 1e-6) &&
               AreVecNearEqual(center(), other.center(), 1e-6);
    }

    // Inverse
    Pose3 inverse() const
    {
        // parameter is center ... which will inverse
        return Pose3(rotation().transpose(),  translation() );
    }

    /// Return the depth (distance) of a point respect to the camera center
    double depth(const Vec3& X) const
    {
        return (rotation() * (X - center()))[2];
    }

    /// Return the pose with the Scale, Rotation and translation applied.
    Pose3 transformSRt(const double S, const Mat3& R, const Vec3& t) const
    {
        Pose3 pose;
        pose.setCenter(S * R * center() + t);
        pose.setRotation(rotation() * R.transpose());

        return pose;
    }

    const SE3::Matrix & getHomogeneous() const 
    {
        return _homogeneous;
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
    return Pose3(R, -R.transpose() * t);
}

inline Pose3 randomPose()
{
    Vec3 vecR = Vec3::Random().normalized() * boost::math::constants::pi<double>();
    
    return geometry::Pose3(SO3::expm(vecR), Vec3::Random());
}

} // namespace geometry
} // namespace aliceVision
