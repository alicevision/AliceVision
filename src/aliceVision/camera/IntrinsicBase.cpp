// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicBase.hpp"

#include <aliceVision/stl/hash.hpp>

namespace aliceVision {
namespace camera {

bool IntrinsicBase::operator==(const IntrinsicBase& other) const
{
    return _w == other._w && _h == other._h && _sensorWidth == other._sensorWidth && _sensorHeight == other._sensorHeight &&
           _serialNumber == other._serialNumber && _initializationMode == other._initializationMode && getType() == other.getType();
}

Vec3 IntrinsicBase::backproject(const Vec2& pt2D, bool applyUndistortion, const geometry::Pose3& pose, double depth) const
{
    const Vec2 pt2D_cam = ima2cam(pt2D);
    const Vec2 pt2D_undist = applyUndistortion ? removeDistortion(pt2D_cam) : pt2D_cam;

    const Vec3 pt3d = depth * toUnitSphere(pt2D_undist);
    const Vec3 output = pose.inverse()(pt3d);
    return output;
}

Vec4 IntrinsicBase::getCartesianfromSphericalCoordinates(const Vec3& pt)
{
    Vec4 rpt;
    rpt.x() = pt(0);
    rpt.y() = pt(1);
    rpt.z() = 1.0;
    rpt.w() = pt(2);

    return rpt;
}

Eigen::Matrix<double, 4, 3> IntrinsicBase::getDerivativeCartesianfromSphericalCoordinates(const Vec3& pt)
{
    Eigen::Matrix<double, 4, 3> ret = Eigen::Matrix<double, 4, 3>::Zero();

    ret(0, 0) = 1.0;
    ret(1, 1) = 1.0;
    ret(3, 2) = 1.0;

    return ret;
}

bool IntrinsicBase::isVisible(const Vec2& pix) const
{
    if (pix(0) < 0 || pix(0) >= _w || pix(1) < 0 || pix(1) >= _h)
    {
        return false;
    }

    return true;
}

bool IntrinsicBase::isVisible(const Vec2f& pix) const
{
    if (pix(0) < 0 || pix(0) >= _w || pix(1) < 0 || pix(1) >= _h)
    {
        return false;
    }

    return true;
}

float IntrinsicBase::getMaximalDistortion(double min_radius, double max_radius) const
{
    /*Without distortion, obvious*/
    return max_radius;
}

std::size_t IntrinsicBase::hashValue() const
{
    std::size_t seed = 0;
    stl::hash_combine(seed, static_cast<int>(this->getType()));
    stl::hash_combine(seed, _w);
    stl::hash_combine(seed, _h);
    stl::hash_combine(seed, _sensorWidth);
    stl::hash_combine(seed, _sensorHeight);
    stl::hash_combine(seed, _serialNumber);
    const std::vector<double> params = this->getParams();
    for (double param : params)
    {
        stl::hash_combine(seed, param);
    }
    return seed;
}

void IntrinsicBase::rescale(float factorW, float factorH)
{
    _w = static_cast<unsigned int>(floor(static_cast<float>(_w) * factorW));
    _h = static_cast<unsigned int>(floor(static_cast<float>(_h) * factorH));
}

}  // namespace camera
}  // namespace aliceVision
