// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Equirectangular.hpp"

#include <cmath>

namespace aliceVision {
namespace camera {

std::shared_ptr<Equirectangular> Equirectangular::cast(std::shared_ptr<IntrinsicBase> sptr)
{
    return std::dynamic_pointer_cast<Equirectangular>(sptr);
}

Vec2 Equirectangular::transformProject(const Eigen::Matrix4d& pose, const Vec4& pt, [[maybe_unused]] bool applyDistortion) const
{
    Vec4 X = pose * pt;
    Vec3 spherical = X.head(3).normalized();

    Vec2 angles;
    angles.x() = atan2(spherical(0), spherical(2));
    angles.y() = asin(spherical(1));

    Vec2 imapt = cam2ima(angles);

    return imapt;
}

Vec2 Equirectangular::project(const Vec4& pt, [[maybe_unused]] bool applyDistortion) const
{
    Vec3 spherical = pt.head(3).normalized();

    Vec2 angles;
    angles.x() = atan2(spherical(0), spherical(2));
    angles.y() = asin(spherical(1));

    Vec2 imapt = cam2ima(angles);

    return imapt;
}

Eigen::Matrix<double, 2, 3> Equirectangular::getDerivativeTransformProjectWrtPoint3(const Eigen::Matrix4d& T, const Vec4& pt) const
{
    const Vec4 X = T * pt;
    const Vec3 Xc = X.head(3);
    const double norm = Xc.norm();
    const Vec3 spherical = Xc / norm;

    const double sx = spherical(0);
    const double sy = spherical(1);
    const double sz = spherical(2);

    // spherical = Xc / |Xc|
    const Eigen::Matrix3d d_spherical_d_Xc = (Eigen::Matrix3d::Identity() - spherical * spherical.transpose()) / norm;

    // atan2(sx, sz) ;  asin(sy)
    const double lenxz = sx * sx + sz * sz;
    Eigen::Matrix<double, 2, 3> d_coords_d_spherical = Eigen::Matrix<double, 2, 3>::Zero();
    d_coords_d_spherical(0, 0) = sz / lenxz;
    d_coords_d_spherical(0, 2) = -sx / lenxz;
    d_coords_d_spherical(1, 1) = 1.0 / sqrt(1.0 - (sy * sy));

    return getDerivativeCam2ImaWrtPoint() * d_coords_d_spherical * d_spherical_d_Xc * T.block<3, 3>(0, 0);
}

Eigen::Matrix<double, 2, 3> Equirectangular::getDerivativeTransformProjectWrtDisto([[maybe_unused]] const Eigen::Matrix4d& pose,
                                                                                   [[maybe_unused]] const Vec4& pt) const
{
    return Eigen::Matrix<double, 2, 3>::Zero();
}

Eigen::Matrix<double, 2, 2> Equirectangular::getDerivativeTransformProjectWrtScale(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    const Vec4 X = pose * pt;
    Vec3 spherical = X.head(3).normalized();

    Vec2 angles;
    angles.x() = atan2(spherical(0), spherical(2));
    angles.y() = asin(spherical(1));

    return getDerivativeCam2ImaWrtScale(angles);
}

Eigen::Matrix<double, 2, 2> Equirectangular::getDerivativeTransformProjectWrtPrincipalPoint([[maybe_unused]] const Eigen::Matrix4d& pose,
                                                                                            [[maybe_unused]] const Vec4& pt) const
{
    return getDerivativeCam2ImaWrtPrincipalPoint();
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Equirectangular::getDerivativeTransformProjectWrtParams(const Eigen::Matrix4d& pose, const Vec4& pt3D) const
{
    Eigen::Matrix<double, 2, Eigen::Dynamic> ret(2, getParameters().size());

    ret.block<2, 2>(0, 0) = getDerivativeTransformProjectWrtScale(pose, pt3D);
    ret.block<2, 2>(0, 2) = getDerivativeTransformProjectWrtPrincipalPoint(pose, pt3D);

    return ret;
}

Vec3 Equirectangular::toUnitSphere(const Vec2& pt) const
{
    const double latitude = pt(1);
    const double longitude = pt(0);

    Vec3 spherical;
    spherical.x() = cos(latitude) * sin(longitude);
    spherical.y() = sin(latitude);
    spherical.z() = cos(latitude) * cos(longitude);

    return spherical;
}

Eigen::Matrix<double, 3, 2> Equirectangular::getDerivativetoUnitSphereWrtPoint(const Vec2& pt) const
{
    const double latitude = pt(1);
    const double longitude = pt(0);

    // spherical = (cos(lat) sin(lon), sin(lat), cos(lat) cos(lon))
    Eigen::Matrix<double, 3, 2> d_spherical_d_pt;
    d_spherical_d_pt(0, 0) = cos(latitude) * cos(longitude);
    d_spherical_d_pt(1, 0) = 0.0;
    d_spherical_d_pt(2, 0) = -cos(latitude) * sin(longitude);
    d_spherical_d_pt(0, 1) = -sin(latitude) * sin(longitude);
    d_spherical_d_pt(1, 1) = cos(latitude);
    d_spherical_d_pt(2, 1) = -sin(latitude) * cos(longitude);

    return d_spherical_d_pt;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Equirectangular::getDerivativeBackProjectUnitWrtParams(const Vec2& pt2D) const
{
    const Vec2 ptMeters = ima2cam(pt2D);

    Eigen::Matrix<double, 3, Eigen::Dynamic> ret(3, 4);

    Eigen::Matrix<double, 3, 2> J = getDerivativetoUnitSphereWrtPoint(ptMeters);

    ret.block<3, 2>(0, 0) = J * getDerivativeIma2CamWrtScale(pt2D);
    ret.block<3, 2>(0, 2) = J * getDerivativeIma2CamWrtPrincipalPoint();

    return ret;
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Equirectangular::getDerivativeTransformProjectWrtDistortion([[maybe_unused]] const Eigen::Matrix4d& pose,
                                                                                                     [[maybe_unused]] const Vec4& pt) const
{
    // No distortion parameter
    return Eigen::Matrix<double, 2, Eigen::Dynamic>(2, 0);
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Equirectangular::getDerivativeBackProjectUnitWrtDistortion([[maybe_unused]] const Vec2& pt2D) const
{
    // No distortion parameter
    return Eigen::Matrix<double, 3, Eigen::Dynamic>(3, 0);
}

double Equirectangular::imagePlaneToCameraPlaneError([[maybe_unused]] double value) const { return 0.0; }

bool Equirectangular::isVisibleRay([[maybe_unused]] const Vec3& ray) const { return true; }

EINTRINSIC Equirectangular::getType() const { return EINTRINSIC::EQUIRECTANGULAR_CAMERA; }

double Equirectangular::getHorizontalFov() const { return 2.0 * M_PI; }

double Equirectangular::getVerticalFov() const { return M_PI; }

double Equirectangular::pixelProbability() const { return 1.0 / double(w()); }

}  // namespace camera
}  // namespace aliceVision
