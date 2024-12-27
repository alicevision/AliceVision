// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Equirectangular.hpp"

#include <algorithm>
#include <cmath>

namespace aliceVision {
namespace camera {

std::shared_ptr<Equirectangular> Equirectangular::cast(std::shared_ptr<IntrinsicBase> sptr)
{
    return std::dynamic_pointer_cast<Equirectangular>(sptr);
}

Vec2 Equirectangular::transformProject(const Eigen::Matrix4d& pose, const Vec4& pt, bool applyDistortion) const
{
    Vec4 X = pose * pt;
    Vec3 spherical = X.head(3).normalized();

    Vec2 angles;
    angles.x() = atan2(spherical(0), spherical(2));
    angles.y() = asin(spherical(1));

    Vec2 imapt = cam2ima(angles);

    return imapt;
}

Vec2 Equirectangular::project(const Vec4& pt, bool applyDistortion) const
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
    Vec3 spherical = pt.head(3).normalized();
    
    double sx = spherical(0);
    double sy = spherical(1);
    double sz = spherical(2);
    double len = sx*sx + sy*sy;

    double norm = pt.norm();
    double normsq = norm * norm;
    double invnormsq = 1.0 / normsq;

    double d_norm_d_x = pt.x() / norm;
    double d_norm_d_y = pt.y() / norm;
    double d_norm_d_z = pt.z() / norm;

    // x / norm; y / norm; z / norm
    Eigen::Matrix<double, 3, 3> d_spherical_d_pt3; 
    d_spherical_d_pt3(0, 0) = (norm * 1.0 - pt.x() * d_norm_d_x) * invnormsq;
    d_spherical_d_pt3(0, 1) = (norm * 0.0 - pt.x() * d_norm_d_y) * invnormsq;
    d_spherical_d_pt3(0, 2) = (norm * 0.0 - pt.x() * d_norm_d_z) * invnormsq;
    d_spherical_d_pt3(1, 0) = (norm * 0.0 - pt.y() * d_norm_d_x) * invnormsq;
    d_spherical_d_pt3(1, 1) = (norm * 1.0 - pt.y() * d_norm_d_y) * invnormsq;
    d_spherical_d_pt3(1, 2) = (norm * 0.0 - pt.y() * d_norm_d_z) * invnormsq;
    d_spherical_d_pt3(2, 0) = (norm * 0.0 - pt.z() * d_norm_d_x) * invnormsq;
    d_spherical_d_pt3(2, 1) = (norm * 0.0 - pt.z() * d_norm_d_y) * invnormsq;
    d_spherical_d_pt3(2, 2) = (norm * 1.0 - pt.z() * d_norm_d_z) * invnormsq;
    
    
    //atan2(spherical(0), spherical(2)) ;  asin(spherical(1));
    Eigen::Matrix<double, 2, 3> d_coords_d_spherical = Eigen::Matrix<double, 2, 3>::Zero();
    d_coords_d_spherical(0, 0) = sz / len;
    d_coords_d_spherical(0, 2) = - sx / len;
    d_coords_d_spherical(1, 1) = 1.0 / sqrt(1.0 - (sy*sy));
    

    return getDerivativeCam2ImaWrtPoint() * d_coords_d_spherical * d_spherical_d_pt3;
}

Eigen::Matrix<double, 2, 3> Equirectangular::getDerivativeTransformProjectWrtDisto(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    return Eigen::Matrix<double, 2, 3>::Zero();
}

Eigen::Matrix<double, 2, 2> Equirectangular::getDerivativeTransformProjectWrtScale(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    Vec3 spherical = pt.head(3).normalized();

    Vec2 angles;
    angles.x() = atan2(spherical(0), spherical(2));
    angles.y() = asin(spherical(1));

    return getDerivativeCam2ImaWrtScale(angles);
}

Eigen::Matrix<double, 2, 2> Equirectangular::getDerivativeTransformProjectWrtPrincipalPoint(const Eigen::Matrix4d& pose, const Vec4& pt) const
{
    return getDerivativeCam2ImaWrtPrincipalPoint();
}

Eigen::Matrix<double, 2, Eigen::Dynamic> Equirectangular::getDerivativeTransformProjectWrtParams(const Eigen::Matrix4d& pose, const Vec4& pt3D) const
{
    Eigen::Matrix<double, 2, Eigen::Dynamic> ret(2, getParams().size());

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

    Vec3 spherical;
    spherical.x() = cos(latitude) * sin(longitude);
    spherical.y() = sin(latitude);
    spherical.z() = cos(latitude) * cos(longitude);

    Eigen::Matrix<double, 3, 2> d_spherical_d_pt;
    d_spherical_d_pt(0, 0) = cos(latitude) * cos(longitude); 
    d_spherical_d_pt(0, 1) = -sin(latitude) * sin(longitude); 
    d_spherical_d_pt(1, 0) = 0; 
    d_spherical_d_pt(1, 1) = cos(latitude); 
    d_spherical_d_pt(0, 0) = cos(latitude) * -sin(longitude); 
    d_spherical_d_pt(0, 1) = -sin(latitude) * cos(longitude); 

    return d_spherical_d_pt;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Equirectangular::getDerivativeBackProjectUnitWrtParams(const Vec2& pt2D) const 
{
    const Vec2 ptMeters = ima2cam(pt2D);
    const Vec3 ptSphere = toUnitSphere(ptMeters);

    Eigen::Matrix<double, 3, Eigen::Dynamic> ret(3, 4);

    Eigen::Matrix<double, 3, 2> J = getDerivativetoUnitSphereWrtPoint(ptMeters);

    ret.block<3, 2>(0, 0) = J * getDerivativeIma2CamWrtScale(pt2D);
    ret.block<3, 2>(0, 2) = J * getDerivativeIma2CamWrtPrincipalPoint();
    
    return ret;
}

double Equirectangular::imagePlaneToCameraPlaneError(double value) const { return 0.0; }

bool Equirectangular::isVisibleRay(const Vec3& ray) const
{
    return true;
}

EINTRINSIC Equirectangular::getType() const
{
    return EINTRINSIC::EQUIRECTANGULAR_CAMERA;
}

double Equirectangular::getHorizontalFov() const
{
    return 2.0 * M_PI;
}

double Equirectangular::getVerticalFov() const { return M_PI; }

double Equirectangular::pixelProbability() const
{
    return 1.0 / double(w());
}



}  // namespace camera
}  // namespace aliceVision
