// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicScaleOffset.hpp"

#include <aliceVision/version.hpp>

namespace aliceVision {
namespace camera {

bool IntrinsicScaleOffset::operator==(const IntrinsicBase& otherBase) const
{
    if (!IntrinsicBase::operator==(otherBase))
    {
        return false;
    }

    if (typeid(*this) != typeid(otherBase))
    {
        return false;
    }

    const IntrinsicScaleOffset& other = static_cast<const IntrinsicScaleOffset&>(otherBase);

    return _scale.isApprox(other._scale) && _offset.isApprox(other._offset);
}

Vec2 IntrinsicScaleOffset::cam2ima(const Vec2& p) const { return p.cwiseProduct(_scale) + getPrincipalPoint(); }

Eigen::Matrix2d IntrinsicScaleOffset::getDerivativeCam2ImaWrtScale(const Vec2& p) const
{
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = p(0);
    M(1, 1) = p(1);

    return M;
}

Eigen::Matrix2d IntrinsicScaleOffset::getDerivativeCam2ImaWrtPoint() const
{
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = _scale(0);
    M(1, 1) = _scale(1);

    return M;
}

Eigen::Matrix2d IntrinsicScaleOffset::getDerivativeCam2ImaWrtPrincipalPoint() const { return Eigen::Matrix2d::Identity(); }

Vec2 IntrinsicScaleOffset::ima2cam(const Vec2& p) const
{
    Vec2 np;

    Vec2 pp = getPrincipalPoint();

    np(0) = (p(0) - pp(0)) / _scale(0);
    np(1) = (p(1) - pp(1)) / _scale(1);

    return np;
}

Eigen::Matrix<double, 2, 2> IntrinsicScaleOffset::getDerivativeIma2CamWrtScale(const Vec2& p) const
{
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    Vec2 pp = getPrincipalPoint();

    M(0, 0) = -(p(0) - pp(0)) / (_scale(0) * _scale(0));
    M(1, 1) = -(p(1) - pp(1)) / (_scale(1) * _scale(1));

    return M;
}

Eigen::Matrix2d IntrinsicScaleOffset::getDerivativeIma2CamWrtPoint() const
{
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = 1.0 / _scale(0);
    M(1, 1) = 1.0 / _scale(1);

    return M;
}

Eigen::Matrix2d IntrinsicScaleOffset::getDerivativeIma2CamWrtPrincipalPoint() const
{
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();

    M(0, 0) = -1.0 / _scale(0);
    M(1, 1) = -1.0 / _scale(1);

    return M;
}

void IntrinsicScaleOffset::rescale(float factorW, float factorH)
{
    IntrinsicBase::rescale(factorW, factorH);

    _scale(0) *= factorW;
    _scale(1) *= factorH;
    _offset(0) *= factorW;
    _offset(1) *= factorH;
    _initialScale(0) *= factorW;
    _initialScale(1) *= factorH;
}

bool IntrinsicScaleOffset::updateFromParams(const std::vector<double>& params)
{
    if (params.size() < 4)
    {
        return false;
    }

    _scale(0) = params[0];
    _scale(1) = params[1];
    _offset(0) = params[2];
    _offset(1) = params[3];

    return true;
}

bool IntrinsicScaleOffset::importFromParams(const std::vector<double>& params, const Version& inputVersion)
{
    std::vector<double> paramsLocal;
    if (inputVersion < Version(1, 2, 0))
    {
        paramsLocal.resize(params.size() + 1);
        paramsLocal[0] = params[0];
        paramsLocal[1] = params[0];

        for (int i = 1; i < params.size(); i++)
        {
            paramsLocal[i + 1] = params[i];
        }
    }
    else
    {
        paramsLocal = params;
    }

    if (!updateFromParams(paramsLocal))
    {
        return false;
    }

    if (inputVersion < Version(1, 2, 1))
    {
        _offset(0) -= static_cast<double>(_w) / 2.0;
        _offset(1) -= static_cast<double>(_h) / 2.0;
    }

    return true;
}

double IntrinsicScaleOffset::getFocalLength() const
{
    const double maxSize = static_cast<double>(std::max(w(), h()));
    const double fx = _scale(0);
    const double fy = _scale(1);

    //The bigger the scale or focal (which have the same direction)
    //The smaller the observed area
    const double focalInMillimeters = fx * sensorWidth() / maxSize;
    
    //Pixel aspect ratio is the how the x dimension is stretched
    //That means that it enlarge the observed area
    //That means a larger pixel ratio leads to a smaller focal (in X).
    const double pixelAspectRatio = getPixelAspectRatio();
    
    //Assumming the focal length is *ignoring* the stretch
    //Thus the returned focal is the bigger focal canceling the pixelAspectRatio
    return focalInMillimeters * pixelAspectRatio;
}

double IntrinsicScaleOffset::getInitialFocalLength() const
{
    const double maxSize = static_cast<double>(std::max(w(), h()));
    const double fx = _initialScale(0);
    const double fy = _initialScale(1);

    if (fx < 0)
    {
        return -1.0;
    }

    //The bigger the scale or focal (which have the same direction)
    //The smaller the observed area
    const double focalInMillimeters = fx * sensorWidth() / maxSize;
    
    //Pixel aspect ratio is the how the x dimension is stretched
    //That means that it enlarge the observed area
    //That means a larger pixel ratio leads to a smaller focal (in X).
    const double pixelAspectRatio = getPixelAspectRatio();
    
    //Assumming the focal length is *ignoring* the stretch
    //Thus the returned focal is the bigger focal canceling the pixelAspectRatio
    return focalInMillimeters * pixelAspectRatio;
}

double IntrinsicScaleOffset::getPixelAspectRatio() const
{
    const double fx = _scale(0);
    const double fy = _scale(1);

    const double focalRatio = fx / fy;
    const double pixelAspectRatio = 1.0 / focalRatio;

    return pixelAspectRatio;
}

void IntrinsicScaleOffset::setFocalLength(double focalInMillimeters, double pixelAspectRatio, bool useCompatibility)
{
    if (useCompatibility)
    {
        const double focalInMillimetersY = focalInMillimeters * pixelAspectRatio;
        const double millimetersToPixels = double(w()) / sensorWidth();
        _scale(0) = focalInMillimeters * millimetersToPixels;
        _scale(1) = focalInMillimetersY * millimetersToPixels;
    }
    else 
    {
        //We assume focalInMillimeters ignore the pixelAspectRatio in X
        const double focalInMillimetersX = focalInMillimeters / pixelAspectRatio;
        const double millimetersToPixels = double(w()) / sensorWidth();
        _scale(0) = focalInMillimetersX * millimetersToPixels;
        _scale(1) = focalInMillimeters * millimetersToPixels;
    }
}

void IntrinsicScaleOffset::setInitialFocalLength(double initialFocalInMillimeters, double pixelAspectRatio, bool useCompatibility)
{
    if (initialFocalInMillimeters < 0.0)
    {
        _initialScale(0) = -1.0;
        _initialScale(1) = -1.0;
    }

    const double millimetersToPixels = double(w()) / sensorWidth();
    
    if (useCompatibility)
    {
        //We assume focalInMillimeters ignore the pixelAspectRatio in X
        const double initialFocalInMillimetersY = initialFocalInMillimeters * pixelAspectRatio;
        _initialScale(0) = initialFocalInMillimeters * millimetersToPixels;
        _initialScale(1) = initialFocalInMillimetersY * millimetersToPixels;
    }
    else
    {
        //We assume focalInMillimeters ignore the pixelAspectRatio in X
        const double initialFocalInMillimetersX = initialFocalInMillimeters / pixelAspectRatio;
        _initialScale(0) = initialFocalInMillimetersX * millimetersToPixels;
        _initialScale(1) = initialFocalInMillimeters * millimetersToPixels;
    }
}


}  // namespace camera
}  // namespace aliceVision
