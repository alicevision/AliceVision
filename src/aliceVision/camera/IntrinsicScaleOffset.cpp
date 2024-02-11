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

}  // namespace camera
}  // namespace aliceVision
