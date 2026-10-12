// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicScaleOffsetDisto.hpp"

namespace aliceVision {
namespace camera {

std::shared_ptr<IntrinsicScaleOffsetDisto> IntrinsicScaleOffsetDisto::cast(std::shared_ptr<IntrinsicBase> sptr)
{
    return std::dynamic_pointer_cast<IntrinsicScaleOffsetDisto>(sptr);
}

bool IntrinsicScaleOffsetDisto::operator==(const IntrinsicBase& otherBase) const
{
    return equalTo(otherBase, false);
}

bool IntrinsicScaleOffsetDisto::equalTo(const IntrinsicBase& otherBase, bool ignoreDistortion) const
{
    if (!IntrinsicScaleOffset::operator==(otherBase))
    {
        return false;
    }

    if (typeid(*this) != typeid(otherBase))
    {
        return false;
    }

    if (ignoreDistortion)
    {
        return true;
    }

    const IntrinsicScaleOffsetDisto& other = static_cast<const IntrinsicScaleOffsetDisto&>(otherBase);

    if (_distortionInitializationMode != other._distortionInitializationMode)
    {
        return false;
    }

    int countNonNull = 0;
    if (_pDistortion == nullptr)
    {
        countNonNull++;
    }
    if (other._pDistortion == nullptr)
    {
        countNonNull++;
    }
    if (countNonNull == 1)
    {
        return false;
    }

    countNonNull = 0;
    if (_pUndistortion == nullptr)
    {
        countNonNull++;
    }
    if (other._pUndistortion == nullptr)
    {
        countNonNull++;
    }
    if (countNonNull == 1)
    {
        return false;
    }

    if (_pDistortion && other._pDistortion)
    {
        if (!((*_pDistortion) == (*other._pDistortion)))
        {
            return false;
        }
    }

    if (_pUndistortion && other._pUndistortion)
    {
        if (!((*_pUndistortion) == (*other._pUndistortion)))
        {
            return false;
        }
    }

    return true;
}

Vec2 IntrinsicScaleOffsetDisto::getUndistortedPixel(const Vec2& p) const { return cam2ima(removeDistortion(ima2cam(p))); }

Vec2 IntrinsicScaleOffsetDisto::getDistortedPixel(const Vec2& p) const { return cam2ima(addDistortion(ima2cam(p))); }

void IntrinsicScaleOffsetDisto::rescale(float factorW, float factorH)
{
    IntrinsicScaleOffset::rescale(factorW, factorH);

    // The distortion works in the camera plane and does not depend on the image size.
    if (!_pUndistortion)
    {
        return;
    }

    // The undistortion normalizes x by the diagonal and y by the diagonal times a ratio:
    // 1 if the image is desqueezed, the pixel aspect ratio otherwise.
    const double pixelAspectRatio = _pUndistortion->getPixelAspectRatio();
    const double ratio = _pUndistortion->isDesqueezed() ? 1.0 : pixelAspectRatio;
    const double newRatio = ratio * factorH / factorW;

    const double diagonal = _pUndistortion->getDiagonal() * factorW;
    const Vec2 scaledCenter = _pUndistortion->getCenter().cwiseProduct(Vec2(factorW, factorH));

    _pUndistortion->setSize(static_cast<int>(_w), static_cast<int>(_h));

    // setSize() rounds the center down to an integer for odd sizes: the offset keeps the scaled center.
    const Vec2 offset = scaledCenter - (_pUndistortion->getCenter() - _pUndistortion->getOffset());

    const double eps = 1e-6;
    if (std::abs(newRatio - 1.0) < eps)
    {
        // Typically, desqueezing an anamorphic image.
        _pUndistortion->setDesqueezed(true);
    }
    else if (std::abs(newRatio - pixelAspectRatio) < eps)
    {
        _pUndistortion->setDesqueezed(false);
    }
    else
    {
        // Only exact for models which use the pixel aspect ratio for the normalization alone:
        // the anamorphic model also uses it as the lens squeeze.
        _pUndistortion->setDesqueezed(false);
        _pUndistortion->setPixelAspectRatio(newRatio);
    }

    _pUndistortion->setOffset(offset);
    _pUndistortion->setDiagonal(diagonal);
}

bool IntrinsicScaleOffsetDisto::updateFromParams(const std::vector<double>& params)
{
    if (!IntrinsicScaleOffset::updateFromParams(params))
    {
        return false;
    }

    if (_pDistortion && params.size() == 4 + _pDistortion->getParameters().size())
    {
        setDistortionParams({params.begin() + 4, params.end()});
    }

    return true;
}

}  // namespace camera
}  // namespace aliceVision
