// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicScaleOffsetDisto.hpp"

namespace aliceVision {
namespace camera {

bool IntrinsicScaleOffsetDisto::operator==(const IntrinsicBase& otherBase) const
{
    if (!IntrinsicScaleOffset::operator==(otherBase))
    {
        return false;
    }

    if (typeid(*this) != typeid(otherBase))
    {
        return false;
    }

    const IntrinsicScaleOffsetDisto& other = static_cast<const IntrinsicScaleOffsetDisto&>(otherBase);

    if (_distortionInitializationMode != other._distortionInitializationMode)
    {
        return false;
    }

    if (_pDistortion && other._pDistortion)
    {
        return (*_pDistortion) == (*other._pDistortion);
    }

    if (_pUndistortion && other._pUndistortion)
    {
        return (*_pUndistortion) == (*other._pUndistortion);
    }

    return _pDistortion == nullptr && other._pDistortion == nullptr && _pUndistortion == nullptr && other._pUndistortion == nullptr;
}

Vec2 IntrinsicScaleOffsetDisto::get_ud_pixel(const Vec2& p) const { return cam2ima(removeDistortion(ima2cam(p))); }

Vec2 IntrinsicScaleOffsetDisto::get_d_pixel(const Vec2& p) const { return cam2ima(addDistortion(ima2cam(p))); }

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
