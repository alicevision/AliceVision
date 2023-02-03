// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IntrinsicsScaleOffsetDisto.hpp"

namespace aliceVision {
namespace camera {

bool IntrinsicsScaleOffsetDisto::operator==(const IntrinsicBase& otherBase) const
{
    if (!IntrinsicsScaleOffset::operator==(otherBase))
        return false;
    if (typeid(*this) != typeid(otherBase))
        return false;
    const IntrinsicsScaleOffsetDisto& other = static_cast<const IntrinsicsScaleOffsetDisto&>(otherBase);

    if (_distortionInitializationMode != other._distortionInitializationMode)
        return false;

    if (_pDistortion != nullptr && other._pDistortion != nullptr)
        return (*_pDistortion) == (*other._pDistortion);
    return _pDistortion == other._pDistortion;
}

Vec2 IntrinsicsScaleOffsetDisto::get_ud_pixel(const Vec2& p) const
{
    return cam2ima(removeDistortion(ima2cam(p)));
}

Vec2 IntrinsicsScaleOffsetDisto::get_d_pixel(const Vec2& p) const
{
    return cam2ima(addDistortion(ima2cam(p)));
}

} // namespace camera
} // namespace aliceVision
