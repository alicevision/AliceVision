// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <ceres/rotation.h>

#include <memory>

namespace aliceVision {
namespace sfm {

/**
 * @brief Transform a point into a pose frame: out = R * (in - c)
 * @param[in] pose 6 parameters: angle-axis rotation R followed by center c
 * @param[in] in the input point
 * @param[out] out the transformed point (may alias in)
 */
template<typename T>
inline void applyPose(const T* pose, const T* in, T* out)
{
    const T* cam_R = pose;
    const T* cam_c = &pose[3];

    T centeredPoint[3];
    centeredPoint[0] = in[0] - cam_c[0];
    centeredPoint[1] = in[1] - cam_c[1];
    centeredPoint[2] = in[2] - cam_c[2];

    // Rotate the point according the camera rotation
    ceres::AngleAxisRotatePoint(cam_R, centeredPoint, out);
}

/**
 * @brief Size of the distortion parameter block for an intrinsic (1 when there is no distortion, to use a fake block)
 */
inline int getDistortionSize(const std::shared_ptr<camera::IntrinsicBase>& intrinsic)
{
    auto isod = camera::IntrinsicScaleOffsetDisto::cast(intrinsic);
    if (isod)
    {
        auto distortion = isod->getDistortion();
        if (distortion)
        {
            return distortion->getParameters().size();
        }
    }

    return 1;
}

}  // namespace sfm
}  // namespace aliceVision
