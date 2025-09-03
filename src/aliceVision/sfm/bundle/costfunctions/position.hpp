// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>

namespace aliceVision {
namespace sfm {

/**
 * Assume the intrinsics are known and the camera rotation is known
 * The observation is backProjected on a unit sphere and rotated back to be represented
 * in world coordinates (with respect to the position of the camera).
 * 
 * Thus the error is e = observation - scale * (point - centrecamera)
 * 
 * scale, point and centrecamera are unknown
*/
struct PositioningErrorFunctor
{
    explicit PositioningErrorFunctor(const Vec3 & obs) : _obs(obs)       
    {        
    }

    template<typename T>
    bool operator()(T const* const* parameters, T* residuals) const
    {       
        const T* parameter_center = parameters[0];
        const T* parameter_landmark = parameters[1];
        const T* parameter_scaler = parameters[2];

        residuals[0] = _obs.x() - (parameter_scaler[0] * (parameter_landmark[0] - parameter_center[0]));
        residuals[1] = _obs.y() - (parameter_scaler[0] * (parameter_landmark[1] - parameter_center[1]));
        residuals[2] = _obs.z() - (parameter_scaler[0] * (parameter_landmark[2] - parameter_center[2]));

        return true;
    }

    Vec3 _obs;
};

}  // namespace sfm
}  // namespace aliceVision
