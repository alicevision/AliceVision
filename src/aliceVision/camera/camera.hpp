// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/camera/Distortion3DE.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>
#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/camera/Undistortion3DE.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>

#include <memory>

namespace aliceVision {
namespace camera {

inline std::shared_ptr<IntrinsicBase> createIntrinsic(EINTRINSIC intrinsicType,
    unsigned int w = 0, unsigned int h = 0,
    double focalLengthPixX = 0.0, double focalLengthPixY = 0.0,
    double offsetX = 0.0, double offsetY = 0.0)
{
    // Create distortion and undistortion objects
    std::shared_ptr<Distortion> distortion = nullptr;
    std::shared_ptr<Undistortion> undistortion = nullptr;
    switch (intrinsicType)
    {
    case EINTRINSIC::PINHOLE_CAMERA:
        break;
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
        distortion = std::make_shared<DistortionRadialK1>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
        distortion = std::make_shared<DistortionRadialK3>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4:
        distortion = std::make_shared<Distortion3DERadial4>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
        distortion = std::make_shared<DistortionBrown>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
        distortion = std::make_shared<DistortionFisheye>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
        distortion = std::make_shared<DistortionFisheye1>();
        break;
    case EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4:
        undistortion = std::make_shared<Undistortion3DEAnamorphic4>(w, h);
        break;
    case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD:
        distortion = std::make_shared<Distortion3DEClassicLD>();
        break;
    case EINTRINSIC::EQUIDISTANT_CAMERA:
        break;
    case EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3:
        distortion = std::make_shared<DistortionRadialK3PT>();
        break;
    case EINTRINSIC::UNKNOWN:
    case EINTRINSIC::VALID_PINHOLE:
    case EINTRINSIC::VALID_EQUIDISTANT:
    case EINTRINSIC::VALID_CAMERA_MODEL:
        break;
    }

    // Create intrinsics
    switch (intrinsicType)
    {
    case EINTRINSIC::PINHOLE_CAMERA:
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
    case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
    case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4:
    case EINTRINSIC::PINHOLE_CAMERA_BROWN:
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
    case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
    case EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4:
    case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD:
        return std::make_shared<Pinhole>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion);
    case EINTRINSIC::EQUIDISTANT_CAMERA:
    case EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3:
        return std::make_shared<Equidistant>(w, h, focalLengthPixX, offsetX, offsetY, distortion);
    case EINTRINSIC::UNKNOWN:
    case EINTRINSIC::VALID_PINHOLE:
    case EINTRINSIC::VALID_EQUIDISTANT:
    case EINTRINSIC::VALID_CAMERA_MODEL:
        break;
    }

    throw std::out_of_range("Unrecognized Intrinsic Enum");
}

} // namespace camera
} // namespace aliceVision
