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
#include <initializer_list>

namespace aliceVision {
namespace camera {

/**
 * @brief Factory function to create a distortion object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] params Distortion parameters.
 * @return Shared pointer of initialized distortion object.
 */
inline std::shared_ptr<Distortion> createDistortion(EDISTORTION distortionType, std::initializer_list<double> params = {})
{
    std::shared_ptr<Distortion> distortion = nullptr;

    switch (distortionType)
    {
        case EDISTORTION::DISTORTION_RADIALK1:
            distortion = std::make_shared<DistortionRadialK1>();
            break;
        case EDISTORTION::DISTORTION_RADIALK3:
            distortion = std::make_shared<DistortionRadialK3>();
            break;
        case EDISTORTION::DISTORTION_RADIALK3PT:
            distortion = std::make_shared<DistortionRadialK3PT>();
            break;
        case EDISTORTION::DISTORTION_BROWN:
            distortion = std::make_shared<DistortionBrown>();
            break;
        case EDISTORTION::DISTORTION_FISHEYE:
            distortion = std::make_shared<DistortionFisheye>();
            break;
        case EDISTORTION::DISTORTION_FISHEYE1:
            distortion = std::make_shared<DistortionFisheye1>();
            break;
        case EDISTORTION::DISTORTION_3DECLASSICLD:
            distortion = std::make_shared<Distortion3DEClassicLD>();
            break;
        case EDISTORTION::DISTORTION_3DERADIAL4:
            distortion = std::make_shared<Distortion3DERadial4>();
            break;
        case EDISTORTION::DISTORTION_3DEANAMORPHIC4:
            distortion = std::make_shared<Distortion3DEAnamorphic4>();
            break;
        default:
            break;
    }

    if (distortion && params.size() > 0)
    {
        if (params.size() != distortion->getParameters().size())
        {
            throw std::invalid_argument("Invalid number of distortion parameters");
        }

        distortion->setParameters(params);
    }

    return distortion;
}

/**
 * @brief Factory function to create an undistortion object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] params Distortion parameters.
 * @return Shared pointer of initialized undistortion object.
 */
inline std::shared_ptr<Undistortion> createUndistortion(EDISTORTION distortionType,
                                                        unsigned int w = 0,
                                                        unsigned int h = 0,
                                                        std::initializer_list<double> params = {})
{
    std::shared_ptr<Undistortion> undistortion = nullptr;

    switch (distortionType)
    {
        case EDISTORTION::DISTORTION_3DEANAMORPHIC4:
            undistortion = std::make_shared<Undistortion3DEAnamorphic4>(w, h);
            break;
        default:
            break;
    }

    if (undistortion && params.size() > 0)
    {
        if (params.size() != undistortion->getParameters().size())
        {
            throw std::invalid_argument("Invalid number of distortion parameters");
        }

        undistortion->setParameters(params);
    }

    return undistortion;
}

/**
 * @brief Retrieve distortion type corresponding to a given intrinsic type.
 * @param[in] intrinsicType Intrinsic type enum value.
 * @return The corresponding distortion type enum value.
 */
inline EDISTORTION getDistortionType(EINTRINSIC intrinsicType)
{
    switch (intrinsicType)
    {
        case EINTRINSIC::PINHOLE_CAMERA:
            return EDISTORTION::NONE;
        case EINTRINSIC::PINHOLE_CAMERA_RADIAL1:
            return EDISTORTION::DISTORTION_RADIALK1;
        case EINTRINSIC::PINHOLE_CAMERA_RADIAL3:
            return EDISTORTION::DISTORTION_RADIALK3;
        case EINTRINSIC::PINHOLE_CAMERA_BROWN:
            return EDISTORTION::DISTORTION_BROWN;
        case EINTRINSIC::PINHOLE_CAMERA_FISHEYE:
            return EDISTORTION::DISTORTION_FISHEYE;
        case EINTRINSIC::PINHOLE_CAMERA_FISHEYE1:
            return EDISTORTION::DISTORTION_FISHEYE1;
        case EINTRINSIC::PINHOLE_CAMERA_3DEANAMORPHIC4:
            return EDISTORTION::DISTORTION_3DEANAMORPHIC4;
        case EINTRINSIC::PINHOLE_CAMERA_3DECLASSICLD:
            return EDISTORTION::DISTORTION_3DECLASSICLD;
        case EINTRINSIC::PINHOLE_CAMERA_3DERADIAL4:
            return EDISTORTION::DISTORTION_3DERADIAL4;
        case EINTRINSIC::EQUIDISTANT_CAMERA:
            return EDISTORTION::NONE;
        case EINTRINSIC::EQUIDISTANT_CAMERA_RADIAL3:
            return EDISTORTION::DISTORTION_RADIALK3PT;
        default:
            break;
    }

    throw std::out_of_range("Invalid intrinsic type");
}

/**
 * @brief Factory function to create an intrinsic object.
 * @param[in] intrinsicType Type of intrinsic to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @return Shared pointer of initialized intrinsic object.
 */
inline std::shared_ptr<IntrinsicBase> createIntrinsic(EINTRINSIC intrinsicType,
                                                      unsigned int w = 0,
                                                      unsigned int h = 0,
                                                      double focalLengthPixX = 0.0,
                                                      double focalLengthPixY = 0.0,
                                                      double offsetX = 0.0,
                                                      double offsetY = 0.0)
{
    auto distortion = createDistortion(getDistortionType(intrinsicType));
    auto undistortion = createUndistortion(getDistortionType(intrinsicType), w, h);

    if (isPinhole(intrinsicType))
    {
        return std::make_shared<Pinhole>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion);
    }

    if (isEquidistant(intrinsicType))
    {
        return std::make_shared<Equidistant>(w, h, focalLengthPixX, offsetX, offsetY, distortion);
    }

    return nullptr;
}

/**
 * @brief Factory function to create a pinhole camera object.
 * @param[in] intrinsicType Type of pinhole camera to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized pinhole camera object.
 */
inline std::shared_ptr<Pinhole> createPinhole(EINTRINSIC intrinsicType,
                                              unsigned int w = 0,
                                              unsigned int h = 0,
                                              double focalLengthPixX = 0.0,
                                              double focalLengthPixY = 0.0,
                                              double offsetX = 0.0,
                                              double offsetY = 0.0,
                                              std::initializer_list<double> distortionParams = {})
{
    if (!isPinhole(intrinsicType))
    {
        return nullptr;
    }

    auto distortion = createDistortion(getDistortionType(intrinsicType), distortionParams);
    auto undistortion = createUndistortion(getDistortionType(intrinsicType), w, h, distortionParams);

    return std::make_shared<Pinhole>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion);
}

/**
 * @brief Factory function to create an equidistant camera object.
 * @param[in] intrinsicType Type of equidistant camera to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized equidistant camera object.
 */
inline std::shared_ptr<Equidistant> createEquidistant(EINTRINSIC intrinsicType,
                                                      unsigned int w = 0,
                                                      unsigned int h = 0,
                                                      double focalLengthPix = 0.0,
                                                      double offsetX = 0.0,
                                                      double offsetY = 0.0,
                                                      std::initializer_list<double> distortionParams = {})
{
    if (!isEquidistant(intrinsicType))
    {
        return nullptr;
    }

    auto distortion = createDistortion(getDistortionType(intrinsicType), distortionParams);

    return std::make_shared<Equidistant>(w, h, focalLengthPix, offsetX, offsetY, distortion);
}

}  // namespace camera
}  // namespace aliceVision
