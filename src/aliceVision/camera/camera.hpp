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
#include <aliceVision/camera/UndistortionRadial.hpp>
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
            return nullptr;
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
inline std::shared_ptr<Undistortion> createUndistortion(EUNDISTORTION undistortionType,
                                                        unsigned int w = 0,
                                                        unsigned int h = 0,
                                                        std::initializer_list<double> params = {})
{
    std::shared_ptr<Undistortion> undistortion = nullptr;

    switch (undistortionType)
    {
        case EUNDISTORTION::UNDISTORTION_RADIALK3:
            undistortion = std::make_shared<UndistortionRadialK3>(w, h);
            break;
        case EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4:
            undistortion = std::make_shared<Undistortion3DEAnamorphic4>(w, h);
            break;
        default:
            return nullptr;
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
 * @brief Factory function to create an intrinsic object.
 * @param[in] intrinsicType Type of intrinsic to use.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] undistortionType Type of undistortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @return Shared pointer of initialized intrinsic object.
 */
inline std::shared_ptr<IntrinsicBase> createIntrinsic(EINTRINSIC intrinsicType,
                                                      EDISTORTION distortionType,
                                                      EUNDISTORTION undistortionType,
                                                      unsigned int w = 0,
                                                      unsigned int h = 0,
                                                      double focalLengthPixX = 0.0,
                                                      double focalLengthPixY = 0.0,
                                                      double offsetX = 0.0,
                                                      double offsetY = 0.0)
{
    auto distortion = createDistortion(distortionType);
    auto undistortion = createUndistortion(undistortionType, w, h);

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
 * @param[in] distortionType Type of distortion to use.
 * @param[in] undistortionType Type of undistortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized pinhole camera object.
 */
inline std::shared_ptr<Pinhole> createPinhole(EDISTORTION distortionType,
                                              EUNDISTORTION undistortionType,
                                              unsigned int w = 0,
                                              unsigned int h = 0,
                                              double focalLengthPixX = 0.0,
                                              double focalLengthPixY = 0.0,
                                              double offsetX = 0.0,
                                              double offsetY = 0.0,
                                              std::initializer_list<double> distortionParams = {})
{
    auto distortion = createDistortion(distortionType, distortionParams);
    auto undistortion = createUndistortion(undistortionType, w, h, distortionParams);

    return std::make_shared<Pinhole>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion);
}

/**
 * @brief Factory function to create an equidistant camera object.
 * @param[in] distortionType Type of distortion to use.
 * @param[in] w Intrinsic width.
 * @param[in] h Intrinsic height.
 * @param[in] focalLengthPixX Focal length in pixels (x-axis).
 * @param[in] focalLengthPixY Focal length in pixels (y-axis).
 * @param[in] offsetX Optical offset in pixels (x-axis).
 * @param[in] offsetY Optical offset in pixels (y-axis).
 * @param[in] distortionParams Distortion parameters.
 * @return Shared pointer of initialized equidistant camera object.
 */
inline std::shared_ptr<Equidistant> createEquidistant(EDISTORTION distortionType,
                                                      unsigned int w = 0,
                                                      unsigned int h = 0,
                                                      double focalLengthPix = 0.0,
                                                      double offsetX = 0.0,
                                                      double offsetY = 0.0,
                                                      std::initializer_list<double> distortionParams = {})
{
    auto distortion = createDistortion(distortionType, distortionParams);

    return std::make_shared<Equidistant>(w, h, focalLengthPix, offsetX, offsetY, distortion);
}

inline EDISTORTION getDistortionType(const IntrinsicBase & intrinsic)
{
    camera::EDISTORTION distoType = camera::EDISTORTION::DISTORTION_NONE;

    try
    {
        const auto & isod = dynamic_cast<const camera::IntrinsicScaleOffsetDisto &>(intrinsic);
        auto disto = isod.getDistortion();
        if (disto)
        {
            distoType = disto->getType();
        }
    }
    catch(const std::bad_cast & e)
    {
    }
    
    return distoType;
}

}  // namespace camera
}  // namespace aliceVision
