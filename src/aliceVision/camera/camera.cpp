// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/camera.hpp>

#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>
#include <aliceVision/camera/Undistortion3DEA4.hpp>
#include <aliceVision/camera/Undistortion3DEClassicLD.hpp>
#include <aliceVision/camera/Undistortion3DERadial4.hpp>
#include <aliceVision/camera/UndistortionRadial.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Equidistant.hpp>
#include <aliceVision/camera/Equirectangular.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>


namespace aliceVision {
namespace camera {


std::shared_ptr<Distortion> createDistortion(EDISTORTION distortionType, std::initializer_list<double> params)
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


std::shared_ptr<Undistortion> createUndistortion(EUNDISTORTION undistortionType,
                                                unsigned int w,
                                                unsigned int h,
                                                std::initializer_list<double> params)
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
        case EUNDISTORTION::UNDISTORTION_3DECLASSICLD:
            undistortion = std::make_shared<Undistortion3DEClassicLD>(w, h);
            break;
        case EUNDISTORTION::UNDISTORTION_3DERADIAL4:
            undistortion = std::make_shared<Undistortion3DERadial4>(w, h);
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

std::shared_ptr<IntrinsicBase> createIntrinsic(EINTRINSIC intrinsicType,
                                            EDISTORTION distortionType,
                                            EUNDISTORTION undistortionType,
                                            unsigned int w,
                                            unsigned int h,
                                            double focalLengthPixX,
                                            double focalLengthPixY,
                                            double offsetX,
                                            double offsetY)
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

    if (isEquirectangular(intrinsicType))
    {
        return std::make_shared<Equirectangular>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY);
    }

    return nullptr;
}

std::shared_ptr<Pinhole> createPinhole(EDISTORTION distortionType,
                                    EUNDISTORTION undistortionType,
                                    unsigned int w,
                                    unsigned int h,
                                    double focalLengthPixX,
                                    double focalLengthPixY,
                                    double offsetX,
                                    double offsetY,
                                    std::initializer_list<double> distortionParams)
{
    auto distortion = createDistortion(distortionType, distortionParams);
    auto undistortion = createUndistortion(undistortionType, w, h, distortionParams);

    return std::make_shared<Pinhole>(w, h, focalLengthPixX, focalLengthPixY, offsetX, offsetY, distortion, undistortion);
}


std::shared_ptr<Equidistant> createEquidistant(EDISTORTION distortionType,
                                            unsigned int w,
                                            unsigned int h,
                                            double focalLengthPix,
                                            double offsetX,
                                            double offsetY,
                                            std::initializer_list<double> distortionParams)
{
    auto distortion = createDistortion(distortionType, distortionParams);

    return std::make_shared<Equidistant>(w, h, focalLengthPix, offsetX, offsetY, distortion);
}

EDISTORTION getDistortionType(const IntrinsicBase& intrinsic)
{
    camera::EDISTORTION distoType = camera::EDISTORTION::DISTORTION_NONE;

    try
    {
        const auto& isod = dynamic_cast<const camera::IntrinsicScaleOffsetDisto&>(intrinsic);
        auto disto = isod.getDistortion();
        if (disto)
        {
            distoType = disto->getType();
        }
    }
    catch (const std::bad_cast& e)
    {}

    return distoType;
}

}  // namespace camera
}  // namespace aliceVision
