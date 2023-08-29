// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/Sampler.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
#include <aliceVision/camera/Pinhole.hpp>
#include <aliceVision/camera/Undistortion.hpp>
#include <aliceVision/image/io.hpp>

#include <memory>

namespace aliceVision {
namespace camera {

/// Undistort an image according a given camera and its distortion model
template <typename T>
void UndistortImage(
    const image::Image<T>& imageIn,
    const camera::IntrinsicBase * intrinsicSource,
    const camera::IntrinsicBase * intrinsicOutput,
    const camera::Undistortion * undistortionOutput,
    image::Image<T>& image_ud,
    T fillcolor,
    const oiio::ROI & roi = oiio::ROI())
{
    if (!intrinsicSource->hasDistortion())  // no distortion, perform a direct copy
    {
        image_ud = imageIn;
        return;
    }

    // There is distortion
    const Vec2 center(imageIn.Width() * 0.5, imageIn.Height() * 0.5);

    int widthRoi = intrinsicOutput->w();
    int heightRoi = intrinsicOutput->h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    image_ud.resize(widthRoi, heightRoi, true, fillcolor);
    const image::Sampler2d<image::SamplerLinear> sampler;

    #pragma omp parallel for
    for (int y = 0; y < heightRoi; ++y)
    {
        for (int x = 0; x < widthRoi; ++x)
        {
            const Vec2 undisto_pix(x + xOffset, y + yOffset);

            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicSource->cam2ima(
                intrinsicSource->addDistortion(
                    intrinsicOutput->ima2cam(
                        (undistortionOutput) ? undistortionOutput->inverse(undisto_pix) : undisto_pix)));

            // pick pixel if it is in the image domain
            if (imageIn.Contains(disto_pix(1), disto_pix(0)))
            {
                image_ud(y, x) = sampler(imageIn, disto_pix(1), disto_pix(0));
            }
        }
    }
}

/// Undistort an image according a given camera and its distortion model
template <typename T>
void UndistortImage(
    const image::Image<T>& imageIn,
    const camera::IntrinsicBase* intrinsicPtr,
    image::Image<T>& image_ud,
    T fillcolor,
    bool correctPrincipalPoint = false,
    const oiio::ROI & roi = oiio::ROI())
{
    if (!intrinsicPtr->hasDistortion())  // no distortion, perform a direct copy
    {
        image_ud = imageIn;
        return;
    }

    // There is distortion
    const Vec2 center(imageIn.Width() * 0.5, imageIn.Height() * 0.5);
    Vec2 ppCorrection(0.0, 0.0);

    if (correctPrincipalPoint)
    {
        if (camera::isPinhole(intrinsicPtr->getType()))
        {
            const camera::Pinhole* pinholePtr = dynamic_cast<const camera::Pinhole*>(intrinsicPtr);
            ppCorrection = pinholePtr->getPrincipalPoint() - center;
        }
    }

    int widthRoi = imageIn.Width();
    int heightRoi = imageIn.Height();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    image_ud.resize(widthRoi, heightRoi, true, fillcolor);
    const image::Sampler2d<image::SamplerLinear> sampler;

    #pragma omp parallel for
    for (int y = 0; y < heightRoi; ++y)
    {
        for (int x = 0; x < widthRoi; ++x)
        {
            const Vec2 undisto_pix(x + xOffset, y + yOffset);
            // compute coordinates with distortion
            const Vec2 disto_pix = intrinsicPtr->get_d_pixel(undisto_pix + ppCorrection);

            // pick pixel if it is in the image domain
            if (imageIn.Contains(disto_pix(1), disto_pix(0)))
            {
                image_ud(y, x) = sampler(imageIn, disto_pix(1), disto_pix(0));
            }
        }
    }
}

} // namespace camera
} // namespace aliceVision
