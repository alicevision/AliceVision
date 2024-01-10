// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/image/Image.hpp"

namespace aliceVision {
namespace image {

/// Horizontal concatenation of images
template<class Image>
void concatH(const Image& imageA, const Image& imageB, Image& out)
{
    // Compute new dimensions // |imgA|+|imgB|
    int ww = imageA.width() + imageB.width();
    out.resize(ww, std::max(imageA.height(), imageB.height()));

    // Copy the first image |imgA|...|
    out.block(0, 0, imageA.height(), imageA.width()) = imageA.getMat();
    // Copy the second image |imgA|imgB|
    out.block(0, imageA.width(), imageB.height(), imageB.width()) = imageB.getMat();
}

/// Vertical concatenation of images
template<class Image>
void concatV(const Image& imageA, const Image& imageB, Image& out)
{
    // Compute new dimensions
    // |imgA|
    // |imgB|
    int hh = imageA.height() + imageB.height();
    out.resize(max(imageA.width(), imageB.width()), hh);

    // Copy the first image
    // |imgA|
    // |....|
    out.block(0, 0, imageA.height(), imageA.width()) = imageA.getMat();
    // Copy the second image
    // |imgA|
    // |imgB|
    out.block(imageA.height(), 0, imageB.height(), imageB.width()) = imageB.getMat();
}

}  // namespace image
}  // namespace aliceVision
