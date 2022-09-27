// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "SgmParams.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

namespace aliceVision {
namespace depthMap {

int computeDownscale(const mvsUtils::MultiViewParams& mp, int scale, int maxWidth, int maxHeight)
{
    const int maxImageWidth = mp.getMaxImageWidth() / scale;
    const int maxImageHeight = mp.getMaxImageHeight() / scale;

    int downscale = 1;
    int downscaleWidth = mp.getMaxImageWidth() / scale;
    int downscaleHeight = mp.getMaxImageHeight() / scale;

    while((downscaleWidth > maxWidth) || (downscaleHeight > maxHeight))
    {
        downscale++;
        downscaleWidth = maxImageWidth / downscale;
        downscaleHeight = maxImageHeight / downscale;
    }

    return downscale;
}

void computeScaleStepSgmParams(const mvsUtils::MultiViewParams& mp, SgmParams& sgmParams) 
{
    const int fileScale = 1; // input images scale (should be one)
    const int maxSideXY = sgmParams.maxSideXY / mp.getProcessDownscale();
    const int maxImageW = mp.getMaxImageWidth();
    const int maxImageH = mp.getMaxImageHeight();

    int maxW = maxSideXY;
    int maxH = maxSideXY * 0.8;
    
    if(maxImageW < maxImageH)
        std::swap(maxW, maxH);

    if(sgmParams.scale == -1)
    {
        // compute the number of scales that will be used in the plane sweeping.
        // the highest scale should have a resolution close to 700x550 (or less).
        const int scaleTmp = computeDownscale(mp, fileScale, maxW, maxH);
        sgmParams.scale = std::min(2, scaleTmp);
    }
    if(sgmParams.stepXY == -1)
    {
        sgmParams.stepXY = computeDownscale(mp, fileScale * sgmParams.scale, maxW, maxH);
    }

    ALICEVISION_LOG_INFO("Computed SGM scale: " << sgmParams.scale << ", stepXY: " << sgmParams.stepXY);
}

} // namespace depthMap
} // namespace aliceVision
