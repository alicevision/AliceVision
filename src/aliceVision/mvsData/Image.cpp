// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Image.hpp"

#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>


namespace aliceVision{


void imageDiff(const ImageRGBf& inImg, const ImageRGBf& inImgDownscaled, ImageRGBf& outImg, unsigned int downscale)
{
    outImg.resize(inImg.width(), inImg.height());
    for(int i = 0; i < inImg.width()*inImg.height(); ++i)
    {
        Point2d pix(i%inImg.width(), static_cast<int>(i/ inImg.width()));
        Point2d pixd = pix/downscale;

        outImg[i] = inImg[i] - inImgDownscaled.getInterpolateColor(pixd);
    }
}

void laplacianPyramid(std::vector<ImageRGBf>& out_pyramidL, const ImageRGBf& image, int nbBand, unsigned int downscale)
{
    assert(nbBand >= 1);

    ImageRGBf img(image);
    int outW = static_cast<int>(img.width()/downscale);
    int outH = static_cast<int>(img.height()/downscale);

    ImageRGBf imgDownscaled(outW, outH);
    out_pyramidL.resize(nbBand);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageAlgo::resizeImage(static_cast<int>(downscale), img, imgDownscaled, "gaussian");
        imageDiff(img, imgDownscaled, out_pyramidL[b], downscale);
        img.swap(imgDownscaled);
/*
        outW = static_cast<int>(outW/downscale);
        outH = static_cast<int>(outH/downscale);
        imgDownscaled.resize(outW, outH);
*/
    }
    out_pyramidL[nbBand-1] = img;

    for(std::size_t i = 0; i < out_pyramidL.size(); ++i)
        ALICEVISION_LOG_DEBUG("laplacianDownscalePyramid: Size level " << i << " : " << out_pyramidL[i].width() << "x" << out_pyramidL[i].height());
}



}
