// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/io.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision{


void imageDiff(const image::Image<image::RGBfColor>& inImg,
               const image::Image<image::RGBfColor>& inImgDownscaled,
               image::Image<image::RGBfColor>& outImg, unsigned int downscale)
{
    outImg.resize(inImg.Width(), inImg.Height());

    for (int iy = 0; iy < inImg.Height(); iy++)
    {
        for (int ix = 0; ix < inImg.Width(); ix++)
        {
            outImg(iy, ix) = inImg(iy, ix) - getInterpolateColor(inImgDownscaled,
                                                                 iy / downscale, ix / downscale);
        }
    }
}

void laplacianPyramid(std::vector<image::Image<image::RGBfColor>>& out_pyramidL,
                      const image::Image<image::RGBfColor>& image, int nbBand, unsigned int downscale)
{
    assert(nbBand >= 1);

    image::Image<image::RGBfColor> img(image);
    int outW = static_cast<int>(img.Width()/downscale);
    int outH = static_cast<int>(img.Height()/downscale);

    image::Image<image::RGBfColor> imgDownscaled(outW, outH);
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
        ALICEVISION_LOG_DEBUG("laplacianDownscalePyramid: Size level " << i << " : "
                              << out_pyramidL[i].Width() << "x" << out_pyramidL[i].Height());
}



}
