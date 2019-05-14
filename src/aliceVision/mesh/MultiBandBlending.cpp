// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiBandBlending.hpp"

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/imageIO/image.hpp>



namespace aliceVision {
namespace mesh {

void MultiBandBlending::laplacianPyramid(std::vector<Image>& out_pyramidL, const Image& inImg, int camId, int nbBand, float sizeKernel)
{
    assert(nbBand >= 1);

    const int width = inImg.width();
    const int height = inImg.height();

    Image base(inImg); // gaussian
    Image baseG(width, height); // gaussian
    out_pyramidL.resize(nbBand); // laplacian

    float s = sizeKernel / std::pow(2, nbBand-1);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageIO::convolveImage(width, height, base.data(), baseG.data(), "gaussian", s, s); //baseG = base * gaussianKernel
        Image::imageDiff(base, baseG, out_pyramidL[b]);
        base.swap(baseG); //newBase = oldBaseG
        s *= 2;
    }
    out_pyramidL.at(nbBand-1) = base;
}

void MultiBandBlending::laplacianDownscalePyramid(std::vector<Image>& out_pyramidL, const Image& inImg, int nbBand, unsigned int downscale)
{
    assert(nbBand >= 1);

    Image img(inImg);
    int outW = static_cast<int>(img.width()/downscale);
    int outH = static_cast<int>(img.height()/downscale);

    Image imgDownscaled(outW, outH);
    out_pyramidL.resize(nbBand);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageIO::resizeImage(img.width(), img.height(), static_cast<int>(downscale), img.data(), imgDownscaled.data(), "gaussian");
        Image::imageDownscaleDiff(img, imgDownscaled, out_pyramidL[b], downscale);
        img.swap(imgDownscaled);

        outW = static_cast<int>(outW/downscale);
        outH = static_cast<int>(outH/downscale);
        imgDownscaled.resize(outW, outH);

    }
    out_pyramidL[nbBand-1] = img;

    for(std::size_t i = 0; i < out_pyramidL.size(); ++i)
        ALICEVISION_LOG_DEBUG("laplacianDownscalePyramid: Size level " << i << " : " << out_pyramidL[i].width() << "x" << out_pyramidL[i].height());
}

}
}
