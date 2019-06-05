// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Image.hpp"

#include <aliceVision/mvsData/imageIO.hpp>


namespace aliceVision{


void Image::imageDiff(const Image& inImgDownscaled, Image& outImg, unsigned int downscale) const
{
    outImg.resize(_width, _height);
    for(int i = 0; i < _width*_height; ++i)
    {
        Point2d pix(i%_width, static_cast<int>(i/_width));
        Point2d pixd = pix/downscale;

        outImg._data[i] = _data[i] - inImgDownscaled.getInterpolateColor(pixd);
    }

}

void Image::laplacianPyramid(std::vector<Image>& out_pyramidL, int nbBand, unsigned int downscale) const
{
    assert(nbBand >= 1);

    Image img(*this);
    int outW = static_cast<int>(img.width()/downscale);
    int outH = static_cast<int>(img.height()/downscale);

    Image imgDownscaled(outW, outH);
    out_pyramidL.resize(nbBand);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageIO::resizeImage(static_cast<int>(downscale), img, imgDownscaled, "gaussian");
        img.imageDiff(imgDownscaled, out_pyramidL[b], downscale);
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

Color Image::getInterpolateColor(const Point2d& pix) const
{
    const int xp = std::min(static_cast<int>(pix.x), _width-2);
    const int yp = std::min(static_cast<int>(pix.y), _height-2);

    // precision to 4 decimal places
    const float ui = pix.x - static_cast<float>(xp);
    const float vi = pix.y - static_cast<float>(yp);

    const Color lu = _data.at( yp * _width + xp   );
    const Color ru = _data.at( yp * _width + (xp+1) );
    const Color rd = _data.at( (yp+1) * _width + (xp+1) );
    const Color ld = _data.at( (yp+1) * _width + xp );

    // bilinear interpolation of the pixel intensity value
    const Color u = lu + (ru - lu) * ui;
    const Color d = ld + (rd - ld) * ui;
    const Color out = u + (d - u) * vi;
    return out;
}

Color Image::getNearestPixelColor(const Point2d& pix) const
{
    const int xp = std::min(static_cast<int>(pix.x), _width-1);
    const int yp = std::min(static_cast<int>(pix.y), _height-1);
    const Color lu = _data.at( yp * _width + xp   );
    return lu;
}


}
