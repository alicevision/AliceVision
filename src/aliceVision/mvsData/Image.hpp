// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/mvsData/Point2d.hpp>

namespace aliceVision {

class Image
{
private:
    std::vector<Color> _img;
    int _width = 0;
    int _height = 0;

public:
    Image()
    {
        _width = 0;
        _height = 0;
    }

    Image(int width, int height)
        : _width(width)
        , _height(height)
    {
        _img.resize(width*height);
    }

    Image(Color* data, int  width, int  height)
    {
        _width = width;
        _height = height;
        _img.resize(_width * _height);

        for(int i = 0; i < _width*_height; ++i)
            _img[i] = data[i];
    }

    Image(const Image& other)
     : _width(other._width)
     , _height(other._height)
     , _img(other._img)
    {
    }

    void resize(int width, int height)
    {
        _width = width;
        _height = height;
        _img.resize(_width*_height);
    }

    void swap(Image& other)
    {
        std::swap(_width, other._width);
        std::swap(_height, other._height);
        _img.swap(other._img);
    }

    Image& operator=(const Image& param)
    {
        _width = param._width;
        _height = param._height;
        _img = param._img;
        return *this;
    }

    //outImg = inImga - inImgb
    static void imageDiff(const Image& inImga, const Image& inImgb, Image& outImg)
    {
        if(inImga._width != inImgb._width || inImga._height != inImgb._height)
            throw std::runtime_error("Incompatible image sizes");

        outImg.resize(inImga._width, inImga._height);
        for(int i = 0; i < inImga._width*inImga._height; ++i)
        {
            outImg._img[i] = inImga._img.at(i) - inImgb._img.at(i);
        }
    }

    int width() const { return _width; }
    int height() const { return _height; }

    std::vector<Color>& data() { return _img; }
    const std::vector<Color>& data() const { return _img; }

    Color getInterpolateColor(const Point2d& pix) const
    {
        const int xp = static_cast<int>(pix.x);
        const int yp = static_cast<int>(pix.y);

        // precision to 4 decimal places
        const float ui = pix.x - static_cast<float>(xp);
        const float vi = pix.y - static_cast<float>(yp);

        const Color lu = _img.at( yp * _width + xp   );
        const Color ru = _img.at( yp * _width + (xp+1) );
        const Color rd = _img.at( (yp+1) * _width + (xp+1) );
        const Color ld = _img.at( (yp+1) * _width + xp );

        // bilinear interpolation of the pixel intensity value
        const Color u = lu + (ru - lu) * ui;
        const Color d = ld + (rd - ld) * ui;
        const Color out = u + (d - u) * vi;
        return out;
    }

};


}



