// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Point2d.hpp>

#include <algorithm>

namespace aliceVision {

template<typename ColorT>
class Image
{
public:
    using Color = ColorT;
private:
    std::vector<Color> _data;
    int _width{0};
    int _height{0};

public:
    Image() = default;

    Image(int width, int height)
        : _width(width)
        , _height(height)
    {
        _data.resize(width*height);
    }

    Image(Color* data, int  width, int  height) : Image(width, height)
    {
        for(int i = 0; i < _width*_height; ++i)
            _data[i] = data[i];
    }

    Image(const Image& other)
     : _width(other._width)
     , _height(other._height)
     , _data(other._data)
    {
    }

    int size() { return _width * _height; }

    void resize(int width, int height)
    {
        _width = width;
        _height = height;
        if(_width*_height != _data.size())
        {
            _data.resize(0); // no need to copy image content
            _data.resize(_width*_height);
        }
    }

    void swap(Image& other)
    {
        std::swap(_width, other._width);
        std::swap(_height, other._height);
        _data.swap(other._data);
    }

    Image& operator=(const Image& param)
    {
        _width = param._width;
        _height = param._height;
        _data.resize(_width*_height);
        _data = param._data;
        return *this;
    }


    int width() const { return _width; }
    int height() const { return _height; }

    void setWidth(int width) { _width = width; }
    void setHeight(int height) { _height = height; }

    std::vector<Color>& data() { return _data; }
    const std::vector<Color>& data() const { return _data; }

    const Color& operator[](std::size_t index) const { return _data[index]; }
    Color& operator[](std::size_t index) { return _data[index]; }

    const Color& at(int x, int y) const { return _data[y * _width + x]; }
    Color& at(int x, int y) { return _data[y * _width + x]; }

    Color getInterpolateColor(const Point2d& pix) const
    {
        const int xp = std::min(static_cast<int>(pix.x), _width - 2);
        const int yp = std::min(static_cast<int>(pix.y), _height - 2);

        // precision to 4 decimal places
        const float ui = pix.x - static_cast<float>(xp);
        const float vi = pix.y - static_cast<float>(yp);

        const Color lu = _data.at(yp * _width + xp);
        const Color ru = _data.at(yp * _width + (xp + 1));
        const Color rd = _data.at((yp + 1) * _width + (xp + 1));
        const Color ld = _data.at((yp + 1) * _width + xp);

        // bilinear interpolation of the pixel intensity value
        const Color u = lu + (ru - lu) * ui;
        const Color d = ld + (rd - ld) * ui;
        const Color out = u + (d - u) * vi;
        return out;
    }

    Color getNearestPixelColor(const Point2d& pix) const
    {
        const int xp = std::min(static_cast<int>(pix.x), _width - 1);
        const int yp = std::min(static_cast<int>(pix.y), _height - 1);
        const Color lu = _data.at(yp * _width + xp);
        return lu;
    }
};

using ImageRGBf = Image<ColorRGBf>;
using ImageRGBAf = Image<ColorRGBAf>;


/**
* @brief Calculate the difference between images of different sizes
* @param [inImgDownscaled] the smaller image
* @param [outImg] the difference
* @param [downscale] the downscale coefficient between image sizes
*/
void imageDiff(const ImageRGBf& inImg, const ImageRGBf& inImgDownscaled, ImageRGBf& outImg, unsigned int downscale);

/**
* @brief Calculate the laplacian pyramid of a given image,
*        ie. its decomposition in frequency bands
* @param [out_pyramidL] the laplacian pyramid
* @param [nbBand] the number of frequency bands
* @param [downscale] the downscale coefficient between floors of the pyramid
*/
void laplacianPyramid(std::vector<ImageRGBf>& out_pyramidL, const ImageRGBf& image, int nbBand, unsigned int downscale);

}

