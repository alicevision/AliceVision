#pragma once

#include <aliceVision/image/all.hpp>
#include "cachedImage.hpp"

namespace aliceVision
{

template <class T>
bool downscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{

    size_t output_width = inputColor.Width() / 2;
    size_t output_height = inputColor.Height() / 2;

    for(int i = 0; i < output_height; i++)
    {
        for(int j = 0; j < output_width; j++)
        {
            outputColor(i, j) = inputColor(i * 2, j * 2);
        }
    }

    return true;
}

template <class T>
bool upscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{

    size_t width = inputColor.Width();
    size_t height = inputColor.Height();

    for(int i = 0; i < height; i++)
    {

        int di = i * 2;

        for(int j = 0; j < width; j++)
        {
            int dj = j * 2;

            outputColor(di, dj) = T();
            outputColor(di, dj + 1) = T();
            outputColor(di + 1, dj) = T();
            outputColor(di + 1, dj + 1) = inputColor(i, j);
        }
    }

    return true;
}

template <class T>
bool substract(aliceVision::image::Image<T>& AminusB, const aliceVision::image::Image<T>& A,
               const aliceVision::image::Image<T>& B)
{

    size_t width = AminusB.Width();
    size_t height = AminusB.Height();

    if(AminusB.size() != A.size())
    {
        return false;
    }

    if(AminusB.size() != B.size())
    {
        return false;
    }

    for(int i = 0; i < height; i++)
    {

        for(int j = 0; j < width; j++)
        {

            AminusB(i, j) = A(i, j) - B(i, j);
        }
    }

    return true;
}

template <class T>
bool addition(aliceVision::image::Image<T>& AplusB, const aliceVision::image::Image<T>& A,
              const aliceVision::image::Image<T>& B)
{

    size_t width = AplusB.Width();
    size_t height = AplusB.Height();

    if(AplusB.size() != A.size())
    {
        return false;
    }

    if(AplusB.size() != B.size())
    {
        return false;
    }

    for(int i = 0; i < height; i++)
    {

        for(int j = 0; j < width; j++)
        {

            AplusB(i, j) = A(i, j) + B(i, j);
        }
    }

    return true;
}

bool downscaleByPowerOfTwo(image::Image<image::RGBfColor> & output, image::Image<unsigned char> & outputMask, const image::Image<image::RGBfColor> & input, const image::Image<unsigned char> & inputMask, const int timesDividedBy2);

void removeNegativeValues(CachedImage<image::RGBfColor>& img);

} // namespace aliceVision