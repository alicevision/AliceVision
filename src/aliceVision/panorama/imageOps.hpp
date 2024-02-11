// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "cachedImage.hpp"
#include <aliceVision/image/all.hpp>
#include <aliceVision/half.hpp>

namespace aliceVision {

template<class T>
bool downscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{
    for (int i = 0; i < outputColor.height(); i++)
    {
        int di = i * 2;

        for (int j = 0; j < outputColor.width(); j++)
        {
            int dj = j * 2;

            outputColor(i, j) = inputColor(di, dj);
        }
    }

    return true;
}

template<class T>
bool upscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{
    size_t width = inputColor.width();
    size_t height = inputColor.height();
    size_t dwidth = outputColor.width();
    size_t dheight = outputColor.height();

    for (int i = 0; i < height - 1; i++)
    {
        int di = i * 2;

        for (int j = 0; j < width - 1; j++)
        {
            int dj = j * 2;

            outputColor(di, dj) = inputColor(i, j);
            outputColor(di, dj + 1) = T();
            outputColor(di + 1, dj) = T();
            outputColor(di + 1, dj + 1) = T();
        }
    }

    for (int i = 0; i < height; i++)
    {
        int j = width - 1;
        int di = i * 2;
        int dj = j * 2;

        outputColor(di, dj) = inputColor(i, j);

        if (dj < dwidth - 1)
        {
            outputColor(di, dj + 1) = T();
        }

        if (di < dheight - 1)
        {
            outputColor(di + 1, dj) = T();

            if (dj < dwidth - 1)
            {
                outputColor(di + 1, dj + 1) = T();
            }
        }
    }

    for (int j = 0; j < width; j++)
    {
        int i = height - 1;
        int di = i * 2;
        int dj = j * 2;

        outputColor(di, dj) = inputColor(i, j);

        if (dj < dwidth - 1)
        {
            outputColor(di, dj + 1) = T();
        }

        if (di < dheight - 1)
        {
            outputColor(di + 1, dj) = T();

            if (dj < dwidth - 1)
            {
                outputColor(di + 1, dj + 1) = T();
            }
        }
    }

    return true;
}

template<class T>
bool substract(aliceVision::image::Image<T>& AminusB, const aliceVision::image::Image<T>& A, const aliceVision::image::Image<T>& B)
{
    size_t width = AminusB.width();
    size_t height = AminusB.height();

    if (AminusB.size() != A.size())
    {
        return false;
    }

    if (AminusB.size() != B.size())
    {
        return false;
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            AminusB(i, j) = A(i, j) - B(i, j);
        }
    }

    return true;
}

template<class T>
bool addition(aliceVision::image::Image<T>& AplusB, const aliceVision::image::Image<T>& A, const aliceVision::image::Image<T>& B)
{
    size_t width = AplusB.width();
    size_t height = AplusB.height();

    if (AplusB.size() != A.size())
    {
        return false;
    }

    if (AplusB.size() != B.size())
    {
        return false;
    }

    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            AplusB(i, j) = A(i, j) + B(i, j);
        }
    }

    return true;
}

void removeNegativeValues(image::Image<image::RGBfColor>& img);

template<class T>
bool loopyImageAssign(image::Image<T>& output,
                      const aliceVision::image::Image<T>& input,
                      const BoundingBox& assignedOutputBb,
                      const BoundingBox& assignedInputBb)
{
    BoundingBox inputBb = assignedInputBb;
    BoundingBox outputBb = assignedOutputBb;

    if (inputBb.width != outputBb.width)
    {
        return false;
    }

    if (inputBb.height != outputBb.height)
    {
        return false;
    }

    if (assignedOutputBb.getRight() < output.width())
    {
        output.block(outputBb.top, outputBb.left, outputBb.height, outputBb.width) =
          input.block(inputBb.top, inputBb.left, inputBb.height, inputBb.width);
    }
    else
    {
        int left_1 = assignedOutputBb.left;
        int left_2 = 0;
        int width1 = output.width() - assignedOutputBb.left;
        int width2 = input.width() - width1;

        inputBb.left = 0;
        outputBb.left = left_1;
        inputBb.width = width1;
        outputBb.width = width1;

        output.block(outputBb.top, outputBb.left, outputBb.height, outputBb.width) =
          input.block(inputBb.top, inputBb.left, inputBb.height, inputBb.width);

        inputBb.left = width1;
        outputBb.left = 0;

        // no overlap
        int width2_clamped = std::min(width2, left_1);
        inputBb.width = width2_clamped;
        outputBb.width = width2_clamped;
        if (width2_clamped == 0)
            return true;

        output.block(outputBb.top, outputBb.left, outputBb.height, outputBb.width) =
          input.block(inputBb.top, inputBb.left, inputBb.height, inputBb.width);
    }

    return true;
}

template<class T>
bool loopyCachedImageAssign(CachedImage<T>& output,
                            const aliceVision::image::Image<T>& input,
                            const BoundingBox& assignedOutputBb,
                            const BoundingBox& assignedInputBb)
{
    BoundingBox inputBb = assignedInputBb;
    BoundingBox outputBb = assignedOutputBb;

    if (inputBb.width != outputBb.width)
    {
        return false;
    }

    if (inputBb.height != outputBb.height)
    {
        return false;
    }

    if (assignedOutputBb.getRight() < output.getWidth())
    {
        if (!output.assign(input, inputBb, outputBb))
        {
            return false;
        }
    }
    else
    {
        int left_1 = assignedOutputBb.left;
        int left_2 = 0;
        int width1 = output.getWidth() - assignedOutputBb.left;
        int width2 = input.width() - width1;

        inputBb.left = 0;
        outputBb.left = left_1;
        inputBb.width = width1;
        outputBb.width = width1;

        if (!output.assign(input, inputBb, outputBb))
        {
            return false;
        }

        inputBb.left = width1;
        outputBb.left = 0;

        // no overlap
        int width2_clamped = std::min(width2, left_1);
        inputBb.width = width2_clamped;
        outputBb.width = width2_clamped;
        if (width2_clamped == 0)
            return true;

        if (!output.assign(input, inputBb, outputBb))
        {
            return false;
        }
    }

    return true;
}

template<class T>
bool loopyImageExtract(image::Image<T>& output, const image::Image<T>& input, const BoundingBox& extractedInputBb)
{
    BoundingBox outputBb;
    BoundingBox inputBb;

    outputBb.left = 0;
    outputBb.top = 0;
    outputBb.width = output.width();
    outputBb.height = std::min(extractedInputBb.height, output.height());

    inputBb = extractedInputBb;

    if (inputBb.getRight() < input.width())
    {
        output.block(outputBb.top, outputBb.left, outputBb.height, outputBb.width) =
          input.block(inputBb.top, inputBb.left, inputBb.height, inputBb.width);
    }
    else
    {
        int availableWidth = output.width();
        while (availableWidth > 0)
        {
            inputBb.clampRight(input.width() - 1);
            int extractedWidth = std::min(inputBb.width, availableWidth);

            inputBb.width = extractedWidth;
            outputBb.width = extractedWidth;

            output.block(outputBb.top, outputBb.left, outputBb.height, outputBb.width) =
              input.block(inputBb.top, inputBb.left, inputBb.height, inputBb.width);

            // Update the bouding box for output
            outputBb.left += extractedWidth;
            availableWidth -= extractedWidth;

            // All the input is available.
            inputBb.left = 0;
            inputBb.width = input.width();
        }
    }

    return true;
}

template<class T>
bool loopyCachedImageExtract(aliceVision::image::Image<T>& output, CachedImage<T>& input, const BoundingBox& extractedInputBb)
{
    BoundingBox outputBb;
    BoundingBox inputBb;

    outputBb.left = 0;
    outputBb.top = 0;
    outputBb.width = output.width();
    outputBb.height = std::min(extractedInputBb.height, output.height());

    inputBb = extractedInputBb;

    if (inputBb.getRight() < input.getWidth())
    {
        if (!input.extract(output, outputBb, inputBb))
        {
            return false;
        }
    }
    else
    {
        int availableWidth = output.width();
        while (availableWidth > 0)
        {
            inputBb.clampRight(input.getWidth() - 1);
            int extractedWidth = std::min(inputBb.width, availableWidth);

            inputBb.width = extractedWidth;
            outputBb.width = extractedWidth;

            if (!input.extract(output, outputBb, inputBb))
            {
                return false;
            }

            // Update the bouding box for output
            outputBb.left += extractedWidth;
            availableWidth -= extractedWidth;

            // All the input is available.
            inputBb.left = 0;
            inputBb.width = input.getWidth();
        }
    }

    return true;
}

template<class T>
bool makeImagePyramidCompatible(image::Image<T>& output,
                                int& outOffsetX,
                                int& outOffsetY,
                                const image::Image<T>& input,
                                int offsetX,
                                int offsetY,
                                size_t borderSize,
                                size_t num_levels)
{
    if (num_levels == 0)
    {
        return false;
    }

    double maxScale = 1.0 / pow(2.0, num_levels - 1);

    double lowOffsetX = double(offsetX) * maxScale;
    double lowOffsetY = double(offsetY) * maxScale;

    /*Make sure offset is integer even at the lowest level*/
    double correctedLowOffsetX = floor(lowOffsetX);
    double correctedLowOffsetY = floor(lowOffsetY);

    /*Add some borders on the top and left to make sure mask can be smoothed*/
    correctedLowOffsetX = correctedLowOffsetX - double(borderSize);
    correctedLowOffsetY = correctedLowOffsetY - double(borderSize);

    /*Compute offset at largest level*/
    outOffsetX = int(correctedLowOffsetX / maxScale);
    outOffsetY = int(correctedLowOffsetY / maxScale);

    /*Compute difference*/
    double doffsetX = double(offsetX) - double(outOffsetX);
    double doffsetY = double(offsetY) - double(outOffsetY);

    /* update size with border update */
    double large_width = double(input.width()) + doffsetX;
    double large_height = double(input.height()) + doffsetY;

    /* compute size at largest scale */
    double low_width = large_width * maxScale;
    double low_height = large_height * maxScale;

    /*Make sure width is integer even at the lowest level*/
    double correctedLowWidth = ceil(low_width);
    double correctedLowHeight = ceil(low_height);

    /*Add some borders on the right and bottom to make sure mask can be smoothed*/
    correctedLowWidth = correctedLowWidth + double(borderSize);
    correctedLowHeight = correctedLowHeight + double(borderSize);

    /*Compute size at largest level*/
    size_t width = size_t(correctedLowWidth / maxScale);
    size_t height = size_t(correctedLowHeight / maxScale);

    output = image::Image<T>(width, height, true, T(0.0f));
    output.block(doffsetY, doffsetX, input.height(), input.width()) = input;

    return true;
}

}  // namespace aliceVision
