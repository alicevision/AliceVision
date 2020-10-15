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

template <class T>
bool loopyCachedImageAssign(CachedImage<T> & output, const aliceVision::image::Image<T> & input, const BoundingBox & assignedOutputBb, const BoundingBox & assignedInputBb) 
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

    if (outputBb.getBottom() >= output.getHeight())
    {
        outputBb.height =  output.getHeight() - outputBb.top;
        inputBb.height = outputBb.height;
    }

    if (assignedOutputBb.getRight() < output.getWidth()) {
        
        if (!output.assign(input, inputBb, outputBb)) 
        {
            return false;
        }
    }
    else {

        int left_1 = assignedOutputBb.left;
        int left_2 = 0;
        int width1 = output.getWidth() - assignedOutputBb.left;
        int width2 = input.Width() - width1;

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

        //no overlap
        int width2_clamped = std::min(width2, left_1);
        inputBb.width = width2_clamped;
        outputBb.width = width2_clamped;
        if (width2_clamped == 0) return true;


        if (!output.assign(input, inputBb, outputBb))
        {
            return false;
        }
    }

    return true;
}

template <class T>
bool loopyCachedImageExtract(aliceVision::image::Image<T> & output, CachedImage<T> & input, const BoundingBox & extractedInputBb) 
{   
    BoundingBox outputBb;
    BoundingBox inputBb;

    outputBb.left = 0;
    outputBb.top = 0;
    outputBb.width = output.Width();
    outputBb.height = output.Height();
    
    inputBb = extractedInputBb;
    if (inputBb.getBottom() >= input.getHeight())
    {
        inputBb.height =  input.getHeight() - inputBb.top;
        outputBb.height = inputBb.height;
    }

    if (extractedInputBb.getRight() < input.getWidth()) 
    {
        if (!input.extract(output, outputBb, inputBb)) 
        {
            return false;
        }
    }
    else 
    {
        int left_1 = extractedInputBb.left;
        int left_2 = 0;
        int width_1 = input.getWidth() - extractedInputBb.left;
        int width_2 = output.Width() - width_1;

        outputBb.left = 0;
        inputBb.left = left_1;
        outputBb.width = width_1;
        inputBb.width = width_1;

        if (!input.extract(output, outputBb, inputBb))
        {
            return false;
        }

        outputBb.left = width_1;
        inputBb.left = 0;
        outputBb.width = width_2;
        inputBb.width = width_2;

        if (!input.extract(output, outputBb, inputBb))
        {
            return false;
        }
    }

    return true;
}

} // namespace aliceVision