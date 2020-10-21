#pragma once

#include <aliceVision/image/all.hpp>
#include "cachedImage.hpp"

namespace aliceVision
{

template <class T>
bool downscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{
    for(int i = 0; i < outputColor.Height(); i++)
    {
        int di = i * 2;

        for(int j = 0; j < outputColor.Width(); j++)
        {
            int dj = j * 2;

            outputColor(i, j) = inputColor(di, dj);
        }
    }

    return true;
}


template <class T>
bool upscale(aliceVision::image::Image<T>& outputColor, const aliceVision::image::Image<T>& inputColor)
{

    size_t width = inputColor.Width();
    size_t height = inputColor.Height();
    size_t dwidth = outputColor.Width();
    size_t dheight = outputColor.Height();

    for(int i = 0; i < height - 1; i++)
    {
        int di = i * 2;

        for(int j = 0; j < width - 1; j++)
        {
            int dj = j * 2;

            outputColor(di, dj) = T();
            outputColor(di, dj + 1) = T();
            outputColor(di + 1, dj) = T();
            outputColor(di + 1, dj + 1) = inputColor(i, j);
        }
    }

    for (int i = 0; i < height; i++)
    {
        int j = width - 1;
        int di = i * 2;
        int dj = j * 2;

        outputColor(di, dj) = T();

        if (dj < dwidth - 1)
        {
            outputColor(di, dj + 1) = T();
        }

        if (di < dheight - 1)
        {
            outputColor(di + 1, dj) = T();

            if (dj < dwidth - 1)
            {
                outputColor(di + 1, dj + 1) = inputColor(i, j);
            }
        }
    }

    for (int j = 0; j < width; j++)
    {
        int i = height - 1;
        int di = i * 2;
        int dj = j * 2;

        outputColor(di, dj) = T();

        if (dj < dwidth - 1)
        {
            outputColor(di, dj + 1) = T();
        }

        if (di < dheight - 1)
        {
            outputColor(di + 1, dj) = T();

            if (dj < dwidth - 1)
            {
                outputColor(di + 1, dj + 1) = inputColor(i, j);
            }
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
    outputBb.height = std::min(extractedInputBb.height, output.Height());

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
        int availableWidth = output.Width();
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
            
            //Update the bouding box for output
            outputBb.left += extractedWidth;
            availableWidth -= extractedWidth;
            
            //All the input is available.
            inputBb.left = 0;
            inputBb.width = input.getWidth();
        }
    }

    return true;
}


template <class T>
bool makeImagePyramidCompatible(image::Image<T>& output, 
                                size_t& out_offset_x, size_t& out_offset_y,
                                const image::Image<T>& input,
                                size_t offset_x, size_t offset_y, 
                                size_t num_levels)
{

    if(num_levels == 0)
    {
        return false;
    }

    double max_scale = 1.0 / pow(2.0, num_levels - 1);

    double low_offset_x = double(offset_x) * max_scale;
    double low_offset_y = double(offset_y) * max_scale;

    /*Make sure offset is integer even at the lowest level*/
    double corrected_low_offset_x = floor(low_offset_x);
    double corrected_low_offset_y = floor(low_offset_y);

    /*Add some borders on the top and left to make sure mask can be smoothed*/
    corrected_low_offset_x = std::max(0.0, corrected_low_offset_x - 3.0);
    corrected_low_offset_y = std::max(0.0, corrected_low_offset_y - 3.0);

    /*Compute offset at largest level*/
    out_offset_x = size_t(corrected_low_offset_x / max_scale);
    out_offset_y = size_t(corrected_low_offset_y / max_scale);

    /*Compute difference*/
    double doffset_x = double(offset_x) - double(out_offset_x);
    double doffset_y = double(offset_y) - double(out_offset_y);

    /* update size with border update */
    double large_width = double(input.Width()) + doffset_x;
    double large_height = double(input.Height()) + doffset_y;

    /* compute size at largest scale */
    double low_width = large_width * max_scale;
    double low_height = large_height * max_scale;

    /*Make sure width is integer event at the lowest level*/
    double corrected_low_width = ceil(low_width);
    double corrected_low_height = ceil(low_height);

    /*Add some borders on the right and bottom to make sure mask can be smoothed*/
    corrected_low_width = corrected_low_width + 3;
    corrected_low_height = corrected_low_height + 3;

    /*Compute size at largest level*/
    size_t width = size_t(corrected_low_width / max_scale);
    size_t height = size_t(corrected_low_height / max_scale);

    output = image::Image<T>(width, height, true, T(0.0f));
    output.block(doffset_y, doffset_x, input.Height(), input.Width()) = input;

    return true;
}


} // namespace aliceVision