#pragma once

#include <aliceVision/image/all.hpp>

#include "cachedImage.hpp"

namespace aliceVision
{

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

class Compositer
{
public:
    Compositer(image::TileCacheManager::shared_ptr & cacheManager, int width, int height) :
    _cacheManager(cacheManager),
    _panoramaWidth(width),
    _panoramaHeight(height)
    {
        
    }

    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        int offset_x, int offset_y, const BoundingBox & contentBox)
    {
        aliceVision::image::Image<image::RGBAfColor> masked(color.Width(), color.Height());

        BoundingBox panoramaBb;
        panoramaBb.left = offset_x;
        panoramaBb.top = offset_y;
        panoramaBb.width = color.Width();
        panoramaBb.height = color.Height();

        if (!loopyCachedImageExtract(masked, _panorama, panoramaBb)) 
        {
            return false;
        }
        

        for(size_t i = 0; i < color.Height(); i++)
        {
            for(size_t j = 0; j < color.Width(); j++)
            {
                if(!inputMask(i, j))
                {
                    continue;
                }

                masked(i, j).r() = color(i, j).r();
                masked(i, j).g() = color(i, j).g();
                masked(i, j).b() = color(i, j).b();
                masked(i, j).a() = 1.0f;
            }
        }

        BoundingBox inputBb;
        inputBb.left = 0;
        inputBb.top = 0;
        inputBb.width = color.Width();
        inputBb.height = color.Height();

        if (!loopyCachedImageAssign(_panorama, masked, panoramaBb, inputBb)) {
            return false;
        }

        return true;
    }

    virtual bool initialize() { 

        if(!_panorama.createImage(_cacheManager, _panoramaWidth, _panoramaHeight))
        {
            return false;
        }

        if(!_panorama.perPixelOperation(
            [](image::RGBAfColor ) -> image::RGBAfColor 
            { 
                return image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f); 
            })
          )
        {
            return false;
        }

        return true; 
    }

    virtual bool terminate() { return true; }

    bool save(const std::string &path) 
    {
        if(!_panorama.writeImage(path))
        {
            return false;
        }

        return true;
    }

protected:
    image::TileCacheManager::shared_ptr _cacheManager;
    CachedImage<image::RGBAfColor> _panorama;
    int _panoramaWidth;
    int _panoramaHeight;
};

} // namespace aliceVision
