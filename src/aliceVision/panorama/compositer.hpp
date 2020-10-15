#pragma once

#include <aliceVision/image/all.hpp>

#include "cachedImage.hpp"
#include "imageOps.hpp"

namespace aliceVision
{

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
