#pragma once

#include "compositer.hpp"

namespace aliceVision
{

class AlphaCompositer : public Compositer
{
public:
    AlphaCompositer(image::TileCacheManager::shared_ptr & cacheManager, size_t outputWidth, size_t outputHeight) 
    : Compositer(cacheManager, outputWidth, outputHeight)
    {
    }

    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        int offset_x, int offset_y)
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

                float wc = inputWeights(i, j);

                masked(i, j).r() += wc * color(i, j).r();
                masked(i, j).g() += wc * color(i, j).g();
                masked(i, j).b() += wc * color(i, j).b();
                masked(i, j).a() += wc;
            }
        }

        BoundingBox inputBb;
        inputBb.left = 0;
        inputBb.top = 0;
        inputBb.width = masked.Width();
        inputBb.height = masked.Height();

        if (!loopyCachedImageAssign(_panorama, masked, panoramaBb, inputBb)) {
            return false;
        }

        return true;
    }

    virtual bool terminate()
    {
        
        _panorama.perPixelOperation(
            [](image::RGBAfColor c) -> image::RGBAfColor
            {
                image::RGBAfColor r;

                if (c.a() < 1e-6f) 
                {
                    r.r() = 1.0f;
                    r.g() = 0.0f;    
                    r.b() = 0.0f;
                    r.a() = 0.0f;
                }
                else 
                {
                    r.r() = c.r() / c.a();
                    r.g() = c.g() / c.a();
                    r.b() = c.b() / c.a();
                    r.a() = 1.0f;
                }

                return r;
            }
        );

        return true;
    }
};

} // namespace aliceVision
