#pragma once

#include <aliceVision/image/all.hpp>

namespace aliceVision
{

class Compositer
{
public:
    Compositer(size_t outputWidth, size_t outputHeight)
        : _panorama(outputWidth, outputHeight, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f))
    {
    }

    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, size_t offset_x, size_t offset_y)
    {

        for(size_t i = 0; i < color.Height(); i++)
        {

            size_t pano_i = offset_y + i;
            if(pano_i >= _panorama.Height())
            {
                continue;
            }

            for(size_t j = 0; j < color.Width(); j++)
            {

                if(!inputMask(i, j))
                {
                    continue;
                }

                size_t pano_j = offset_x + j;
                if(pano_j >= _panorama.Width())
                {
                    pano_j = pano_j - _panorama.Width();
                }

                _panorama(pano_i, pano_j).r() = color(i, j).r();
                _panorama(pano_i, pano_j).g() = color(i, j).g();
                _panorama(pano_i, pano_j).b() = color(i, j).b();
                _panorama(pano_i, pano_j).a() = 1.0f;
            }
        }

        return true;
    }

    virtual bool terminate() { return true; }

    aliceVision::image::Image<image::RGBAfColor>& getPanorama() { return _panorama; }

protected:
    aliceVision::image::Image<image::RGBAfColor> _panorama;
};

} // namespace aliceVision