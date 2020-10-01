#pragma once

#include "compositer.hpp"

namespace aliceVision
{

class AlphaCompositer : public Compositer
{
public:
    AlphaCompositer(size_t outputWidth, size_t outputHeight)
        : Compositer(outputWidth, outputHeight)
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

                float wc = inputWeights(i, j);

                _panorama(pano_i, pano_j).r() += wc * color(i, j).r();
                _panorama(pano_i, pano_j).g() += wc * color(i, j).g();
                _panorama(pano_i, pano_j).b() += wc * color(i, j).b();
                _panorama(pano_i, pano_j).a() += wc;
            }
        }

        return true;
    }

    virtual bool terminate()
    {

        for(int i = 0; i < _panorama.Height(); i++)
        {
            for(int j = 0; j < _panorama.Width(); j++)
            {

                if(_panorama(i, j).a() < 1e-6)
                {
                    _panorama(i, j).r() = 1.0f;
                    _panorama(i, j).g() = 0.0f;
                    _panorama(i, j).b() = 0.0f;
                    _panorama(i, j).a() = 0.0f;
                }
                else
                {
                    _panorama(i, j).r() = _panorama(i, j).r() / _panorama(i, j).a();
                    _panorama(i, j).g() = _panorama(i, j).g() / _panorama(i, j).a();
                    _panorama(i, j).b() = _panorama(i, j).b() / _panorama(i, j).a();
                    _panorama(i, j).a() = 1.0f;
                }
            }
        }

        return true;
    }
};

} // namespace aliceVision
