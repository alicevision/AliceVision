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

    virtual bool append(aliceVision::image::Image<image::RGBfColor>& color,
                        aliceVision::image::Image<unsigned char>& inputMask,
                        aliceVision::image::Image<float>& inputWeights, 
                        int offsetX, int offsetY)
    {
       offsetX -= _outputRoi.left;
       offsetY -= _outputRoi.top;

       for(int i = 0; i < color.Height(); i++)
        {
            int y = i + offsetY;
            if (y < 0 || y >= _outputRoi.height) continue;

            for (int j = 0; j < color.Width(); j++)
            {
                int x = j + offsetX;
                if (x < 0 || x >= _outputRoi.width) continue;

                if (!inputMask(i, j))
                {
                    continue;
                }

                float wc = inputWeights(i, j);

                _panorama(y, x).r() += wc * color(i, j).r();
                _panorama(y, x).g() += wc * color(i, j).g();
                _panorama(y, x).b() += wc * color(i, j).b();
                _panorama(y, x).a() += wc;
            }
        }

        return true;
    }

    virtual bool terminate()
    {
        for (int i = 0; i < _panorama.Height(); i++) 
        {
            for (int j = 0; j < _panorama.Width(); j++)
            {
                image::RGBAfColor r;
                image::RGBAfColor c = _panorama(i, j);

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

                _panorama(i, j) = r;
            }
        }

        return true;
    }
};

} // namespace aliceVision
