#pragma once

#include <aliceVision/image/all.hpp>

#include "cachedImage.hpp"
#include "imageOps.hpp"
#include "seams.hpp"

namespace aliceVision
{

class Compositer
{
public:
    Compositer(int width, int height) :
    _panoramaWidth(width),
    _panoramaHeight(height)
    {
        
    }

    virtual ~Compositer()
    {
        
    }

    virtual bool append(aliceVision::image::Image<image::RGBfColor>& color,
                        aliceVision::image::Image<unsigned char>& inputMask,
                        aliceVision::image::Image<float>& inputWeights, 
                        int offset_x, int offset_y)
    {
        for(int i = 0; i < color.Height(); i++)
        {
            int y = i + offset_y;
            if (y < 0 || y >= _panoramaHeight) continue;

            for (int j = 0; j < color.Width(); j++)
            {
                int x = j + offset_x;
                if (x < 0 || x >= _panoramaWidth) continue;

                if (!inputMask(i, j))
                {
                    continue;
                }

                _panorama(y, x).r() = color(i, j).r();
                _panorama(y, x).g() = color(i, j).g();
                _panorama(y, x).b() = color(i, j).b();
                _panorama(y, x).a() = 1.0f;
            }
        }

        return true;
    }

    virtual bool initialize() { 

        _panorama = image::Image<image::RGBAfColor>(_panoramaWidth, _panoramaHeight, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

        return true; 
    }

    virtual bool terminate() { return true; }

    
    image::Image<image::RGBAfColor> &  getOutput() 
    {
        return _panorama;
    }

    virtual int getBorderSize() const 
    {
        return 0;
    }

protected:
    image::Image<image::RGBAfColor> _panorama;
    int _panoramaWidth;
    int _panoramaHeight;
};

} // namespace aliceVision
