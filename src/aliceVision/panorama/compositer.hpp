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

                _panorama(y, x).r() = color(i, j).r();
                _panorama(y, x).g() = color(i, j).g();
                _panorama(y, x).b() = color(i, j).b();
                _panorama(y, x).a() = 1.0f;
            }
        }

        return true;
    }

    virtual bool initialize(const BoundingBox & outputRoi) { 

        _outputRoi = outputRoi;
        
        if (_outputRoi.left < 0) return false;
        if (_outputRoi.top < 0) return false;
        if (_outputRoi.getRight() >= _panoramaWidth) return false;
        if (_outputRoi.getBottom() >= _panoramaHeight) return false;

        _panorama = image::Image<image::RGBAfColor>(_outputRoi.width, _outputRoi.height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

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
    BoundingBox _outputRoi;
};

} // namespace aliceVision
