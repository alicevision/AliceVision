#pragma once

#include "compositer.hpp"
#include "feathering.hpp"
#include "laplacianPyramid.hpp"

namespace aliceVision
{

class LaplacianCompositer : public Compositer
{
public:
    LaplacianCompositer(size_t outputWidth, size_t outputHeight, size_t scale)
        : Compositer(outputWidth, outputHeight)
        , _pyramidPanorama(outputWidth, outputHeight, scale + 1)
        , _bands(scale + 1)
    {
    }

    virtual ~LaplacianCompositer()
    {
        
    }

    virtual bool initialize(const BoundingBox & outputRoi) 
    { 
        _outputRoi = outputRoi;
        
        
        if (_outputRoi.left < 0) return false;
        if (_outputRoi.top < 0) return false;
        if (_outputRoi.getRight() >= _panoramaWidth) return false;
        if (_outputRoi.getBottom() >= _panoramaHeight) return false;

        return _pyramidPanorama.initialize();
    }

    virtual int getBorderSize() const 
    {
        return _gaussianFilterRadius;
    }
    
    virtual bool append(aliceVision::image::Image<image::RGBfColor>& color,
                        aliceVision::image::Image<unsigned char>& inputMask,
                        aliceVision::image::Image<float>& inputWeights, 
                        int offsetX, int offsetY) 
    {
        // Fill Color images masked parts with fake but coherent info
        aliceVision::image::Image<image::RGBfColor> feathered;
        if (!feathering(feathered, color, inputMask)) 
        {
            return false;
        }

        color = aliceVision::image::Image<image::RGBfColor>();

        //  To log space for hdr
        for(int i = 0; i < feathered.Height(); i++)
        {
            for(int j = 0; j < feathered.Width(); j++)
            {
                feathered(i, j).r() = std::log(std::max(1e-8f, feathered(i, j).r()));
                feathered(i, j).g() = std::log(std::max(1e-8f, feathered(i, j).g()));
                feathered(i, j).b() = std::log(std::max(1e-8f, feathered(i, j).b()));
            }
        }

        // Convert mask to alpha layer 
        image::Image<float> maskFloat(inputMask.Width(), inputMask.Height());
        for(int i = 0; i < inputMask.Height(); i++)
        {
            for(int j = 0; j < inputMask.Width(); j++)
            {
                if(inputMask(i, j))
                {
                    maskFloat(i, j) = 1.0f;
                }
                else
                {
                    maskFloat(i, j) = 0.0f;
                }
            }
        }


        BoundingBox bb;
        bb.left = offsetX;
        bb.top = offsetY;
        bb.width = feathered.Width();
        bb.height = feathered.Height();

        int scale = _bands - 1;
        BoundingBox potbb = bb.divide(scale).dilate(getBorderSize()).multiply(scale);

        BoundingBox contentbb = bb;
        contentbb.left = bb.left - potbb.left;
        contentbb.top = bb.top - potbb.top;

        if (!_pyramidPanorama.apply(feathered, maskFloat, inputWeights, potbb, contentbb)) 
        {
            return false;
        }

        return true;
    }

    virtual bool terminate()
    {
        _panorama = image::Image<image::RGBAfColor>(_outputRoi.width, _outputRoi.height, true, image::RGBAfColor(0.0f, 0.0f, 0.0f, 0.0f));

        if (!_pyramidPanorama.rebuild(_panorama, _outputRoi)) 
        {
            return false;
        }

        for (int i = 0; i < _outputRoi.height; i++) 
        {
            for (int j = 0; j < _outputRoi.width; j++)
            {
                image::RGBAfColor c = _panorama(i, j);

                image::RGBAfColor out;
                out.r() = std::exp(c.r());
                out.g() = std::exp(c.g());
                out.b() = std::exp(c.b());
                out.a() = c.a();

                _panorama(i, j) = out;
            }
        }

        return true;
    }

protected:
    const int _gaussianFilterRadius = 2;
    LaplacianPyramid _pyramidPanorama;
    size_t _bands;
};

} // namespace aliceVision
