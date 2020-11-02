#pragma once

#include "compositer.hpp"
#include "feathering.hpp"
#include "laplacianPyramid.hpp"

namespace aliceVision
{

class LaplacianCompositer : public Compositer
{
public:
    LaplacianCompositer(image::TileCacheManager::shared_ptr & cacheManager, size_t outputWidth, size_t outputHeight, size_t bands)
        : Compositer(cacheManager, outputWidth, outputHeight)
        , _pyramidPanorama(outputWidth, outputHeight, bands)
        , _bands(bands)
    {
    }

    virtual bool initialize() 
    { 
        if (!Compositer::initialize()) 
        {
            return false;
        }

        return _pyramidPanorama.initialize(_cacheManager);
    }

    virtual size_t getOptimalScale(int width, int height) const
    {
        /*
        Look for the smallest scale such that the image is not smaller than the
        convolution window size.
        minsize / 2^x = 5
        minsize / 5 = 2^x
        x = log2(minsize/5)
        */

        size_t minsize = std::min(width, height);

        int gaussianFilterSize = 1 + 2 * _gaussianFilterRadius;
        
        size_t optimal_scale = size_t(floor(std::log2(double(minsize) / gaussianFilterSize)));
        
        return (optimal_scale - 1/*Security*/);
    }

    virtual int getBorderSize() const 
    {
        return _gaussianFilterRadius;
    }
    
    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        int offsetX, int offsetY) 
    {
        size_t optimalScale = getOptimalScale(color.Width(), color.Height());
        size_t optimalLevelsCount = optimalScale + 1;

        if(optimalLevelsCount < _bands)
        {
            ALICEVISION_LOG_ERROR("Decreasing level count !");
            return false;
        }

        //If the input scale is more important than previously processed, 
        // The pyramid must be deepened accordingly
        if(optimalLevelsCount > _bands)
        {
            _bands = optimalLevelsCount;
            if (!_pyramidPanorama.augment(_cacheManager, _bands)) 
            {
                return false;
            }
        }

        // Make sure input is compatible with pyramid processing
        // See comments inside function for details
        size_t newOffsetX, newOffsetY;
        aliceVision::image::Image<image::RGBfColor> colorPot;
        aliceVision::image::Image<unsigned char> maskPot;
        aliceVision::image::Image<float> weightsPot;

        makeImagePyramidCompatible(colorPot, newOffsetX, newOffsetY, color, offsetX, offsetY, getBorderSize(), _bands);
        makeImagePyramidCompatible(maskPot, newOffsetX, newOffsetY, inputMask, offsetX, offsetY, getBorderSize(), _bands);
        makeImagePyramidCompatible(weightsPot, newOffsetX, newOffsetY, inputWeights, offsetX, offsetY, getBorderSize(), _bands);

        
        // Fill Color images masked parts with fake but coherent info
        aliceVision::image::Image<image::RGBfColor> feathered;
        if (!feathering(feathered, colorPot, maskPot)) 
        {
            return false;
        }

        /*To log space for hdr*/
        for(int i = 0; i < feathered.Height(); i++)
        {
            for(int j = 0; j < feathered.Width(); j++)
            {
                feathered(i, j).r() = std::log(std::max(1e-8f, feathered(i, j).r()));
                feathered(i, j).g() = std::log(std::max(1e-8f, feathered(i, j).g()));
                feathered(i, j).b() = std::log(std::max(1e-8f, feathered(i, j).b()));
            }
        }

        /* Convert mask to alpha layer */
        image::Image<float> maskFloat(maskPot.Width(), maskPot.Height());
        for(int i = 0; i < maskPot.Height(); i++)
        {
            for(int j = 0; j < maskPot.Width(); j++)
            {
                if(maskPot(i, j))
                {
                    maskFloat(i, j) = 1.0f;
                }
                else
                {
                    maskFloat(i, j) = 0.0f;
                }
            }
        }

        if (!_pyramidPanorama.apply(feathered, maskFloat, weightsPot, 0, newOffsetX, newOffsetY)) 
        {
            return false;
        }

        return true;
    }

    virtual bool terminate()
    {

        if (!_pyramidPanorama.rebuild(_panorama)) 
        {
            return false;
        }

        _panorama.perPixelOperation(
            [](const image::RGBAfColor & a) -> image::RGBAfColor {

                image::RGBAfColor out;
                out.r() = std::exp(a.r());
                out.g() = std::exp(a.g());
                out.b() = std::exp(a.b());
                out.a() = a.a();

                return out;
            }
        );

        return true;
    }

protected:
    const int _gaussianFilterRadius = 2;
    LaplacianPyramid _pyramidPanorama;
    size_t _bands;
};

} // namespace aliceVision
