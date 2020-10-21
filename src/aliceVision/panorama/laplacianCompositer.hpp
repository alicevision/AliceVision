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

    size_t getOptimalScale(int width, int height) 
    {
        /*
        Look for the smallest scale such that the image is not smaller than the
        convolution window size.
        minsize / 2^x = 5
        minsize / 5 = 2^x
        x = log2(minsize/5)
        */

        size_t minsize = std::min(width, height);
        const float gaussian_filter_size = 5.0f;
        size_t optimal_scale = size_t(floor(std::log2(double(minsize) / gaussian_filter_size)));
        return 6;//optimal_scale;
    }

    
    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        size_t offset_x, size_t offset_y)
    {
        size_t optimalScale = getOptimalScale(color.Width(), color.Height());
        std::cout << "---" << optimalScale << std::endl;
        if(optimalScale < _bands)
        {
            ALICEVISION_LOG_ERROR("Decreasing scale !");
            return false;
        }

        //If the input scale is more important than previously processed, 
        // The pyramid must be deepened accordingly
        if(optimalScale > _bands)
        {
            _bands = optimalScale;
            _pyramidPanorama.augment(_cacheManager, _bands);
        }

        // Make sure input is compatible with pyramid processing
        size_t new_offset_x, new_offset_y;
        aliceVision::image::Image<image::RGBfColor> color_pot;
        aliceVision::image::Image<unsigned char> mask_pot;
        aliceVision::image::Image<float> weights_pot;

        makeImagePyramidCompatible(color_pot, new_offset_x, new_offset_y, color, offset_x, offset_y, _bands);
        makeImagePyramidCompatible(mask_pot, new_offset_x, new_offset_y, inputMask, offset_x, offset_y, _bands);
        makeImagePyramidCompatible(weights_pot, new_offset_x, new_offset_y, inputWeights, offset_x, offset_y, _bands);

        
        // Fill Color images masked parts with fake but coherent info
        aliceVision::image::Image<image::RGBfColor> feathered;
        if (!feathering(feathered, color_pot, mask_pot)) 
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

        if (!_pyramidPanorama.apply(feathered, mask_pot, weights_pot, new_offset_x, new_offset_y)) 
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
    LaplacianPyramid _pyramidPanorama;
    size_t _bands;
};

} // namespace aliceVision
