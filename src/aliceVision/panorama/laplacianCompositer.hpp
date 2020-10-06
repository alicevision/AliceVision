#pragma once

#include "compositer.hpp"
#include "feathering.hpp"
#include "laplacianPyramid.hpp"

namespace aliceVision
{

template <class T>
bool makeImagePyramidCompatible(image::Image<T>& output, 
                                size_t& out_offset_x, size_t& out_offset_y,
                                const image::Image<T>& input,
                                size_t offset_x, size_t offset_y, 
                                size_t num_levels)
{

    if(num_levels == 0)
    {
        return false;
    }

    double max_scale = 1.0 / pow(2.0, num_levels - 1);

    double low_offset_x = double(offset_x) * max_scale;
    double low_offset_y = double(offset_y) * max_scale;

    /*Make sure offset is integer even at the lowest level*/
    double corrected_low_offset_x = floor(low_offset_x);
    double corrected_low_offset_y = floor(low_offset_y);

    /*Add some borders on the top and left to make sure mask can be smoothed*/
    corrected_low_offset_x = std::max(0.0, corrected_low_offset_x - 3.0);
    corrected_low_offset_y = std::max(0.0, corrected_low_offset_y - 3.0);

    /*Compute offset at largest level*/
    out_offset_x = size_t(corrected_low_offset_x / max_scale);
    out_offset_y = size_t(corrected_low_offset_y / max_scale);

    /*Compute difference*/
    double doffset_x = double(offset_x) - double(out_offset_x);
    double doffset_y = double(offset_y) - double(out_offset_y);

    /* update size with border update */
    double large_width = double(input.Width()) + doffset_x;
    double large_height = double(input.Height()) + doffset_y;

    /* compute size at largest scale */
    double low_width = large_width * max_scale;
    double low_height = large_height * max_scale;

    /*Make sure width is integer event at the lowest level*/
    double corrected_low_width = ceil(low_width);
    double corrected_low_height = ceil(low_height);

    /*Add some borders on the right and bottom to make sure mask can be smoothed*/
    corrected_low_width = corrected_low_width + 3;
    corrected_low_height = corrected_low_height + 3;

    /*Compute size at largest level*/
    size_t width = size_t(corrected_low_width / max_scale);
    size_t height = size_t(corrected_low_height / max_scale);

    output = image::Image<T>(width, height, true, T(0.0f));
    output.block(doffset_y, doffset_x, input.Height(), input.Width()) = input;

    return true;
}

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

        optimal_scale = 5;

        return optimal_scale;
    }

    
    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& color,
                        const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        size_t offset_x, size_t offset_y, const BoundingBox & contentBox)
    {
        size_t optimalScale = getOptimalScale(contentBox.width, contentBox.height);
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
        /*for(int i = 0; i < feathered.Height(); i++)
        {
            for(int j = 0; j < feathered.Width(); j++)
            {
                feathered(i, j).r() = std::log(std::max(1e-8f, feathered(i, j).r()));
                feathered(i, j).g() = std::log(std::max(1e-8f, feathered(i, j).g()));
                feathered(i, j).b() = std::log(std::max(1e-8f, feathered(i, j).b()));
            }
        }*/

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

        /*_panorama.perPixelOperation(
            [](const image::RGBAfColor & a) -> image::RGBAfColor {

                image::RGBAfColor out;

                out.r() = std::exp(a.r());
                out.g() = std::exp(a.g());
                out.b() = std::exp(a.b());

                return out;
            }
        );*/

        return true;
    }

protected:
    LaplacianPyramid _pyramidPanorama;
    size_t _bands;
};

} // namespace aliceVision
