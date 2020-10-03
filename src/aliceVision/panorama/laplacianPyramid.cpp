#include "laplacianPyramid.hpp"

#include "feathering.hpp"
#include "gaussian.hpp"
#include "compositer.hpp"

namespace aliceVision
{

LaplacianPyramid::LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels) :
_baseWidth(base_width),
_baseHeight(base_height),
_maxLevels(max_levels)
{
}

bool LaplacianPyramid::initialize(image::TileCacheManager::shared_ptr & cacheManager) 
{
    size_t width = _baseWidth;
    size_t height = _baseHeight;

    /*Make sure pyramid size can be divided by 2 on each levels*/
    double max_scale = 1.0 / pow(2.0, _maxLevels - 1);

    /*Prepare pyramid*/
    for(int lvl = 0; lvl < _maxLevels; lvl++)
    {
        CachedImage<image::RGBfColor> color;
        CachedImage<float> weights;

        if(!color.createImage(cacheManager, width, height))
        {
            return false;
        }

        if(!weights.createImage(cacheManager, width, height))
        {
            return false;
        }

        if(!color.perPixelOperation(
            [](image::RGBfColor ) -> image::RGBfColor 
            { 
                return image::RGBfColor(0.0f, 0.0f, 0.0f); 
            })
          )
        {
            return false;
        }

        if(!weights.perPixelOperation(
            [](float) -> float
            { 
                return 0.0f; 
            })
          )
        {
            return false;
        }

        _levels.push_back(color);
        _weights.push_back(weights);

        height /= 2;
        width /= 2;
    }

    return true;
}

bool LaplacianPyramid::augment(size_t new_max_levels)
{

    /*if(new_max_levels <= _levels.size())
    {
        return false;
    }

    _maxLevels = new_max_levels;

    size_t old_max_level = _levels.size();

    image::Image<image::RGBfColor> current_color = _levels[_levels.size() - 1];
    image::Image<float> current_weights = _weights[_weights.size() - 1];

    _levels[_levels.size() - 1].fill(image::RGBfColor(0.0f, 0.0f, 0.0f));
    _weights[_weights.size() - 1].fill(0.0f);

    image::Image<unsigned char> current_mask(current_color.Width(), current_color.Height(), true, 0);
    image::Image<image::RGBfColor> current_color_feathered(current_color.Width(), current_color.Height());

    for(int i = 0; i < current_color.Height(); i++)
    {
        for(int j = 0; j < current_color.Width(); j++)
        {
            if(current_weights(i, j) < 1e-6)
            {
                current_color(i, j) = image::RGBfColor(0.0);
                continue;
            }

            current_color(i, j).r() = current_color(i, j).r() / current_weights(i, j);
            current_color(i, j).g() = current_color(i, j).g() / current_weights(i, j);
            current_color(i, j).b() = current_color(i, j).b() / current_weights(i, j);
            current_mask(i, j) = 255;
        }
    }

    feathering(current_color_feathered, current_color, current_mask);
    current_color = current_color_feathered;

    for(int l = old_max_level; l < new_max_levels; l++)
    {

        _levels.emplace_back(_levels[l - 1].Width() / 2, _levels[l - 1].Height() / 2, true,
                             image::RGBfColor(0.0f, 0.0f, 0.0f));
        _weights.emplace_back(_weights[l - 1].Width() / 2, _weights[l - 1].Height() / 2, true, 0.0f);
    }

    int width = current_color.Width();
    int height = current_color.Height();
    image::Image<image::RGBfColor> next_color;
    image::Image<float> next_weights;

    for(int l = old_max_level - 1; l < new_max_levels - 1; l++)
    {
        aliceVision::image::Image<image::RGBfColor> buf(width, height);
        aliceVision::image::Image<image::RGBfColor> buf2(width, height);
        aliceVision::image::Image<float> bufw(width, height);

        next_color = aliceVision::image::Image<image::RGBfColor>(width / 2, height / 2);
        next_weights = aliceVision::image::Image<float>(width / 2, height / 2);

        convolveGaussian5x5<image::RGBfColor>(buf, current_color);
        downscale(next_color, buf);

        convolveGaussian5x5<float>(bufw, current_weights);
        downscale(next_weights, bufw);

        upscale(buf, next_color);
        convolveGaussian5x5<image::RGBfColor>(buf2, buf);

        for(int i = 0; i < buf2.Height(); i++)
        {
            for(int j = 0; j < buf2.Width(); j++)
            {
                buf2(i, j) *= 4.0f;
            }
        }

        substract(current_color, current_color, buf2);

        merge(current_color, current_weights, l, 0, 0);

        current_color = next_color;
        current_weights = next_weights;
        width /= 2;
        height /= 2;
    }

    merge(current_color, current_weights, _levels.size() - 1, 0, 0);*/

    return true;
}

bool LaplacianPyramid::apply(const aliceVision::image::Image<image::RGBfColor>& source,
                             const aliceVision::image::Image<unsigned char>& mask,
                             const aliceVision::image::Image<float>& weights, size_t offset_x, size_t offset_y)
{

    

    int width = source.Width();
    int height = source.Height();

    /* Convert mask to alpha layer */
    image::Image<float> mask_float(width, height);
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            if(mask(i, j))
            {
                mask_float(i, j) = 1.0f;
            }
            else
            {
                mask_float(i, j) = 0.0f;
            }
        }
    }

    image::Image<image::RGBfColor> current_color = source;
    image::Image<image::RGBfColor> next_color;
    image::Image<float> current_weights = weights;
    image::Image<float> next_weights;
    image::Image<float> current_mask = mask_float;
    image::Image<float> next_mask;

    for(int l = 0; l < _levels.size() - 1; l++)
    {
        aliceVision::image::Image<image::RGBfColor> buf_masked(width, height);
        aliceVision::image::Image<image::RGBfColor> buf(width, height);
        aliceVision::image::Image<image::RGBfColor> buf2(width, height);
        aliceVision::image::Image<float> buf_float(width, height);

        next_color = aliceVision::image::Image<image::RGBfColor>(width / 2, height / 2);
        next_weights = aliceVision::image::Image<float>(width / 2, height / 2);
        next_mask = aliceVision::image::Image<float>(width / 2, height / 2);

        /*Apply mask to content before convolution*/
        for(int i = 0; i < current_color.Height(); i++)
        {
            for(int j = 0; j < current_color.Width(); j++)
            {
                if(std::abs(current_mask(i, j)) > 1e-6)
                {
                    buf_masked(i, j) = current_color(i, j);
                }
                else
                {
                    buf_masked(i, j).r() = 0.0f;
                    buf_masked(i, j).g() = 0.0f;
                    buf_masked(i, j).b() = 0.0f;
                    current_weights(i, j) = 0.0f;
                }
            }
        }

        convolveGaussian5x5<image::RGBfColor>(buf, buf_masked);
        convolveGaussian5x5<float>(buf_float, current_mask);

        /*
        Normalize given mask
        */
        for(int i = 0; i < current_color.Height(); i++)
        {
            for(int j = 0; j < current_color.Width(); j++)
            {

                float m = buf_float(i, j);

                if(std::abs(m) > 1e-6)
                {
                    buf(i, j).r() = buf(i, j).r() / m;
                    buf(i, j).g() = buf(i, j).g() / m;
                    buf(i, j).b() = buf(i, j).b() / m;
                    buf_float(i, j) = 1.0f;
                }
                else
                {
                    buf(i, j).r() = 0.0f;
                    buf(i, j).g() = 0.0f;
                    buf(i, j).b() = 0.0f;
                    buf_float(i, j) = 0.0f;
                }
            }
        }

        downscale(next_color, buf);
        downscale(next_mask, buf_float);

        upscale(buf, next_color);
        convolveGaussian5x5<image::RGBfColor>(buf2, buf);

        for(int i = 0; i < buf2.Height(); i++)
        {
            for(int j = 0; j < buf2.Width(); j++)
            {
                buf2(i, j) *= 4.0f;
            }
        }

        substract(current_color, current_color, buf2);

        convolveGaussian5x5<float>(buf_float, current_weights);
        downscale(next_weights, buf_float);

        merge(current_color, current_weights, l, offset_x, offset_y);

        current_color = next_color;
        current_weights = next_weights;
        current_mask = next_mask;

        width /= 2;
        height /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    merge(current_color, current_weights, _levels.size() - 1, offset_x, offset_y);

    return true;
}


bool LaplacianPyramid::merge(const aliceVision::image::Image<image::RGBfColor>& oimg,
                             const aliceVision::image::Image<float>& oweight, size_t level, size_t offset_x,
                             size_t offset_y)
{
    CachedImage<image::RGBfColor> & img = _levels[level];
    CachedImage<float> & weight = _weights[level];

    aliceVision::image::Image<image::RGBfColor> extractedColor(oimg.Width(), oimg.Height());
    aliceVision::image::Image<float> extractedWeight(oimg.Width(), oimg.Height());


    BoundingBox extractBb;
    extractBb.left = offset_x;
    extractBb.top = offset_y;
    extractBb.width = oimg.Width();
    extractBb.height = oimg.Height();


    if (!loopyCachedImageExtract(extractedColor, img, extractBb)) 
    {
        return false;
    }

    if (!loopyCachedImageExtract(extractedWeight, weight, extractBb)) 
    {
        return false;
    }
    
    for(int i = 0; i < oimg.Height(); i++)
    {
        for(int j = 0; j < oimg.Width(); j++)
        {
            extractedColor(i, j).r() += oimg(i, j).r() * oweight(i, j);
            extractedColor(i, j).g() += oimg(i, j).g() * oweight(i, j);
            extractedColor(i, j).b() += oimg(i, j).b() * oweight(i, j);
            extractedWeight(i, j) += oweight(i, j);
        }
    }

    if (!loopyCachedImageAssign(img, extractedColor, extractBb)) {
        return false;
    }

    if (!loopyCachedImageAssign(weight, extractedWeight, extractBb)) {
        return false;
    }

    return true;
}

bool LaplacianPyramid::rebuild(CachedImage<image::RGBAfColor>& output)
{

    for(int l = 0; l < _levels.size(); l++)
    {
        _levels[l].perPixelOperation(_weights[l], 
            [](const image::RGBfColor & c, const float & w) -> image::RGBfColor 
            {
                if (w < 1e-6) 
                {
                    return image::RGBfColor(0.0f, 0.0f, 0.0f);
                }

                image::RGBfColor r;

                r.r() = c.r() / w;
                r.g() = c.g() / w;
                r.b() = c.b() / w;

                return r;
            }
        );
    }

    
    removeNegativeValues(_levels[_levels.size() - 1]);

    for(int l = _levels.size() - 2; l >= 0; l--)
    {

        /*aliceVision::image::Image<image::RGBfColor> buf(_levels[l].Width(), _levels[l].Height());
        aliceVision::image::Image<image::RGBfColor> buf2(_levels[l].Width(), _levels[l].Height());

        upscale(buf, _levels[l + 1]);
        convolveGaussian5x5<image::RGBfColor>(buf2, buf, true);

        for(int i = 0; i < buf2.Height(); i++)
        {
            for(int j = 0; j < buf2.Width(); j++)
            {
                buf2(i, j) *= 4.0f;
            }
        }

        addition(_levels[l], _levels[l], buf2);*/
        removeNegativeValues(_levels[l]);
    }

    // Write output to RGBA
    /*for(int i = 0; i < output.Height(); i++)
    {
        for(int j = 0; j < output.Width(); j++)
        {
            output(i, j).r() = _levels[0](i, j).r();
            output(i, j).g() = _levels[0](i, j).g();
            output(i, j).b() = _levels[0](i, j).b();

            if(_weights[0](i, j) < 1e-6)
            {
                output(i, j).a() = 0.0f;
            }
            else
            {
                output(i, j).a() = 1.0f;
            }
        }
    }*/

    return true;
}

} // namespace aliceVision