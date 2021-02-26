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
    omp_init_lock(&_merge_lock);
}

LaplacianPyramid::~LaplacianPyramid()
{
    omp_destroy_lock(&_merge_lock);
}

bool LaplacianPyramid::initialize() 
{
    size_t width = _baseWidth;
    size_t height = _baseHeight;

    /*Make sure pyramid size can be divided by 2 on each levels*/
    double max_scale = 1.0 / pow(2.0, _maxLevels - 1);

    /*Prepare pyramid*/
    for(int lvl = 0; lvl < _maxLevels; lvl++)
    {
        image::Image<image::RGBfColor> color(width, height, true, image::RGBfColor(0.0f, 0.0f, 0.0f));
        image::Image<float> weights(width, height, true, 0.0f);

        _levels.push_back(color);
        _weights.push_back(weights);

        width = int(ceil(float(width) / 2.0f));
        height = int(ceil(float(height) / 2.0f));
    }

    return true;
}

bool LaplacianPyramid::apply(aliceVision::image::Image<image::RGBfColor>& source,
                             aliceVision::image::Image<float>& mask,
                             aliceVision::image::Image<float>& weights, 
                             const BoundingBox &outputBoundingBox, const BoundingBox &contentBoudingBox)
{
    //We assume the input source has been feathered 
    //the outputBoundingBox has been scaled to accept the pyramid scales
    int width = outputBoundingBox.width;
    int height = outputBoundingBox.height;
    int offsetX = outputBoundingBox.left;
    int offsetY = outputBoundingBox.top;

    image::Image<image::RGBfColor> currentColor(width, height, true, image::RGBfColor(0.0));
    image::Image<image::RGBfColor> nextColor;
    image::Image<float> currentWeights(width, height, true, 0.0f);
    image::Image<float> nextWeights;
    image::Image<float> currentMask(width, height, true, 0.0f);
    image::Image<float> nextMask;

    for (int i = 0; i < source.Height(); i++)
    {
        int di = contentBoudingBox.top + i;

        memcpy(&currentColor(di, contentBoudingBox.left), &source(i, 0), sizeof(image::RGBfColor) * source.Width());
        memcpy(&currentWeights(di, contentBoudingBox.left), &weights(i, 0), sizeof(float) * source.Width());
        memcpy(&currentMask(di, contentBoudingBox.left), &mask(i, 0), sizeof(float) * source.Width());
    }

    source = aliceVision::image::Image<image::RGBfColor>();
    mask = aliceVision::image::Image<float>();
    weights = aliceVision::image::Image<float>();

    for(int l = 0; l < _levels.size() - 1; l++)
    {
        BoundingBox inputBbox;
        inputBbox.left = offsetX;
        inputBbox.top = offsetY;
        inputBbox.width = width;
        inputBbox.height = height;

        image::Image<image::RGBfColor> bufMasked(width, height);
        image::Image<image::RGBfColor> buf(width, height);
        image::Image<image::RGBfColor> buf2(width, height);
        image::Image<float> bufFloat(width, height);

        // Apply mask to content before convolution
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                if(std::abs(currentMask(i, j)) > 1e-6)
                {
                    bufMasked(i, j) = currentColor(i, j);
                }
                else
                {
                    bufMasked(i, j).r() = 0.0f;
                    bufMasked(i, j).g() = 0.0f;
                    bufMasked(i, j).b() = 0.0f;
                    currentWeights(i, j) = 0.0f;
                }
            }
        }

        if (!convolveGaussian5x5<image::RGBfColor>(buf, bufMasked)) 
        {
            return false;
        }

        if (!convolveGaussian5x5<float>(bufFloat, currentMask)) 
        {
            return false;
        }

        //Normalize given mask
        //(Make sure the convolution sum is 1)
        for(int i = 0; i < buf.Height(); i++)
        {
            for(int j = 0; j < buf.Width(); j++)
            {
                float m = bufFloat(i, j);

                if(std::abs(m) > 1e-6)
                {
                    buf(i, j).r() = buf(i, j).r() / m;
                    buf(i, j).g() = buf(i, j).g() / m;
                    buf(i, j).b() = buf(i, j).b() / m;
                    bufFloat(i, j) = 1.0f;
                }
                else
                {
                    buf(i, j).r() = 0.0f;
                    buf(i, j).g() = 0.0f;
                    buf(i, j).b() = 0.0f;
                    bufFloat(i, j) = 0.0f;
                }
            }
        }


        int nextWidth = width / 2;
        int nextHeight = int(floor(float(height) / 2.0f));

        nextColor = aliceVision::image::Image<image::RGBfColor>(nextWidth, nextHeight);
        nextWeights = aliceVision::image::Image<float>(nextWidth, nextHeight);
        nextMask = aliceVision::image::Image<float>(nextWidth, nextHeight);

        if (!downscale(nextColor, buf)) 
        {
            return false;
        }

        if (!downscale(nextMask, bufFloat)) 
        {
            return false;
        }

        if (!upscale(buf, nextColor)) 
        {
            return false;
        }

        //Filter
        if (!convolveGaussian5x5<image::RGBfColor>(buf2, buf)) 
        {
            return false;
        }

        //Values must be multiplied by 4 as our upscale was using 
        //filling of 0 values
        for(int i = 0; i < buf2.Height(); i++)
        {
            for(int j = 0; j < buf2.Width(); j++)
            {   
                buf2(i, j) *= 4.0f;
            }
        }

        //Only keep the difference (Band pass)
        if (!substract(currentColor, currentColor, buf2)) 
        {
            return false;
        }
        
        //Downscale weights
        if (!convolveGaussian5x5<float>(bufFloat, currentWeights)) 
        {
            return false;
        }

        if (!downscale(nextWeights, bufFloat))
        {
            return false;
        }

        //Merge this view with previous ones
        omp_set_lock(&_merge_lock);
        bool res = merge(currentColor, currentWeights, l, offsetX, offsetY);
        omp_unset_lock(&_merge_lock);

        if (!res)
        {
            return false;
        }

        //Swap buffers
        currentColor = nextColor;
        currentWeights = nextWeights;
        currentMask = nextMask;
        width = nextWidth;
        height = nextHeight;

        //Thanks to previous operation,
        //We are sure the offset is a power of two for the required levels
        offsetX = offsetX / 2;
        offsetY = offsetY / 2;
    }

    InputInfo iinfo;
    iinfo.offsetX = offsetX;
    iinfo.offsetY = offsetY;
    iinfo.color = currentColor;
    iinfo.mask = currentMask;
    iinfo.weights = currentWeights;

    omp_set_lock(&_merge_lock);
    _inputInfos.push_back(iinfo);
    omp_unset_lock(&_merge_lock);
    

    return true;
}

bool LaplacianPyramid::merge(const aliceVision::image::Image<image::RGBfColor>& oimg,
                             const aliceVision::image::Image<float>& oweight, 
                             size_t level, int offsetX, int offsetY)
{
    image::Image<image::RGBfColor> & img = _levels[level];
    image::Image<float> & weight = _weights[level];

    for(int i = 0; i < oimg.Height(); i++)
    {   
        int y = i + offsetY;
        if (y < 0 || y >= img.Height()) continue;

        for(int j = 0; j < oimg.Width(); j++)
        {  
            int x = j + offsetX;
            if (x < 0 || x >= img.Width()) continue;

            img(y, x).r() += oimg(i, j).r() * oweight(i, j);
            img(y, x).g() += oimg(i, j).g() * oweight(i, j);
            img(y, x).b() += oimg(i, j).b() * oweight(i, j);
            weight(y, x) += oweight(i, j);
        }
    }


    return true;
}

bool LaplacianPyramid::rebuild(image::Image<image::RGBAfColor>& output, const BoundingBox & roi)
{
    for (InputInfo & iinfo : _inputInfos)
    {
        if (!merge(iinfo.color, iinfo.weights, _levels.size() - 1, iinfo.offsetX, iinfo.offsetY))
        {
            return false;
        }
    }

    // We first want to compute the final pixels mean
    for(int l = 0; l < _levels.size(); l++)
    {
        image::Image<image::RGBfColor> & level = _levels[l];
        image::Image<float> & weight = _weights[l];

        for (int i = 0; i < level.Height(); i++) 
        {
            for (int j = 0; j < level.Width(); j++)
            {
                float w = weight(i, j);
                image::RGBfColor c = level(i, j);

                if (w < 1e-6) 
                {
                   level(i, j) = image::RGBfColor(0.0f, 0.0f, 0.0f);
                   continue;
                }

                image::RGBfColor r;

                r.r() = c.r() / w;
                r.g() = c.g() / w;
                r.b() = c.b() / w;

                level(i, j) = r;
            }
        }
    }

    removeNegativeValues(_levels[_levels.size() - 1]);

    for(int l = _levels.size() - 2; l >= 0; l--)
    {
        int halfLevel = l + 1;
        int currentLevel = l;

        aliceVision::image::Image<image::RGBfColor> buf(_levels[currentLevel].Width(), _levels[currentLevel].Height());
        aliceVision::image::Image<image::RGBfColor> buf2(_levels[currentLevel].Width(), _levels[currentLevel].Height());

        if (!upscale(buf, _levels[halfLevel])) 
        {
            return false;
        }

        if (!convolveGaussian5x5<image::RGBfColor>(buf2, buf, false)) 
        {
            return false;
        }

        for(int y = 0; y < buf2.Height(); y++)
        {
            for(int x = 0; x < buf2.Width(); x++)
            {
                buf2(y, x) *= 4.0f;
            }
        }
        
        if (!addition(_levels[currentLevel], _levels[currentLevel], buf2))
        {
            return false;
        }

        removeNegativeValues(_levels[currentLevel]);
    }
    
    image::Image<image::RGBfColor> & level = _levels[0];
    image::Image<float> & weight = _weights[0];
    for(int i = 0; i < roi.height; i++)
    {
        int y = i + roi.top;

        for(int j = 0; j < roi.width; j++)
        {
            int x = j + roi.left;

            output(i, j).r() = level(y, x).r();
            output(i, j).g() = level(y, x).g();
            output(i, j).b() = level(y, x).b();

            if (weight(y, x) < 1e-6)
            {
                output(i, j).a() = 0.0f;
            }
            else
            {
                output(i, j).a() = 1.0f;
            }
        }
    }

    return true;
}

} // namespace aliceVision
