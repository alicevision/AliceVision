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

        if(!color.fill(image::RGBfColor(0.0f, 0.0f, 0.0f)))
        {
            return false;
        }

        if(!weights.fill(0.0f))
        {
            return false;
        }

        _levels.push_back(color);
        _weights.push_back(weights);

        height = int(ceil(float(height) / 2.0f));
        width = int(ceil(float(width) / 2.0f));
    }

    return true;
}

bool LaplacianPyramid::augment(image::TileCacheManager::shared_ptr & cacheManager, size_t newMaxLevels)
{
    ALICEVISION_LOG_INFO("augment number of levels to " << newMaxLevels);
    
    if(newMaxLevels <= _levels.size())
    {
        return false;
    }

    int oldMaxLevels = _levels.size();
    _maxLevels = newMaxLevels;


    //Get content of last level of pyramid
    CachedImage<image::RGBfColor> largerColor;
    if(!largerColor.createImage(cacheManager, _levels[oldMaxLevels - 1].getWidth(), _levels[oldMaxLevels - 1].getHeight()))
    {
        return false;
    }

    if (!largerColor.deepCopy(_levels[oldMaxLevels - 1])) 
    {
        return false;
    }

    CachedImage<float> largerWeight;
    if(!largerWeight.createImage(cacheManager, _weights[oldMaxLevels - 1].getWidth(), _weights[oldMaxLevels - 1].getHeight()))
    {
        return false;
    }

    if (!largerWeight.deepCopy(_weights[oldMaxLevels - 1])) 
    {
        return false;
    }

    //Last level was multiplied by the weight. 
    //Remove this factor
    largerColor.perPixelOperation(largerWeight, 
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

    //Create a mask
    CachedImage<unsigned char> largerMask;
    if(!largerMask.createImage(cacheManager, largerWeight.getWidth(), largerWeight.getHeight()))
    {
        return false;
    }

    //Build the mask
    largerMask.perPixelOperation(largerWeight, 
        [](const unsigned char & c, const float & w) -> unsigned char
        {
            if (w < 1e-6) 
            {
                return 0;
            }

            return 255;
        }
    );

    if (!feathering(largerColor, largerMask))
    {
        return false;
    }


    //Augment the number of levels
    for (int level = oldMaxLevels; level < newMaxLevels; level++)
    {
        CachedImage<image::RGBfColor> pyramidImage;
        CachedImage<float> pyramidWeights;

        if(!pyramidImage.createImage(cacheManager, _levels[_levels.size() - 1].getWidth() / 2, _levels[_levels.size() - 1].getHeight() / 2))
        {
            return false;
        }

        if(!pyramidWeights.createImage(cacheManager, _weights[_weights.size() - 1].getWidth() / 2, _weights[_weights.size() - 1].getHeight() / 2))
        {
            return false;
        }
        
        pyramidImage.fill(image::RGBfColor(0.0f));
        pyramidWeights.fill(0.0f);

        _levels.push_back(pyramidImage);
        _weights.push_back(pyramidWeights);
    }


    const int processingSize = 512;
    const size_t borderSize = 5;

    CachedImage<image::RGBfColor> currentImage = largerColor;
    CachedImage<float> currentWeights = largerWeight;

    for (int level = oldMaxLevels - 1; level < _maxLevels - 1; level++) 
    {
        CachedImage<image::RGBfColor> nextImage;
        CachedImage<float> nextWeights;

        if(!nextImage.createImage(cacheManager, currentImage.getWidth() / 2, currentImage.getHeight() / 2))
        {
            return false;
        }

        if(!nextWeights.createImage(cacheManager, currentImage.getWidth() / 2, currentImage.getHeight() / 2))
        {
            return false;
        }

        nextImage.fill(image::RGBfColor(0.0f));
        nextWeights.fill(0.0f);

        for (int y = 0; y < nextImage.getHeight(); y += processingSize)
        {
            for (int x = 0; x < nextImage.getWidth(); x += processingSize)
            {
                BoundingBox nextBbox;
                nextBbox.left = x;
                nextBbox.top = y;
                nextBbox.width = processingSize;
                nextBbox.height = processingSize;
                nextBbox.clampRight(nextImage.getWidth() - 1);
                nextBbox.clampBottom(nextImage.getHeight() - 1);

                BoundingBox dilatedNextBbox = nextBbox.dilate(borderSize);
                dilatedNextBbox.clampLeft();
                dilatedNextBbox.clampTop();
                dilatedNextBbox.clampBottom(nextImage.getHeight() - 1);

                BoundingBox currentBbox = nextBbox.doubleSize();
                BoundingBox dilatedCurrentBbox = dilatedNextBbox.doubleSize();
                
                aliceVision::image::Image<image::RGBfColor> extractedColor(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                if (!loopyCachedImageExtract(extractedColor, currentImage, dilatedCurrentBbox)) 
                {
                    return false;
                }

                aliceVision::image::Image<float> extractedWeight(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                if (!loopyCachedImageExtract(extractedWeight, currentWeights, dilatedCurrentBbox)) 
                {
                    return false;
                }

                
                /*Compute raw next scale from current scale */
                aliceVision::image::Image<image::RGBfColor> buf(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                aliceVision::image::Image<float> bufw(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                aliceVision::image::Image<image::RGBfColor> colorDownscaled(dilatedNextBbox.width, dilatedNextBbox.height);
                aliceVision::image::Image<float> weightDownscaled(dilatedNextBbox.width, dilatedNextBbox.height);

                convolveGaussian5x5<image::RGBfColor>(buf, extractedColor);
                convolveGaussian5x5<float>(bufw, extractedWeight);
                
                downscale(colorDownscaled, buf);
                downscale(weightDownscaled, bufw);

                BoundingBox saveBoundingBox;
                saveBoundingBox.left = nextBbox.left - dilatedNextBbox.left;
                saveBoundingBox.top = nextBbox.top - dilatedNextBbox.top;
                saveBoundingBox.width = nextBbox.width;
                saveBoundingBox.height = nextBbox.height;

                

                if (!loopyCachedImageAssign(nextImage, colorDownscaled, nextBbox, saveBoundingBox)) {
                    return false;
                }

                if (!loopyCachedImageAssign(nextWeights, weightDownscaled, nextBbox, saveBoundingBox)) {
                    return false;
                }



                /* Compute difference */
                aliceVision::image::Image<image::RGBfColor> buf2(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                upscale(buf, colorDownscaled);
                convolveGaussian5x5<image::RGBfColor>(buf2, buf);

                for (int i = 0; i  < buf2.Height(); i++) {
                    for (int j = 0; j < buf2.Width(); j++) {
                        buf2(i,j) *= 4.0f;
                    }
                }

                substract(extractedColor, extractedColor, buf2);

                for (int i = 0; i < extractedColor.Height(); i++) 
                {
                    for (int j = 0; j < extractedColor.Width(); j++)
                    {
                        extractedColor(i, j).r() = extractedColor(i, j).r() * extractedWeight(i, j);
                        extractedColor(i, j).g() = extractedColor(i, j).r() * extractedWeight(i, j);
                        extractedColor(i, j).b() = extractedColor(i, j).b() * extractedWeight(i, j);
                    }
                }

                
                saveBoundingBox.left = currentBbox.left - dilatedCurrentBbox.left;
                saveBoundingBox.top = currentBbox.top - dilatedCurrentBbox.top;
                saveBoundingBox.width = currentBbox.width;
                saveBoundingBox.height = currentBbox.height;

                if (!loopyCachedImageAssign(_levels[level], extractedColor, currentBbox, saveBoundingBox)) {
                    return false;
                }

                if (!loopyCachedImageAssign(_weights[level], extractedWeight, currentBbox, saveBoundingBox)) {
                    return false;
                } 
            }
        }

        currentImage = nextImage;
        currentWeights = nextWeights;
    }


    currentImage.perPixelOperation(currentWeights, 
        [](const image::RGBfColor & c, const float & w) -> image::RGBfColor 
        {
            image::RGBfColor r;

            r.r() = c.r() * w;
            r.g() = c.g() * w;
            r.b() = c.b() * w;

            return r;
        }
    );


    _levels[_levels.size() - 1].deepCopy(currentImage);
    _weights[_weights.size() - 1].deepCopy(currentWeights);


    return true;
}


static int pos = 0;
bool LaplacianPyramid::apply(const aliceVision::image::Image<image::RGBfColor>& source,
                             const aliceVision::image::Image<unsigned char>& mask,
                             const aliceVision::image::Image<float>& weights, size_t offset_x, size_t offset_y)
{
    //We assume the input source has been feathered 
    //and resized to be a simili power of 2 for the needed scales.

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

        if (!merge(current_color, current_weights, l, offset_x, offset_y)) 
        {
            return false;
        }

        current_color = next_color;
        current_weights = next_weights;
        current_mask = next_mask;

        width /= 2;
        height /= 2;
        offset_x /= 2;
        offset_y /= 2;
    }

    if (!merge(current_color, current_weights, _levels.size() - 1, offset_x, offset_y))
    {
        return false;
    }
    pos++;

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
    extractBb.clampBottom(img.getHeight() - 1);
    

    BoundingBox inputBb = extractBb;
    inputBb.left = 0;
    inputBb.top = 0;

    if (!loopyCachedImageExtract(extractedColor, img, extractBb)) 
    {
        return false;
    }

    if (!loopyCachedImageExtract(extractedWeight, weight, extractBb)) 
    {
        return false;
    }

    bool special = false;
    if (img.getWidth() % 2)
    {
        special = true;
    }
    
    for(int y = 0; y < inputBb.height; y++)
    {   
        int vx = 0;
        for(int x = 0; x < inputBb.width && vx < inputBb.width; x++)
        {   
            //Special process for levels with a width non divided by 2
            //To account for the upscale difference, we "add" a black column 
            if (special)
            {
                int posX = offset_x + x;
                if (posX == img.getWidth() - 1)
                {
                    vx++;
                }
            }

            extractedColor(y, x).r() += oimg(y, vx).r() * oweight(y, vx);
            extractedColor(y, x).g() += oimg(y, vx).g() * oweight(y, vx);
            extractedColor(y, x).b() += oimg(y, vx).b() * oweight(y, vx);
            extractedWeight(y, x) += oweight(y, vx);
            
            vx++;
        }
    }

    if (!loopyCachedImageAssign(img, extractedColor, extractBb, inputBb)) {
        return false;
    }
    
    if (!loopyCachedImageAssign(weight, extractedWeight, extractBb, inputBb)) {
        return false;
    }

    return true;
}

bool LaplacianPyramid::rebuild(CachedImage<image::RGBAfColor>& output)
{
    // We first want to compute the final pixels mean
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
        const size_t processingSize = 512;
        const size_t borderSize = 5;

        int halfLevel = l + 1;
        int currentLevel = l;

        int x = 0;
        int y = 0;

        bool specialBorder = false;
        if (_levels[halfLevel].getWidth() % 2)
        {
            specialBorder = true;
        }

        for (int py = 0; py < _levels[halfLevel].getHeight(); py += processingSize)
        {
            for (int px = 0; px < _levels[halfLevel].getWidth(); px += processingSize) 
            {
                //Estimate the Bounding box
                //Make sure we are not processing outside of the image bounding box
                BoundingBox extractedBb;
                extractedBb.left = px;
                extractedBb.top = py;
                extractedBb.width = processingSize;
                extractedBb.height = processingSize;
                extractedBb.clampLeft();
                extractedBb.clampTop();
                extractedBb.clampRight(_levels[halfLevel].getWidth() - 1);
                extractedBb.clampBottom(_levels[halfLevel].getHeight() - 1);

                BoundingBox doubleBb = extractedBb.doubleSize();
                doubleBb.clampLeft();
                doubleBb.clampTop();
                doubleBb.clampRight(_levels[currentLevel].getWidth() - 1);
                doubleBb.clampBottom(_levels[currentLevel].getHeight() - 1);

                //Add borders to this bounding box for filtering
                BoundingBox dilatedBb = extractedBb.dilate(borderSize);
                dilatedBb.clampTop();   
                dilatedBb.clampBottom(_levels[halfLevel].getHeight() - 1);    

                //If the box has a negative left border,
                //it is equivalent (with the loop, to shifting at the end)
                int leftBorder = extractedBb.left - dilatedBb.left;
                int topBorder = extractedBb.top - dilatedBb.top;
                if (dilatedBb.left < 0)
                {
                    dilatedBb.left = _levels[halfLevel].getWidth() + dilatedBb.left;
                }                

                aliceVision::image::Image<image::RGBfColor> extracted(dilatedBb.width, dilatedBb.height);
                if (!loopyCachedImageExtract(extracted, _levels[halfLevel], dilatedBb)) 
                {
                    return false;   
                }

                aliceVision::image::Image<image::RGBfColor> extractedNext(doubleBb.width, doubleBb.height);
                if (!loopyCachedImageExtract(extractedNext, _levels[currentLevel], doubleBb)) 
                {
                    return false;
                }

                aliceVision::image::Image<image::RGBfColor> buf(dilatedBb.width * 2, dilatedBb.height * 2);
                aliceVision::image::Image<image::RGBfColor> buf2(dilatedBb.width * 2, dilatedBb.height * 2);

                upscale(buf, extracted);
                convolveGaussian5x5<image::RGBfColor>(buf2, buf, false);

                for(int y = 0; y < buf2.Height(); y++)
                {
                    for(int x = 0; x < buf2.Width(); x++)
                    {
                        buf2(y, x) *= 4.0f;
                    }
                }
                
                int shiftY = topBorder * 2;
                int shiftX = leftBorder * 2;
                for (int y = 0; y < doubleBb.height; y++)
                {
                    for (int x = 0; x < doubleBb.width; x++)
                    {
                        extractedNext(y, x) += buf2(y + shiftY, x + shiftX);
                    }
                }

                BoundingBox inputAssigmentBb = doubleBb;
                inputAssigmentBb.left = 0;
                inputAssigmentBb.top = 0;
                if (!loopyCachedImageAssign(_levels[currentLevel], extractedNext, doubleBb, inputAssigmentBb)) 
                {
                    std::cout << "failed assign" << std::endl;
                    return false;
                }
            }
        }

        removeNegativeValues(_levels[currentLevel]);

        
    }

    


    for(int i = 0; i < output.getTiles().size(); i++)
    {

        std::vector<image::CachedTile::smart_pointer> & rowOutput = output.getTiles()[i];
        std::vector<image::CachedTile::smart_pointer> & rowInput = _levels[0].getTiles()[i];
        std::vector<image::CachedTile::smart_pointer> & rowWeight = _weights[0].getTiles()[i];

        for(int j = 0; j < rowOutput.size(); j++)
        {

            if(!rowOutput[j]->acquire())
            {
                return false;
            }

            if(!rowInput[j]->acquire())
            {
                return false;
            }

            if(!rowWeight[j]->acquire())
            {
                return false;
            }

            image::RGBAfColor* ptrOutput = (image::RGBAfColor *)rowOutput[j]->getDataPointer();
            image::RGBfColor* ptrInput = (image::RGBfColor *)rowInput[j]->getDataPointer();
            float* ptrWeight = (float *)rowWeight[j]->getDataPointer();

            for (int k = 0; k < output.getTileSize() * output.getTileSize(); k++) 
            {
                ptrOutput[k].r() = ptrInput[k].r();
                ptrOutput[k].g() = ptrInput[k].g();
                ptrOutput[k].b() = ptrInput[k].b();

                if(ptrWeight[k] < 1e-6)
                {
                    ptrOutput[k].a() = 0.0f;
                }
                else
                {
                    ptrOutput[k].a() = 1.0f;
                }
            }
        }
    }

    return true;
}

} // namespace aliceVision
