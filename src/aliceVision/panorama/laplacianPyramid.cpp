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

        //If the level width is odd, it is a problem.
        //Let update this level to an even size. we'll manage it later
        //Store the real size to know we updated it
        _realWidths.push_back(width);
        if (width % 2)
        {
            width++;
        }

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

        width = width / 2;
        height = int(ceil(float(height) / 2.0f));
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

    //Get weights of last level of pyramid
    CachedImage<float> largerWeight;
    if(!largerWeight.createImage(cacheManager, _weights[oldMaxLevels - 1].getWidth(), _weights[oldMaxLevels - 1].getHeight()))
    {
        return false;
    }

    if (!largerWeight.deepCopy(_weights[oldMaxLevels - 1])) 
    {
        return false;
    }

    // Last level was multiplied by the weight. 
    // Remove this factor
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

    // Create a mask
    CachedImage<unsigned char> largerMask;
    if(!largerMask.createImage(cacheManager, largerWeight.getWidth(), largerWeight.getHeight()))
    {
        return false;
    }

    // Build the mask
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


    // Put colors in masked areas
    if (!feathering(largerColor, largerMask))
    {
        return false;
    }

    // Augment the number of levels in the pyramid
    for (int level = oldMaxLevels; level < newMaxLevels; level++)
    {
        CachedImage<image::RGBfColor> pyramidImage;
        CachedImage<float> pyramidWeights;

        int width = _levels[_levels.size() - 1].getWidth();
        int height = _levels[_levels.size() - 1].getHeight();
        
        int nextWidth = width / 2;
        int nextHeight = int(ceil(float(height) / 2.0f));

        //If the level width is odd, it is a problem.
        //Let update this level to an even size. we'll manage it later
        //Store the real size to know we updated it
        _realWidths.push_back(nextWidth);
        if (nextWidth % 2)
        {
            nextWidth++;
        }

        if(!pyramidImage.createImage(cacheManager, nextWidth, nextHeight))
        {
            return false;
        }

        if(!pyramidWeights.createImage(cacheManager, nextWidth, nextHeight))
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
        int width = currentImage.getWidth();
        int height = currentImage.getHeight();
        int nextWidth = int(ceil(float(width) / 2.0f));
        int nextHeight = int(ceil(float(height) / 2.0f));


        // Create buffer for next level
        CachedImage<image::RGBfColor> nextImage;
        CachedImage<float> nextWeights;

        if(!nextImage.createImage(cacheManager, nextWidth, nextHeight))
        {
            return false;
        }

        if(!nextWeights.createImage(cacheManager, nextWidth, nextHeight))
        {
            return false;
        }

        nextImage.fill(image::RGBfColor(0.0f));
        nextWeights.fill(0.0f);


        //Process image rectangle by rectangle
        for (int y = 0; y < nextImage.getHeight(); y += processingSize)
        {
            for (int x = 0; x < nextImage.getWidth(); x += processingSize)
            {
                //Compute the initial bouding box for this rectangle
                BoundingBox nextBbox;
                nextBbox.left = x;
                nextBbox.top = y;
                nextBbox.width = processingSize;
                nextBbox.height = processingSize;
                nextBbox.clampLeft();
                nextBbox.clampTop();
                nextBbox.clampRight(nextImage.getWidth() - 1);
                nextBbox.clampBottom(nextImage.getHeight() - 1);


                // Dilate the bounding box to account for filtering
                BoundingBox dilatedNextBbox = nextBbox.dilate(borderSize);
                dilatedNextBbox.clampTop();
                dilatedNextBbox.clampBottom(nextImage.getHeight() - 1);

                //If the box has a negative left border,
                //it is equivalent (with the loop, to shifting at the end)
                int leftBorder = nextBbox.left - dilatedNextBbox.left;
                int topBorder = nextBbox.top - dilatedNextBbox.top;
                if (dilatedNextBbox.left < 0)
                {
                    dilatedNextBbox.left += nextImage.getWidth();
                }

                //Same box in the current level
                BoundingBox currentBbox = nextBbox.doubleSize();
                currentBbox.clampLeft();
                currentBbox.clampTop();
                currentBbox.clampRight(currentImage.getWidth() - 1);
                currentBbox.clampBottom(currentImage.getHeight() - 1);  

                //Dilated box in the current level
                BoundingBox dilatedCurrentBbox = dilatedNextBbox.doubleSize(); 
                currentBbox.clampTop();
                currentBbox.clampBottom(currentImage.getHeight() - 1);         

                //Extract the image with borders in the current level
                aliceVision::image::Image<image::RGBfColor> extractedColor(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                if (!loopyCachedImageExtract(extractedColor, currentImage, dilatedCurrentBbox)) 
                {
                    return false;
                }

                //Extract the weights with borders in the current level
                aliceVision::image::Image<float> extractedWeight(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                if (!loopyCachedImageExtract(extractedWeight, currentWeights, dilatedCurrentBbox)) 
                {
                    return false;
                }

                //Filter current image
                aliceVision::image::Image<image::RGBfColor> buf(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                aliceVision::image::Image<float> bufw(dilatedCurrentBbox.width, dilatedCurrentBbox.height);
                convolveGaussian5x5<image::RGBfColor>(buf, extractedColor);
                convolveGaussian5x5<float>(bufw, extractedWeight);
                
                //Downscale current image to next level
                aliceVision::image::Image<image::RGBfColor> colorDownscaled(dilatedNextBbox.width, dilatedNextBbox.height);
                aliceVision::image::Image<float> weightDownscaled(dilatedNextBbox.width, dilatedNextBbox.height);
                downscale(colorDownscaled, buf);
                downscale(weightDownscaled, bufw);

                BoundingBox saveBoundingBox;
                saveBoundingBox.left = leftBorder;
                saveBoundingBox.top = topBorder;
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

                saveBoundingBox.left = leftBorder * 2;
                saveBoundingBox.top = topBorder * 2;
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



bool LaplacianPyramid::apply(const aliceVision::image::Image<image::RGBfColor>& source,
                             const aliceVision::image::Image<unsigned char>& mask,
                             const aliceVision::image::Image<float>& weights, 
                             size_t offsetX, size_t offsetY)
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


    image::Image<image::RGBfColor> currentColor = source;
    image::Image<image::RGBfColor> nextColor;
    image::Image<float> currentWeights = weights;
    image::Image<float> nextWeights;
    image::Image<float> currentMask = mask_float;
    image::Image<float> nextMask;

    for(int l = 0; l < _levels.size() - 1; l++)
    {
        CachedImage<image::RGBfColor> & img = _levels[l];

        BoundingBox inputBbox;
        inputBbox.left = offsetX;
        inputBbox.top = offsetY;
        inputBbox.width = width;
        inputBbox.height = height;

        //If the destination image size is odd,
        //It is a big problem
        bool specialLoop = false;
        int loopPosition = std::numeric_limits<int>::max();
        int realLevelWidth = _realWidths[l];
        int offset = 0;

        if (img.getWidth() != realLevelWidth) 
        {
            //Do we cross the loop ?
            if (inputBbox.getRight() > img.getWidth())
            {
                specialLoop = true;
                loopPosition = realLevelWidth - inputBbox.left;
                offset = 2;
            }
        }

        //Create aligned images if necessary
        if (specialLoop)
        {
            image::Image<image::RGBfColor> alignedColor(width + offset, height);
            image::Image<float> alignedWeights(width + offset, height);
            image::Image<float> alignedMask(width + offset, height);
            
            for(int i = 0; i < currentColor.Height(); i++)
            {
                int dj = 0;

                for(int j = 0; j < currentColor.Width(); j++)
                {
                    if (j == loopPosition)
                    {
                        dj++;
                    }

                    alignedMask(i, dj) = currentMask(i, j);
                    alignedColor(i, dj) = currentColor(i, j);
                    alignedWeights(i, dj) = currentWeights(i, j);

                    dj++;
                }

                if (specialLoop)
                {
                    alignedColor(i, loopPosition) = alignedColor(i, loopPosition + 1);
                    alignedMask(i, loopPosition) = alignedMask(i, loopPosition + 1);
                    alignedWeights(i, loopPosition) = alignedWeights(i, loopPosition + 1);
                    alignedMask(i, width + 1) = 0;
                    alignedWeights(i, width + 1) = 0;
                }
            }

            currentColor = alignedColor;
            currentWeights = alignedWeights;
            currentMask = alignedMask;
        }


        image::Image<image::RGBfColor> bufMasked(width + offset, height);
        image::Image<image::RGBfColor> buf(width + offset, height);
        image::Image<image::RGBfColor> buf2(width + offset, height);
        image::Image<float> bufFloat(width + offset, height);

        /*Apply mask to content before convolution*/
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
        if (!merge(currentColor, currentWeights, l, offsetX, offsetY)) 
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

    if (!merge(currentColor, currentWeights, _levels.size() - 1, offsetX, offsetY))
    {
        return false;
    }

    return true;
}

bool LaplacianPyramid::merge(const aliceVision::image::Image<image::RGBfColor>& oimg,
                             const aliceVision::image::Image<float>& oweight, size_t level, size_t offsetX,
                             size_t offsetY)
{
    CachedImage<image::RGBfColor> & img = _levels[level];
    CachedImage<float> & weight = _weights[level];

    aliceVision::image::Image<image::RGBfColor> extractedColor(oimg.Width(), oimg.Height());
    aliceVision::image::Image<float> extractedWeight(oimg.Width(), oimg.Height());

    BoundingBox extractBb;
    extractBb.left = offsetX;
    extractBb.top = offsetY;
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

    for(int y = 0; y < inputBb.height; y++)
    {   
        for(int x = 0; x < inputBb.width; x++)
        {  
            extractedColor(y, x).r() += oimg(y, x).r() * oweight(y, x);
            extractedColor(y, x).g() += oimg(y, x).g() * oweight(y, x);
            extractedColor(y, x).b() += oimg(y, x).b() * oweight(y, x);
            extractedWeight(y, x) += oweight(y, x);
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

                if (!upscale(buf, extracted)) 
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
