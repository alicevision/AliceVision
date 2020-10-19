#include "seams.hpp"

#include "gaussian.hpp"
#include "imageOps.hpp"
#include "compositer.hpp"
#include "feathering.hpp"

namespace aliceVision
{

void drawBorders(aliceVision::image::Image<image::RGBAfColor>& inout, aliceVision::image::Image<unsigned char>& mask,
                 size_t offset_x, size_t offset_y)
{

    for(int i = 0; i < mask.Height(); i++)
    {
        int j = 0;
        int di = i + offset_y;
        int dj = j + offset_x;
        if(dj >= inout.Width())
        {
            dj = dj - inout.Width();
        }

        if(mask(i, j))
        {
            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int i = 0; i < mask.Height(); i++)
    {
        int j = mask.Width() - 1;
        int di = i + offset_y;
        int dj = j + offset_x;
        if(dj >= inout.Width())
        {
            dj = dj - inout.Width();
        }

        if(mask(i, j))
        {
            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int j = 0; j < mask.Width(); j++)
    {
        int i = 0;
        int di = i + offset_y;
        int dj = j + offset_x;
        if(dj >= inout.Width())
        {
            dj = dj - inout.Width();
        }

        if(mask(i, j))
        {
            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int j = 0; j < mask.Width(); j++)
    {
        int i = mask.Height() - 1;
        int di = i + offset_y;
        int dj = j + offset_x;
        if(dj >= inout.Width())
        {
            dj = dj - inout.Width();
        }

        if(mask(i, j))
        {
            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int i = 1; i < mask.Height() - 1; i++)
    {

        int di = i + offset_y;

        for(int j = 1; j < mask.Width() - 1; j++)
        {

            int dj = j + offset_x;
            if(dj >= inout.Width())
            {
                dj = dj - inout.Width();
            }

            if(!mask(i, j))
                continue;

            unsigned char others = true;
            others &= mask(i - 1, j - 1);
            others &= mask(i - 1, j + 1);
            others &= mask(i, j - 1);
            others &= mask(i, j + 1);
            others &= mask(i + 1, j - 1);
            others &= mask(i + 1, j + 1);
            if(others)
                continue;

            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }
}

void drawSeams(aliceVision::image::Image<image::RGBAfColor>& inout, aliceVision::image::Image<IndexT>& labels)
{

    for(int i = 1; i < labels.Height() - 1; i++)
    {

        for(int j = 1; j < labels.Width() - 1; j++)
        {

            IndexT label = labels(i, j);
            IndexT same = true;

            same &= (labels(i - 1, j - 1) == label);
            same &= (labels(i - 1, j + 1) == label);
            same &= (labels(i, j - 1) == label);
            same &= (labels(i, j + 1) == label);
            same &= (labels(i + 1, j - 1) == label);
            same &= (labels(i + 1, j + 1) == label);

            if(same)
            {
                continue;
            }

            inout(i, j) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
        }
    }
}

bool WTASeams::initialize(image::TileCacheManager::shared_ptr & cacheManager) 
{
    if(!_weights.createImage(cacheManager, _panoramaWidth, _panoramaHeight))
    {
        return false;
    }

    if(!_weights.fill(0.0f))
    {
        return false;
    }

    if(!_labels.createImage(cacheManager, _panoramaWidth, _panoramaHeight))
    {
        return false;
    }

    if(!_labels.fill(UndefinedIndexT))
    {
        return false;
    }

    return true;
}

bool WTASeams::append(const aliceVision::image::Image<unsigned char>& inputMask,
                      const aliceVision::image::Image<float>& inputWeights, IndexT currentIndex, size_t offset_x,
                      size_t offset_y)
{
    if( inputMask.size() != inputWeights.size())
    {
        return false;
    }

    aliceVision::image::Image<float> weights(inputMask.Width(), inputMask.Height());
    aliceVision::image::Image<IndexT> labels(inputMask.Width(), inputMask.Height());

    BoundingBox globalBb;
    globalBb.left = offset_x;
    globalBb.top = offset_y;
    globalBb.width = inputMask.Width();
    globalBb.height = inputMask.Height();

    if (!loopyCachedImageExtract(weights, _weights, globalBb)) 
    {
        return false;
    }

    if (!loopyCachedImageExtract(labels, _labels, globalBb)) 
    {
        return false;
    }
    

    for(size_t i = 0; i < weights.Height(); i++)
    {
        for(size_t j = 0; j < weights.Width(); j++)
        {
            if(!inputMask(i, j))
            {
                continue;
            }

            if (inputWeights(i, j) > weights(i, j))
            {
                labels(i, j) = currentIndex;
                weights(i, j) = inputWeights(i, j);
            }
        }
    }

    BoundingBox inputBb;
    inputBb.left = 0;
    inputBb.top = 0;
    inputBb.width = labels.Width();
    inputBb.height = labels.Height();

    if (!loopyCachedImageAssign(_weights, weights, globalBb, inputBb)) {
        return false;
    }

    if (!loopyCachedImageAssign(_labels, labels, globalBb, inputBb)) {
        return false;
    }

    return true;
}

bool HierarchicalGraphcutSeams::setOriginalLabels(CachedImage<IndexT>& labels)
{
    
    if (!_graphcuts[0].setOriginalLabels(labels)) 
    {
        return false;
    }

    for (int l = 1; l < _countLevels; l++)
    {

        CachedImage<IndexT> & largerLabels = _graphcuts[l - 1].getLabels();
        CachedImage<IndexT> & smallerLabels = _graphcuts[l].getLabels();

        int processingSize = 256;

        for (int i = 0; i < smallerLabels.getHeight(); i+= processingSize)
        {
            for (int j = 0; j < smallerLabels.getWidth(); j+= processingSize)
            {
                BoundingBox smallBb;
                smallBb.left = j;
                smallBb.top = i;
                smallBb.width = processingSize;
                smallBb.height = processingSize;
                smallBb.clampRight(smallerLabels.getWidth() - 1);
                smallBb.clampBottom(smallerLabels.getHeight() - 1);

                BoundingBox smallInputBb;
                smallInputBb.left = 0;
                smallInputBb.top = 0;
                smallInputBb.width = smallBb.width;
                smallInputBb.height = smallBb.height;

                BoundingBox largeBb = smallBb.doubleSize();

                BoundingBox largeInputBb;
                largeInputBb.left = 0;
                largeInputBb.top = 0;
                largeInputBb.width = largeBb.width;
                largeInputBb.height = largeBb.height;

                image::Image<IndexT> largeView(largeBb.width, largeBb.height);
                if (!largerLabels.extract(largeView, largeInputBb, largeBb)) 
                {
                    return false;
                }

                image::Image<IndexT> smallView(smallBb.width, smallBb.height);
                downscale(smallView, largeView);

                if (!smallerLabels.assign(smallView, smallInputBb, smallBb))
                {
                    return false;
                }
            }
        }
    }
    
    return true;
}

bool HierarchicalGraphcutSeams::append(const aliceVision::image::Image<image::RGBfColor>& input,
                                       const aliceVision::image::Image<unsigned char>& inputMask, 
                                       IndexT currentIndex, size_t offsetX, size_t offsetY)
{
    // Make sure input is compatible with pyramid processing
    size_t newOffsetX, newOffsetY;
    aliceVision::image::Image<image::RGBfColor> potImage;
    aliceVision::image::Image<unsigned char> potMask;
    makeImagePyramidCompatible(potImage, newOffsetX, newOffsetY, input, offsetX, offsetY, _countLevels);
    makeImagePyramidCompatible(potMask, newOffsetX, newOffsetY, inputMask, offsetX, offsetY, _countLevels);

    // Fill Color images masked parts with fake but coherent info
    aliceVision::image::Image<image::RGBfColor> feathered;
    if (!feathering(feathered, potImage, potMask)) 
    {
        return false;
    }

    int width = feathered.Width();
    int height = feathered.Height();

    for (int level = 0; level < _countLevels; level++)
    {
        BoundingBox bb;
        bb.left = 0;
        bb.top = 0;
        bb.width = width;
        bb.height = height;

        //Create cached image and assign image
        CachedImage<image::RGBfColor> destColor;
        if (!destColor.createImage(_cacheManager, width, height)) 
        {
            return false;
        }

        if (!destColor.fill(image::RGBfColor(0.0f)))
        {
            return false;
        }

        if (!destColor.assign(feathered, bb, bb)) 
        {
            return false;
        }

        //Create cached mask and assign content
        CachedImage<unsigned char> destMask;
        if (!destMask.createImage(_cacheManager, width, height)) 
        {
            return false;
        }

        if (!destMask.fill(0))
        {
            return false;
        }

        if (!destMask.assign(potMask, bb, bb)) 
        {
            return false;
        }

        //Assign content to graphcut
        if (!_graphcuts[level].append(destColor, destMask, currentIndex, newOffsetX, newOffsetY))
        {
            return false;
        }

        if (level == _countLevels - 1) 
        {
            continue;
        }

        //Resize for next level
        newOffsetX = newOffsetX / 2;
        newOffsetY = newOffsetY / 2;
        int newWidth = int(ceil(float(width) / 2.0f));
        int newHeight = int(ceil(float(height) / 2.0f));

        aliceVision::image::Image<image::RGBfColor> nextImage(newWidth, newHeight);
        aliceVision::image::Image<unsigned char> nextMask(newWidth, newHeight);
        aliceVision::image::Image<image::RGBfColor> buf(width, height);

        //Convolve + divide
        convolveGaussian5x5<image::RGBfColor>(buf, feathered);
        downscale(nextImage, buf);

        //Just nearest neighboor divide for mask
        for(int i = 0; i < nextMask.Height(); i++)
        {
            int di = i * 2;

            for(int j = 0; j < nextMask.Width(); j++)
            {
                int dj = j * 2;

                if(potMask(di, dj) && potMask(di, dj + 1) 
                   && potMask(di + 1, dj) && potMask(di + 1, dj + 1))
                {
                    nextMask(i, j) = 255;
                }
                else
                {
                    nextMask(i, j) = 0;
                }
            }
        }

        
        std::swap(feathered, nextImage);
        std::swap(potMask, nextMask);

        width = newWidth;
        height = newHeight;
    }

    return true;
}

bool HierarchicalGraphcutSeams::process()
{  
    for (int level = _countLevels - 1; level >= 0; level--) 
    {
        ALICEVISION_LOG_INFO("Hierachical graphcut processing level #" << level);

        if (level < _countLevels - 1)
        {
            _graphcuts[level].setMaximalDistance(10);
        }

        if(!_graphcuts[level].process())
        {
            return false;
        }

        if (level == 0) 
        {
            return true;
        }
        
        //Enlarge result of this level to be an initialization for next level
        CachedImage<IndexT> & smallLabels = _graphcuts[level].getLabels();
        CachedImage<IndexT> & largeLabels = _graphcuts[level - 1].getLabels();

        const int processingSize = 256;

        for (int i = 0; i < smallLabels.getHeight(); i+= processingSize)
        {
            for (int j = 0; j < smallLabels.getWidth(); j+= processingSize)
            {
                BoundingBox smallBb;
                smallBb.left = j;
                smallBb.top = i;
                smallBb.width = processingSize;
                smallBb.height = processingSize;
                smallBb.clampRight(smallLabels.getWidth() - 1);
                smallBb.clampBottom(smallLabels.getHeight() - 1);

                BoundingBox smallInputBb = smallBb;
                smallInputBb.left = 0;
                smallInputBb.top = 0;
                

                image::Image<IndexT> smallView(smallBb.width, smallBb.height);
                if (!smallLabels.extract(smallView, smallInputBb, smallBb)) 
                {
                    return false;
                }

                BoundingBox largeBb = smallBb.doubleSize();
                BoundingBox largeInputBb = largeBb;
                largeInputBb.left = 0;
                largeInputBb.top = 0;

                image::Image<IndexT> largeView(largeBb.width, largeBb.height);
                for (int y = 0; y < largeBb.height; y++) 
                {
                    int hy = y / 2;
                    
                    for (int x = 0; x < largeBb.width; x++)
                    {
                        int hx = x / 2;
                        largeView(y, x) = smallView(hy, hx);
                    }
                }

                if (!largeLabels.assign(largeView, largeInputBb, largeBb))
                {
                    return false;
                }
            }
        }
    }


    return true;
}

bool HierarchicalGraphcutSeams::initialize() 
{
    int width = _outputWidth;
    int height = _outputHeight;

    for (int i = 0; i < _countLevels; i++)
    {
        GraphcutSeams gcs(width, height);

        if (!gcs.initialize(_cacheManager))
        {
            return false;
        }

        _graphcuts.push_back(gcs);

        width = int(ceil(float(width) / 2.0f));
        height = int(ceil(float(height) / 2.0f));
    }

    return true;
}

bool getMaskFromLabels(aliceVision::image::Image<float> & mask, CachedImage<IndexT> & labels, IndexT index, size_t offset_x, size_t offset_y) {

    image::Image<IndexT> extractedLabels(mask.Width(), mask.Height());

    BoundingBox bb;
    bb.left = offset_x;
    bb.top = offset_y;
    bb.width = mask.Width();
    bb.height = mask.Height();

    if (!loopyCachedImageExtract(extractedLabels, labels, bb))
    {
        return false;
    }

    for (int i = 0; i < extractedLabels.Height(); i++) 
    {
        for (int j = 0; j < extractedLabels.Width(); j++) 
        {
            if (extractedLabels(i, j) == index) 
            {
                mask(i, j) = 1.0f;
            }
            else 
            {
                mask(i, j) = 0.0f;
            }
        }
    }

    return true;
}

} // namespace aliceVision