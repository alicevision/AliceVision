#include "seams.hpp"

#include "gaussian.hpp"
#include "imageOps.hpp"
#include "compositer.hpp"

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
    if (_levelOfInterest == 0)
    {    
        return _graphcut->setOriginalLabels(labels);
    }
    
    int scale = pow(2, _levelOfInterest);
    int nw = _outputWidth / scale;
    int nh = _outputHeight / scale;

    CachedImage<IndexT> smallLabels;
    if(!smallLabels.createImage(_cacheManager, nw, nh))
    {
        return false;
    }

    int processingSize = 256;
    int largeSize = 256 * scale;

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

            BoundingBox smallInputBb;
            smallInputBb.left = 0;
            smallInputBb.top = 0;
            smallInputBb.width = smallBb.width;
            smallInputBb.height = smallBb.height;
            

            image::Image<IndexT> smallView(smallBb.width, smallBb.height);
            
            if (!smallLabels.extract(smallView, smallInputBb, smallBb)) 
            {
                return false;
            }

            BoundingBox largeBb;
            largeBb.left = smallBb.left * scale;
            largeBb.top = smallBb.top * scale;
            largeBb.width = smallBb.width * scale;
            largeBb.height = smallBb.height * scale;

            BoundingBox largeInputBb;
            largeInputBb.left = 0;
            largeInputBb.top = 0;
            largeInputBb.width = largeBb.width;
            largeInputBb.height = largeBb.height;

            image::Image<IndexT> largeView(largeBb.width, largeBb.height);
            if (!labels.extract(largeView, largeInputBb, largeBb)) 
            {
                return false;
            }

            for (int y = 0; y < smallBb.height; y++) 
            {
                for (int x = 0; x < smallBb.width; x++)
                {
                    smallView(y, x) = largeView(y * scale, x * scale);
                }
            }

            if (!smallLabels.assign(smallView, smallInputBb, smallBb))
            {
                return false;
            }
        }
    }

    return _graphcut->setOriginalLabels(smallLabels);
}

bool HierarchicalGraphcutSeams::append(const aliceVision::image::Image<image::RGBfColor>& input,
                                       const aliceVision::image::Image<unsigned char>& inputMask, 
                                       IndexT currentIndex, size_t offsetX, size_t offsetY)
{
    image::Image<image::RGBfColor> resizedColor;
    image::Image<unsigned char> resizedMask;

    int scale = pow(2, _levelOfInterest);
    int levelOffsetX = offsetX / scale;
    int levelOffsetY = offsetY / scale;

    if (!downscaleByPowerOfTwo(resizedColor, resizedMask, input, inputMask, _levelOfInterest))
    {
        return false;
    }

    BoundingBox bb;
    bb.left = 0;
    bb.top = 0;
    bb.width = resizedColor.Width();
    bb.height = resizedColor.Height();

    CachedImage<image::RGBfColor> destColor;
    if (!destColor.createImage(_cacheManager, resizedColor.Width(), resizedColor.Height())) 
    {
        return false;
    }

    if (!destColor.fill(image::RGBfColor(0.0f)))
    {
        return false;
    }

    if (!destColor.assign(resizedColor, bb, bb)) 
    {
        return false;
    }


    CachedImage<unsigned char> destMask;
    if (!destMask.createImage(_cacheManager, resizedMask.Width(), resizedMask.Height())) 
    {
        return false;
    }

    if (!destMask.fill(0))
    {
        return false;
    }

    if (!destMask.assign(resizedMask, bb, bb)) 
    {
        return false;
    }
    

    return _graphcut->append(destColor, destMask, currentIndex, levelOffsetX, levelOffsetY);
}

bool HierarchicalGraphcutSeams::process()
{  
    if(!_graphcut->process())
    {
        return false;
    }

    CachedImage<IndexT> smallLabels = _graphcut->getLabels();

    int scale = pow(2, _levelOfInterest);

    int processingSize = 256;
    int largeSize = 256 * scale;

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

            BoundingBox smallInputBb;
            smallInputBb.left = 0;
            smallInputBb.top = 0;
            smallInputBb.width = smallBb.width;
            smallInputBb.height = smallBb.height;
            

            image::Image<IndexT> smallView(smallBb.width, smallBb.height);
            if (!smallLabels.extract(smallView, smallInputBb, smallBb)) 
            {
                return false;
            }

            BoundingBox largeBb;
            largeBb.left = smallBb.left * scale;
            largeBb.top = smallBb.top * scale;
            largeBb.width = smallBb.width * scale;
            largeBb.height = smallBb.height * scale;

            BoundingBox largeInputBb;
            largeInputBb.left = 0;
            largeInputBb.top = 0;
            largeInputBb.width = largeBb.width;
            largeInputBb.height = largeBb.height;

            image::Image<IndexT> largeView(largeBb.width, largeBb.height);
            for (int y = 0; y < largeBb.height; y++) 
            {
                for (int x = 0; x < largeBb.width; x++)
                {
                    largeView(y, x) = smallView(y / scale, x / scale);
                }
            }

            if (!_labels.assign(largeView, largeInputBb, largeBb))
            {
                return false;
            }
        }
    }


    return true;
}

bool HierarchicalGraphcutSeams::initialize() 
{
    if (!_graphcut->initialize(_cacheManager))
    {
        return false;
    }

    if(!_labels.createImage(_cacheManager, _outputWidth, _outputHeight))
    {
        return false;
    }

    if(!_labels.fill(UndefinedIndexT))
    {
        return false;
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