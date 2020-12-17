#include "seams.hpp"

#include "gaussian.hpp"
#include "imageOps.hpp"
#include "compositer.hpp"
#include "feathering.hpp"

namespace aliceVision
{


bool computeSeamsMap(image::Image<unsigned char>& seams, const image::Image<IndexT>& labels)
{
    if(seams.size() != labels.size())
    {
        return false;
    }

    seams.fill(0);

    for(int j = 1; j < labels.Width() - 1; j++)
    {
        IndexT label = labels(0, j);
        IndexT same = true;

        same &= (labels(0, j - 1) == label);
        same &= (labels(0, j + 1) == label);
        same &= (labels(1, j - 1) == label);
        same &= (labels(1, j) == label);
        same &= (labels(1, j + 1) == label);

        if(same)
        {
            continue;
        }

        seams(0, j) = 255;
    }

    int lastrow = labels.Height() - 1;
    for(int j = 1; j < labels.Width() - 1; j++)
    {
        IndexT label = labels(lastrow, j);
        IndexT same = true;

        same &= (labels(lastrow - 1, j - 1) == label);
        same &= (labels(lastrow - 1, j + 1) == label);
        same &= (labels(lastrow, j - 1) == label);
        same &= (labels(lastrow, j) == label);
        same &= (labels(lastrow, j + 1) == label);

        if(same)
        {
            continue;
        }

        seams(lastrow, j) = 255;
    }

    for(int i = 1; i < labels.Height() - 1; i++)
    {

        for(int j = 1; j < labels.Width() - 1; j++)
        {

            IndexT label = labels(i, j);
            IndexT same = true;

            same &= (labels(i - 1, j - 1) == label);
            same &= (labels(i - 1, j) == label);
            same &= (labels(i - 1, j + 1) == label);
            same &= (labels(i, j - 1) == label);
            same &= (labels(i, j + 1) == label);
            same &= (labels(i + 1, j - 1) == label);
            same &= (labels(i + 1, j) == label);
            same &= (labels(i + 1, j + 1) == label);

            if(same)
            {
                continue;
            }

            seams(i, j) = 255;
        }
    }

    return true;
}

bool drawBorders(CachedImage<image::RGBAfColor>& inout, const aliceVision::image::Image<unsigned char>& mask, size_t offsetX, size_t offsetY)
{
    BoundingBox bb;
    bb.left = offsetX;
    bb.top = offsetY;
    bb.width = mask.Width();
    bb.height = mask.Height();

    const int border = 2;
    BoundingBox dilatedBb = bb.dilate(border);
    dilatedBb.clampTop();
    dilatedBb.clampBottom(inout.getHeight() - 1);
    int leftBorder = bb.left - dilatedBb.left;
    int topBorder = bb.top - dilatedBb.top;

    if (dilatedBb.left < 0)
    {
        dilatedBb.left += inout.getWidth();
    }

    image::Image<image::RGBAfColor> extractedColor(dilatedBb.width, dilatedBb.height);
    if (!loopyCachedImageExtract(extractedColor, inout, dilatedBb)) 
    {
        return false;
    }

    for(int i = 0; i < mask.Height(); i++)
    {
        int j = 0;
        int di = i + topBorder;
        int dj = j + leftBorder;

        if(mask(i, j))
        {
            extractedColor(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int i = 0; i < mask.Height(); i++)
    {
        int j = mask.Width() - 1;
        int di = i + topBorder;
        int dj = j + leftBorder;

        if(mask(i, j))
        {
            extractedColor(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int j = 0; j < mask.Width(); j++)
    {
        int i = 0;
        int di = i + topBorder;
        int dj = j + leftBorder;
        
        if(mask(i, j))
        {
            extractedColor(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int j = 0; j < mask.Width(); j++)
    {
        int i = mask.Height() - 1;
        int di = i + topBorder;
        int dj = j + leftBorder;

        if(mask(i, j))
        {
            extractedColor(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    for(int i = 1; i < mask.Height() - 1; i++)
    {
        int di = i + topBorder;

        for(int j = 1; j < mask.Width() - 1; j++)
        {
            int dj = j + leftBorder;

            if(!mask(i, j))
            {
                continue;
            }

            unsigned char others = true;
            others &= mask(i - 1, j - 1);
            others &= mask(i - 1, j + 1);
            others &= mask(i, j - 1);
            others &= mask(i, j + 1);
            others &= mask(i + 1, j - 1);
            others &= mask(i + 1, j + 1);
            if(others) {
                continue;
            }

            extractedColor(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }

    BoundingBox inputBb = dilatedBb;
    inputBb.left = 0;
    inputBb.top = 0;
    if (!loopyCachedImageAssign(inout, extractedColor, dilatedBb, inputBb)) 
    {
        return false;
    }

    return true;
}

bool drawSeams(CachedImage<image::RGBAfColor>& inout, CachedImage<IndexT>& labels)
{
    const int processingSize = 512;

    //Process image rectangle by rectangle
    for (int y = 0; y < inout.getHeight(); y += processingSize)
    {
        for (int x = 0; x < inout.getWidth(); x += processingSize)
        {
            //Compute the initial bouding box for this rectangle
            BoundingBox currentBbox;
            currentBbox.left = x;
            currentBbox.top = y;
            currentBbox.width = processingSize;
            currentBbox.height = processingSize;
            currentBbox.clampLeft();
            currentBbox.clampTop();
            currentBbox.clampRight(inout.getWidth() - 1);
            currentBbox.clampBottom(inout.getHeight() - 1);

            image::Image<image::RGBAfColor> extractedColor(currentBbox.width, currentBbox.height);
            if (!loopyCachedImageExtract(extractedColor, inout, currentBbox)) 
            {
                return false;
            }

            image::Image<IndexT> extractedLabels(currentBbox.width, currentBbox.height);
            if (!loopyCachedImageExtract(extractedLabels, labels, currentBbox)) 
            {
                return false;
            }

            for(int i = 1; i < extractedLabels.Height() - 1; i++)
            {

                for(int j = 1; j < extractedLabels.Width() - 1; j++)
                {

                    IndexT label = extractedLabels(i, j);
                    IndexT same = true;

                    same &= (extractedLabels(i - 1, j - 1) == label);
                    same &= (extractedLabels(i - 1, j + 1) == label);
                    same &= (extractedLabels(i, j - 1) == label);
                    same &= (extractedLabels(i, j + 1) == label);
                    same &= (extractedLabels(i + 1, j - 1) == label);
                    same &= (extractedLabels(i + 1, j + 1) == label);

                    if(same)
                    {
                        continue;
                    }

                    extractedColor(i, j) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
                }
            }
            
            BoundingBox inputBbox = currentBbox;
            inputBbox.left = 0;
            inputBbox.top = 0;
            if (!loopyCachedImageAssign(inout, extractedColor, currentBbox, inputBbox)) 
            {
                return false;
            }
        }
    }

    return true;
}

bool WTASeams::append(const aliceVision::image::Image<unsigned char>& inputMask,
                      const aliceVision::image::Image<float>& inputWeights, 
                      IndexT currentIndex, size_t offset_x, size_t offset_y)
{
    if (inputMask.size() != inputWeights.size())
    {
        return false;
    }    

    for (size_t i = 0; i < inputWeights.Height(); i++)
    {
        int y = i + offset_y;
        if (y < 0 || y >= _panoramaHeight) continue;

        for (size_t j = 0; j < inputWeights.Width(); j++)
        {
            int x = j + offset_x;
            if (x < 0 || x >= _panoramaWidth) continue;

            if (!inputMask(i, j))
            {
                continue;
            }

            if (inputWeights(i, j) > _weights(y, x))
            {
                _labels(y, x) = currentIndex;
                _weights(y, x) = inputWeights(i, j);
            }
        }
    }

    return true;
}

bool HierarchicalGraphcutSeams::setOriginalLabels(CachedImage<IndexT>& labels)
{
    
    if (!_graphcuts[0].setOriginalLabels(labels)) 
    {
        return false;
    }

    //
    // Simply downscale the labels
    // [a b e f ]
    // [c d g h ] --> [a e]

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
                largeBb.clampRight(largerLabels.getWidth() - 1);
                largeBb.clampBottom(largerLabels.getHeight() - 1);

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
    int newOffsetX, newOffsetY;
    aliceVision::image::Image<image::RGBfColor> potImage;
    aliceVision::image::Image<unsigned char> potMask;
    makeImagePyramidCompatible(potImage, newOffsetX, newOffsetY, input, offsetX, offsetY, 2, _countLevels);
    makeImagePyramidCompatible(potMask, newOffsetX, newOffsetY, inputMask, offsetX, offsetY, 2, _countLevels);

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

                bool valid = potMask(di, dj);

                /*if (j < nextMask.Width() - 1) 
                {
                    valid = valid && potMask(di, dj + 1);
                }

                if (i < nextMask.Height() - 1)
                {
                    valid = valid && potMask(di + 1, dj);

                    if (j < nextMask.Width() - 1) 
                    {
                        valid = valid && potMask(di + 1, dj + 1);
                    }
                }*/

                if (valid)
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

        CachedImage<IndexT> & smallLabels = _graphcuts[level].getLabels();
        int w = smallLabels.getWidth();
        int h = smallLabels.getHeight();
        
        if (level == _countLevels - 1)
        {
            _graphcuts[level].setMaximalDistance(w + h);
        }
        else 
        {
            _graphcuts[level].setMaximalDistance(200);
        }

        bool computeAlphaExpansion = false;
        if (w < 5000) 
        {
            computeAlphaExpansion = true;
        }

        if(!_graphcuts[level].process(computeAlphaExpansion))
        {
            return false;
        }

        if (level == 0) 
        {
            return true;
        }
        
        //Enlarge result of this level to be an initialization for next level
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
                largeBb.clampRight(largeLabels.getWidth() - 1);
                largeBb.clampBottom(largeLabels.getHeight() - 1);

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

        //Divide by 2 (rounding to the superior integer)
        width = int(ceil(float(width) / 2.0f));
        height = int(ceil(float(height) / 2.0f));
    }

    return true;
}

bool getMaskFromLabels(aliceVision::image::Image<float> & mask, image::Image<IndexT> & labels, IndexT index, int offset_x, int offset_y) 
{

    for (int i = 0; i < mask.Height(); i++) 
    {
        int y = i + offset_y;

        for (int j = 0; j < mask.Width(); j++) 
        {
            int x = j + offset_x;
            mask(i, j) = 0;

            if (y < 0 || y >= labels.Height()) continue;
            if (x < 0 || x >= labels.Width()) continue;

            if (labels(y, x) == index) 
            {
                mask(i, j) = 1.0f;
            }
        }
    }

    return true;
}

} // namespace aliceVision
