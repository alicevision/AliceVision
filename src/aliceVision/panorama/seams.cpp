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

void drawBorders(aliceVision::image::Image<image::RGBAfColor>& inout, aliceVision::image::Image<unsigned char>& mask, int offset_x, int offset_y)
{
    for (int i = 0; i < mask.Height(); i++)
    {
        int j = 0;
        int di = i + offset_y;
        int dj = j + offset_x;

        if (di < 0 || dj < 0 || di >= inout.Height() || dj >= inout.Width())
        {
            continue;
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

        if (di < 0 || dj < 0 || di >= inout.Height() || dj >= inout.Width())
        {
            continue;
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
        if (di < 0 || dj < 0 || di >= inout.Height() || dj >= inout.Width())
        {
            continue;
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
        if (di < 0 || dj < 0 || di >= inout.Height() || dj >= inout.Width())
        {
            continue;
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
            if (di < 0 || dj < 0 || di >= inout.Height() || dj >= inout.Width())
            {
                continue;
            }

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
            if(others)
            {
                continue;
            }

            inout(di, dj) = image::RGBAfColor(0.0f, 1.0f, 0.0f, 1.0f);
        }
    }
}

void drawSeams(aliceVision::image::Image<image::RGBAfColor> & inout, aliceVision::image::Image<IndexT> & labels, int offset_x, int offset_y) {

    for (int i = 1; i < labels.Height() - 1; i++) 
    {
        int di = i + offset_y;
        if (di < 0 || di >= inout.Height()) continue;

        for (int j = 1; j < labels.Width() - 1; j++) 
        {
            int dj = j + offset_x;
            if (dj < 0 || dj >= inout.Width()) continue;

            IndexT label = labels(i, j);
            IndexT same = true;

            same &= (labels(i - 1, j - 1) == label);
            same &= (labels(i - 1, j + 1) == label);
            same &= (labels(i, j - 1) == label);
            same &= (labels(i, j + 1) == label);
            same &= (labels(i + 1, j - 1) == label);
            same &= (labels(i + 1, j + 1) == label);

            if (same) {
                continue;
            }

            inout(di, dj) = image::RGBAfColor(1.0f, 0.0f, 0.0f, 1.0f);
        }
    }
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

bool WTASeams::appendWithLoop(const aliceVision::image::Image<unsigned char>& inputMask,
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
            if (x < 0) 
            {
                x += _panoramaWidth;
            }
            if (x >= _panoramaWidth)
            {
                x -= _panoramaWidth;
            }

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

bool HierarchicalGraphcutSeams::initialize(const image::Image<IndexT> & labels) 
{
    int width = _outputWidth;
    int height = _outputHeight;

    for (int i = 0; i < _countLevels; i++)
    {
        _graphcuts.emplace_back(width, height);

        //Divide by 2 (rounding to the superior integer)
        width = int(ceil(float(width) / 2.0f));
        height = int(ceil(float(height) / 2.0f));
    }

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

        image::Image<IndexT> & largerLabels = _graphcuts[l - 1].getLabels();
        image::Image<IndexT> & smallerLabels = _graphcuts[l].getLabels();

        downscale(smallerLabels, largerLabels);
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
        //Assign content to graphcut
        if (!_graphcuts[level].append(feathered, potMask, currentIndex, newOffsetX, newOffsetY))
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

        image::Image<IndexT> & smallLabels = _graphcuts[level].getLabels();
        int w = smallLabels.Width();
        int h = smallLabels.Height();
        
        if (level == _countLevels - 1)
        {
            _graphcuts[level].setMaximalDistance(sqrt(w*w + h*h));
        }
        else 
        {
            double sw = double(0.2 * w);
            double sh = double(0.2 * h);
            _graphcuts[level].setMaximalDistance(sqrt(sw*sw + sh*sh));
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
        image::Image<IndexT> & largeLabels = _graphcuts[level - 1].getLabels();

        for (int y = 0; y < largeLabels.Height(); y++) 
        {
            int hy = y / 2;
            
            for (int x = 0; x < largeLabels.Width(); x++)
            {
                int hx = x / 2;
                largeLabels(y, x) = smallLabels(hy, hx);
            }
        }
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
