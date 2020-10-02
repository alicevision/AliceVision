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

    if(!_weights.perPixelOperation(
        [](float ) -> float
        { 
            return 0.0f; 
        }))
    {
        return false;
    }

    if(!_labels.createImage(cacheManager, _panoramaWidth, _panoramaHeight))
    {
        return false;
    }

    if(!_labels.perPixelOperation(
        [](IndexT ) -> IndexT
        { 
            return UndefinedIndexT; 
        }))
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

    if (!loopyCachedImageAssign(_weights, weights, globalBb)) {
        return false;
    }

    if (!loopyCachedImageAssign(_labels, labels, globalBb)) {
        return false;
    }

    return true;
}

void HierarchicalGraphcutSeams::setOriginalLabels(const image::Image<IndexT>& labels)
{

    /*
    First of all, Propagate label to all levels
    */
    image::Image<IndexT> current_label = labels;

    for(int l = 1; l <= _levelOfInterest; l++)
    {

        aliceVision::image::Image<IndexT> next_label(current_label.Width() / 2, current_label.Height() / 2);

        for(int i = 0; i < next_label.Height(); i++)
        {
            int di = i * 2;

            for(int j = 0; j < next_label.Width(); j++)
            {
                int dj = j * 2;

                next_label(i, j) = current_label(di, dj);
            }
        }

        current_label = next_label;
    }

    _graphcut->setOriginalLabels(current_label);
}

bool HierarchicalGraphcutSeams::append(const aliceVision::image::Image<image::RGBfColor>& input,
                                       const aliceVision::image::Image<unsigned char>& inputMask, IndexT currentIndex,
                                       size_t offset_x, size_t offset_y)
{
    image::Image<image::RGBfColor> current_color = input;
    image::Image<unsigned char> current_mask = inputMask;

    for(int l = 1; l <= _levelOfInterest; l++)
    {

        aliceVision::image::Image<image::RGBfColor> buf(current_color.Width(), current_color.Height());
        aliceVision::image::Image<image::RGBfColor> next_color(current_color.Width() / 2, current_color.Height() / 2);
        aliceVision::image::Image<unsigned char> next_mask(current_color.Width() / 2, current_color.Height() / 2);

        convolveGaussian5x5<image::RGBfColor>(buf, current_color);
        downscale(next_color, buf);

        for(int i = 0; i < next_mask.Height(); i++)
        {
            int di = i * 2;

            for(int j = 0; j < next_mask.Width(); j++)
            {
                int dj = j * 2;

                if(current_mask(di, dj) && current_mask(di, dj + 1) && current_mask(di + 1, dj) &&
                   current_mask(di + 1, dj + 1))
                {
                    next_mask(i, j) = 255;
                }
                else
                {
                    next_mask(i, j) = 0;
                }
            }
        }

        current_color = next_color;
        current_mask = next_mask;
        offset_x /= 2;
        offset_y /= 2;
    }

    return _graphcut->append(current_color, current_mask, currentIndex, offset_x, offset_y);
}

bool HierarchicalGraphcutSeams::process()
{

    if(!_graphcut->process())
    {
        return false;
    }

    image::Image<IndexT> current_labels = _graphcut->getLabels();

    for(int l = _levelOfInterest - 1; l >= 0; l--)
    {

        int nw = current_labels.Width() * 2;
        int nh = current_labels.Height() * 2;
        if(l == 0)
        {
            nw = _outputWidth;
            nh = _outputHeight;
        }

        aliceVision::image::Image<IndexT> next_label(nw, nh);
        for(int i = 0; i < nh; i++)
        {
            int hi = i / 2;

            for(int j = 0; j < nw; j++)
            {
                int hj = j / 2;

                next_label(i, j) = current_labels(hi, hj);
            }
        }

        current_labels = next_label;
    }

    _labels = current_labels;

    return true;
}

void getMaskFromLabels(aliceVision::image::Image<float> & mask, aliceVision::image::Image<IndexT> & labels, IndexT index, size_t offset_x, size_t offset_y) {

  for (int i = 0; i < mask.Height(); i++) {

    int di = i + offset_y;

    for (int j = 0; j < mask.Width(); j++) {

      int dj = j + offset_x;
      if (dj >= labels.Width()) {
        dj = dj - labels.Width();
      }


      if (labels(di, dj) == index) {
        mask(i, j) = 1.0f;
      }
      else {
        mask(i, j) = 0.0f;
      }
    }
  }
}

} // namespace aliceVision