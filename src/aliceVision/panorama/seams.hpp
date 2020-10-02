#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/image/all.hpp>

#include "cachedImage.hpp"
#include "graphcut.hpp"

namespace aliceVision
{

void drawBorders(aliceVision::image::Image<image::RGBAfColor>& inout, aliceVision::image::Image<unsigned char>& mask,
                 size_t offset_x, size_t offset_y);
void drawSeams(aliceVision::image::Image<image::RGBAfColor>& inout, aliceVision::image::Image<IndexT>& labels);

void getMaskFromLabels(aliceVision::image::Image<float> & mask, aliceVision::image::Image<IndexT> & labels, IndexT index, size_t offset_x, size_t offset_y);

class WTASeams
{
public:
    WTASeams(size_t outputWidth, size_t outputHeight)
        : _panoramaWidth(outputWidth)
        , _panoramaHeight(outputHeight)
    {
    }

    virtual ~WTASeams() = default;

    bool initialize(image::TileCacheManager::shared_ptr & cacheManager);

    bool append(const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, IndexT currentIndex, size_t offset_x,
                        size_t offset_y);

    CachedImage<IndexT> & getLabels() 
    {
        return _labels; 
    }

private:
    CachedImage<float> _weights;
    CachedImage<IndexT> _labels;

    int _panoramaWidth;
    int _panoramaHeight;
};

class HierarchicalGraphcutSeams
{
public:
    HierarchicalGraphcutSeams(size_t outputWidth, size_t outputHeight, size_t levelOfInterest)
        : _outputWidth(outputWidth)
        , _outputHeight(outputHeight)
        , _levelOfInterest(levelOfInterest)
        , _labels(outputWidth, outputHeight, true, UndefinedIndexT)
    {

        double scale = 1.0 / pow(2.0, levelOfInterest);
        size_t width = size_t(floor(double(outputWidth) * scale));
        size_t height = size_t(floor(double(outputHeight) * scale));

        _graphcut = std::unique_ptr<GraphcutSeams>(new GraphcutSeams(width, height));
    }

    virtual ~HierarchicalGraphcutSeams() = default;

    void setOriginalLabels(const image::Image<IndexT>& labels);

    void setMaximalDistance(int distance) { _graphcut->setMaximalDistance(distance); }

    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& input,
                        const aliceVision::image::Image<unsigned char>& inputMask, IndexT currentIndex, size_t offset_x,
                        size_t offset_y);

    bool process();

    const image::Image<IndexT>& getLabels() { return _labels; }

private:
    std::unique_ptr<GraphcutSeams> _graphcut;
    image::Image<IndexT> _labels;
    size_t _levelOfInterest;
    size_t _outputWidth;
    size_t _outputHeight;
};

} // namespace aliceVision