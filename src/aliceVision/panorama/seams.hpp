#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/image/all.hpp>

#include "cachedImage.hpp"
#include "graphcut.hpp"

namespace aliceVision
{

void drawBorders(aliceVision::image::Image<image::RGBAfColor> & inout, aliceVision::image::Image<unsigned char> & mask, int offset_x, int offset_y);

void drawSeams(aliceVision::image::Image<image::RGBAfColor> & inout, aliceVision::image::Image<IndexT> & labels, int offset_x, int offset_y);

bool getMaskFromLabels(aliceVision::image::Image<float> & mask, image::Image<IndexT> & labels, IndexT index, int offset_x, int offset_y);

class WTASeams
{
public:
    WTASeams(size_t outputWidth, size_t outputHeight)
        : _weights(outputWidth, outputHeight, true, 0.0f)
        , _labels(outputWidth, outputHeight, true, UndefinedIndexT)
        , _panoramaWidth(outputWidth)
        , _panoramaHeight(outputHeight)
    {
    }

    virtual ~WTASeams() = default;
    
    bool append(const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        IndexT currentIndex, size_t offset_x, size_t offset_y);

    bool appendWithLoop(const aliceVision::image::Image<unsigned char>& inputMask,
                        const aliceVision::image::Image<float>& inputWeights, 
                        IndexT currentIndex, size_t offset_x, size_t offset_y);

    image::Image<IndexT> & getLabels() 
    {
        return _labels; 
    }

private:
    image::Image<float> _weights;
    image::Image<IndexT> _labels;

    int _panoramaWidth;
    int _panoramaHeight;
};

class HierarchicalGraphcutSeams
{
public:
    HierarchicalGraphcutSeams(size_t outputWidth, size_t outputHeight, size_t countLevels)
        : _outputWidth(outputWidth)
        , _outputHeight(outputHeight)
        , _countLevels(countLevels)
    {
    }

    virtual ~HierarchicalGraphcutSeams() = default;

    bool initialize(const image::Image<IndexT>& labels);

    virtual bool append(const aliceVision::image::Image<image::RGBfColor>& input,
                        const aliceVision::image::Image<unsigned char>& inputMask, 
                        IndexT currentIndex, size_t offset_x, size_t offset_y);

    bool process();

    image::Image<IndexT>& getLabels() 
    { 
        return _graphcuts[0].getLabels();
    }

private:
    std::vector<GraphcutSeams> _graphcuts;

    size_t _countLevels;
    size_t _outputWidth;
    size_t _outputHeight;
};

} // namespace aliceVision