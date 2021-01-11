#pragma once

#include "imageOps.hpp"

#include <aliceVision/image/all.hpp>

namespace aliceVision
{

class LaplacianPyramid
{
public:
    struct InputInfo 
    {
        aliceVision::image::Image<image::RGBfColor> color; 
        aliceVision::image::Image<float> mask;
        aliceVision::image::Image<float> weights;
        int offsetX;
        int offsetY;
    };

public:
    LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels);

    bool initialize();

    bool augment(size_t new_max_levels);
    
    bool apply(const aliceVision::image::Image<image::RGBfColor>& source,
               const aliceVision::image::Image<float>& mask, 
               const aliceVision::image::Image<float>& weights,
               size_t initialLevel, int offset_x, int offset_y);

    bool merge(const aliceVision::image::Image<image::RGBfColor>& oimg, 
               const aliceVision::image::Image<float>& oweight,
               size_t level, int offset_x, int offset_y);

    bool rebuild(image::Image<image::RGBAfColor>& output, const BoundingBox & roi);

private:
    int _baseWidth;
    int _baseHeight;
    int _maxLevels;

    std::vector<image::Image<image::RGBfColor>> _levels;
    std::vector<image::Image<float>> _weights;
    std::vector<InputInfo> _inputInfos;
};

} // namespace aliceVision