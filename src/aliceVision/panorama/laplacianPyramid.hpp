#pragma once

#include "imageOps.hpp"

namespace aliceVision
{

class LaplacianPyramid
{
public:
    LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels);
    bool augment(size_t new_max_levels);
    bool apply(const aliceVision::image::Image<image::RGBfColor>& source,
               const aliceVision::image::Image<unsigned char>& mask, const aliceVision::image::Image<float>& weights,
               size_t offset_x, size_t offset_y);
    bool merge(const aliceVision::image::Image<image::RGBfColor>& oimg, const aliceVision::image::Image<float>& oweight,
               size_t level, size_t offset_x, size_t offset_y);
    bool rebuild(image::Image<image::RGBAfColor>& output);

private:
    std::vector<aliceVision::image::Image<image::RGBfColor>> _levels;
    std::vector<aliceVision::image::Image<float>> _weights;
};

} // namespace aliceVision