#pragma once

#include "imageOps.hpp"

#include "cachedImage.hpp"

namespace aliceVision
{

class LaplacianPyramid
{
public:
    LaplacianPyramid(size_t base_width, size_t base_height, size_t max_levels);

    bool initialize(image::TileCacheManager::shared_ptr & cacheManager);

    bool augment(size_t new_max_levels);
    
    bool apply(const aliceVision::image::Image<image::RGBfColor>& source,
               const aliceVision::image::Image<unsigned char>& mask, const aliceVision::image::Image<float>& weights,
               size_t offset_x, size_t offset_y);

    bool merge(const aliceVision::image::Image<image::RGBfColor>& oimg, const aliceVision::image::Image<float>& oweight,
               size_t level, size_t offset_x, size_t offset_y);

    bool rebuild(CachedImage<image::RGBAfColor>& output);

private:
    int _baseWidth;
    int _baseHeight;
    int _maxLevels;

    std::vector<CachedImage<image::RGBfColor>> _levels;
    std::vector<CachedImage<float>> _weights;
};

} // namespace aliceVision