// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "imageOps.hpp"

#include <aliceVision/image/all.hpp>

namespace aliceVision {

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

    virtual ~LaplacianPyramid();

    bool initialize();

    bool apply(aliceVision::image::Image<image::RGBfColor>& source,
               aliceVision::image::Image<float>& mask,
               aliceVision::image::Image<float>& weights,
               const BoundingBox& outputBoundingBox,
               const BoundingBox& contentBoudingBox);

    bool merge(const aliceVision::image::Image<image::RGBfColor>& oimg,
               const aliceVision::image::Image<float>& oweight,
               size_t level,
               int offset_x,
               int offset_y);

    bool rebuild(image::Image<image::RGBAfColor>& output, const BoundingBox& roi);

  private:
    int _baseWidth;
    int _baseHeight;
    int _maxLevels;
    omp_lock_t _merge_lock;

    std::vector<image::Image<image::RGBfColor>> _levels;
    std::vector<image::Image<float>> _weights;
    std::vector<InputInfo> _inputInfos;
};

}  // namespace aliceVision